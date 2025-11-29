#!/usr/bin/env python3
"""
Offline evaluation of GP-enhanced quadrotor model.

What it does:
- Loads a trained GP model from:
    /home/grandediw/ros2_px4_offboard_example_ws/gp_models/gp_model_20251119_030043.pkl
- Loads flight dataset (CSV) from:
    /home/grandediw/ros2_px4_offboard_example_ws/gp_datasets/gp_mpc_data_20251124_225535.csv

CSV columns (as given):
    x, y, z, vx, vy, vz, ax, ay, az, yaw_rate,
    res_dx, res_dy, res_dz, res_dvx, res_dvy, res_dvz

We assume:
- Nominal dynamics (double integrator) are:
    xdot_nom = [vx, vy, vz, ax, ay, az]
- True residuals (what the GP was trained on) are:
    res = [res_dx, res_dy, res_dz, res_dvx, res_dvy, res_dvz]
- So true derivative is:
    xdot_true = xdot_nom + res

The GP model takes as input:
    [x, y, z, vx, vy, vz, ax, ay, az, yaw_rate]
and outputs a 6D residual vector.

This script compares:
- Nominal error:      err_nom = xdot_true - xdot_nom (= res)
- GP-corrected error: err_gp  = xdot_true - (xdot_nom + res_pred)

and prints:
- Global MSE / RMSE (all 6 components)
- MSE / RMSE for acceleration components only (vx_dot,vy_dot,vz_dot)
- Per-component table: MSE, RMSE, R² and % improvement for each residual
- Fraction of samples where GP improves squared error
- Saves metrics as CSV + LaTeX table for your report.
"""

import argparse
import pickle
import numpy as np
import pandas as pd
from pathlib import Path


# -------------------------------------------------------------------------
# 1. Nominal dynamics (must match what you use in MPC)
# -------------------------------------------------------------------------
def f_nominal(x, u):
    """
    Double integrator nominal model.

    x: (6,) = [px, py, pz, vx, vy, vz]
    u: (4,) = [ax_cmd, ay_cmd, az_cmd, yaw_rate_cmd]

    returns:
        xdot: (6,) = [vx, vy, vz, ax_cmd, ay_cmd, az_cmd]
    """
    vx, vy, vz = x[3:6]
    ax, ay, az = u[0], u[1], u[2]

    xdot = np.zeros(6, dtype=float)
    xdot[0:3] = [vx, vy, vz]
    xdot[3:6] = [ax, ay, az]
    return xdot


# -------------------------------------------------------------------------
# 2. Load GP model
# -------------------------------------------------------------------------
def load_gp_model(model_path: Path):
    """
    Load your GP model from pickle.

    It may be:
      - a dict with key 'gp_model'
      - the sklearn GP model itself
      - a sklearn Pipeline containing the GP

    We return the object that has a .predict(X) method.
    """
    with open(model_path, "rb") as f:
        obj = pickle.load(f)

    # If it's a dict like {'gp_model': <model>, 'training_count': ...}
    if isinstance(obj, dict) and "gp_model" in obj:
        gp = obj["gp_model"]
    else:
        gp = obj

    if not hasattr(gp, "predict"):
        raise RuntimeError(
            f"Loaded object of type {type(gp)} has no .predict() method. "
            "Check the contents of your pickle file."
        )

    print(f"✅ Loaded GP model from {model_path}")
    print(f"   Model type: {type(gp)}")
    return gp


# -------------------------------------------------------------------------
# 3. Load dataset
# -------------------------------------------------------------------------
def load_dataset(csv_path: Path):
    """
    Load the CSV dataset with columns:

    x,y,z,vx,vy,vz,ax,ay,az,yaw_rate,res_dx,res_dy,res_dz,res_dvx,res_dvy,res_dvz
    """
    df = pd.read_csv(csv_path)
    print(f"✅ Loaded dataset from {csv_path}")
    print(f"   Rows: {len(df)}, Columns: {list(df.columns)}")

    # Drop rows with NaNs just in case
    df = df.dropna().reset_index(drop=True)

    # Feature and residual columns
    feature_cols = [
        "x", "y", "z",
        "vx", "vy", "vz",
        "ax", "ay", "az",
        "yaw_rate",
    ]
    residual_cols = [
        "res_dx", "res_dy", "res_dz",
        "res_dvx", "res_dvy", "res_dvz",
    ]

    for c in feature_cols + residual_cols:
        if c not in df.columns:
            raise ValueError(f"Expected column '{c}' not found in CSV!")

    X_feat = df[feature_cols].to_numpy(dtype=float)        # (N, 10)
    R_true = df[residual_cols].to_numpy(dtype=float)      # (N, 6)

    # Extract x and u for nominal dynamics
    X_state = df[["x", "y", "z", "vx", "vy", "vz"]].to_numpy(dtype=float)  # (N, 6)
    U_ctrl = df[["ax", "ay", "az", "yaw_rate"]].to_numpy(dtype=float)      # (N, 4)

    return df, X_feat, R_true, X_state, U_ctrl


# -------------------------------------------------------------------------
# 4. Helper for R²
# -------------------------------------------------------------------------
def r2_score(y_true: np.ndarray, y_pred: np.ndarray) -> float:
    """
    Compute coefficient of determination R^2.
    """
    y_true = np.asarray(y_true, dtype=float)
    y_pred = np.asarray(y_pred, dtype=float)
    ss_res = np.sum((y_true - y_pred) ** 2)
    ss_tot = np.sum((y_true - np.mean(y_true)) ** 2)
    if ss_tot <= 1e-12:
        return np.nan
    return 1.0 - ss_res / ss_tot


# -------------------------------------------------------------------------
# 5. Evaluation
# -------------------------------------------------------------------------
def evaluate_gp(gp, X_feat, R_true, X_state, U_ctrl, save_prefix: Path = None):
    """
    Compare nominal vs GP-corrected dynamics.

    Inputs:
        gp      : trained GP model with .predict(X)
        X_feat  : (N, 10) features [x,y,z,vx,vy,vz,ax,ay,az,yaw_rate]
        R_true  : (N, 6) true residuals
        X_state : (N, 6) states [x,y,z,vx,vy,vz]
        U_ctrl  : (N, 4) controls [ax,ay,az,yaw_rate]

    We compute:
        xdot_nom  = f_nominal(x, u)          (N, 6)
        xdot_true = xdot_nom + R_true        (N, 6)
        R_pred    = gp.predict(X_feat)       (N, 6)
        xdot_gp   = xdot_nom + R_pred        (N, 6)

    Then:
        err_nom = xdot_true - xdot_nom (= R_true)
        err_gp  = xdot_true - xdot_gp  (= R_true - R_pred)

    and report MSEs, RMSEs, R², improvements, etc.
    """
    N = X_feat.shape[0]
    print(f"🔍 Evaluating GP on {N} samples...")

    # Nominal derivatives
    xdot_nom = np.zeros((N, 6), dtype=float)
    for i in range(N):
        xdot_nom[i] = f_nominal(X_state[i], U_ctrl[i])

    # True derivatives reconstructed from nominal + residual
    xdot_true = xdot_nom + R_true

    # GP prediction
    R_pred = gp.predict(X_feat)  # should be (N, 6)
    R_pred = np.asarray(R_pred)

    if R_pred.ndim == 1:
        # If it returns a flat array, try to reshape
        if R_pred.shape[0] == 6:
            R_pred = np.tile(R_pred, (N, 1))
        else:
            raise RuntimeError(
                f"GP predicted shape {R_pred.shape}, "
                "expected (N, 6) or (6,)."
            )

    if R_pred.shape[1] != 6:
        # If output dimension != 6, truncate or pad as needed
        print(f"⚠️ GP output dimension is {R_pred.shape[1]}, expected 6. Adjusting...")
        if R_pred.shape[1] > 6:
            R_pred = R_pred[:, :6]
        else:
            # pad with zeros at the end
            pad_width = 6 - R_pred.shape[1]
            R_pred = np.hstack([R_pred, np.zeros((N, pad_width), dtype=float)])

    # GP-corrected derivative
    xdot_gp = xdot_nom + R_pred

    # Errors
    err_nom = xdot_true - xdot_nom        # = R_true
    err_gp = xdot_true - xdot_gp          # = R_true - R_pred

    # Squared error per sample
    se_nom = np.sum(err_nom ** 2, axis=1)
    se_gp = np.sum(err_gp ** 2, axis=1)

    mse_nom = np.mean(se_nom)
    mse_gp = np.mean(se_gp)
    rmse_nom = np.sqrt(mse_nom)
    rmse_gp = np.sqrt(mse_gp)

    # Also look only at acceleration components (indices 3,4,5)
    err_nom_acc = err_nom[:, 3:6]
    err_gp_acc = err_gp[:, 3:6]
    se_nom_acc = np.sum(err_nom_acc ** 2, axis=1)
    se_gp_acc = np.sum(err_gp_acc ** 2, axis=1)
    mse_nom_acc = np.mean(se_nom_acc)
    mse_gp_acc = np.mean(se_gp_acc)
    rmse_nom_acc = np.sqrt(mse_nom_acc)
    rmse_gp_acc = np.sqrt(mse_gp_acc)

    # Relative improvement where positive means "GP better"
    rel_improvement = (mse_nom - mse_gp) / max(mse_nom, 1e-12) * 100.0
    rel_improvement_acc = (mse_nom_acc - mse_gp_acc) / max(mse_nom_acc, 1e-12) * 100.0

    print("\n=== GLOBAL RESULTS (all 6 components of xdot) ===")
    print(f"MSE nominal:        {mse_nom:.4e}")
    print(f"MSE GP-corrected:   {mse_gp:.4e}")
    print(f"RMSE nominal:       {rmse_nom:.4e}")
    print(f"RMSE GP-corrected:  {rmse_gp:.4e}")
    print(f"Relative improvement: {rel_improvement:.2f}%")

    print("\n=== ACCELERATION COMPONENTS ONLY (vx_dot, vy_dot, vz_dot) ===")
    print(f"MSE nominal (acc):  {mse_nom_acc:.4e}")
    print(f"MSE GP (acc):       {mse_gp_acc:.4e}")
    print(f"RMSE nominal (acc): {rmse_nom_acc:.4e}")
    print(f"RMSE GP (acc):      {rmse_gp_acc:.4e}")
    print(f"Relative improvement (acc): {rel_improvement_acc:.2f}%")

    # Fraction of samples where GP improves the error
    improvement_per_sample = se_nom - se_gp
    frac_better = np.mean(improvement_per_sample > 0.0)
    frac_worse = np.mean(improvement_per_sample < 0.0)
    frac_equal = 1.0 - frac_better - frac_worse
    print(f"\nFraction of samples where GP is better than nominal: {frac_better * 100.0:.1f}%")
    print(f"Fraction of samples where GP is worse:              {frac_worse * 100.0:.1f}%")
    print(f"Fraction of samples where equal:                    {frac_equal * 100.0:.1f}%")

    # ------------------------------------------------------------------
    # Per-component metrics: MSE, RMSE, R², improvement, frac_better
    # ------------------------------------------------------------------
    component_names = ["dx", "dy", "dz", "dvx", "dvy", "dvz"]
    rows = []

    for j, name in enumerate(component_names):
        e_nom_j = err_nom[:, j]
        e_gp_j = err_gp[:, j]

        mse_nom_j = np.mean(e_nom_j ** 2)
        mse_gp_j = np.mean(e_gp_j ** 2)
        rmse_nom_j = np.sqrt(mse_nom_j)
        rmse_gp_j = np.sqrt(mse_gp_j)
        improvement_j = (mse_nom_j - mse_gp_j) / max(mse_nom_j, 1e-12) * 100.0

        # For R², treat true residual as "target", nominal prediction = 0, GP prediction = R_pred
        y_true_j = R_true[:, j]
        y_nom_j = np.zeros_like(y_true_j)         # nominal residual = 0
        y_gp_j = R_pred[:, j]

        r2_nom_j = r2_score(y_true_j, y_nom_j)
        r2_gp_j = r2_score(y_true_j, y_gp_j)

        frac_better_j = np.mean(e_nom_j ** 2 > e_gp_j ** 2)

        rows.append({
            "component": name,
            "mse_nom": mse_nom_j,
            "mse_gp": mse_gp_j,
            "rmse_nom": rmse_nom_j,
            "rmse_gp": rmse_gp_j,
            "improvement_%": improvement_j,
            "r2_nom": r2_nom_j,
            "r2_gp": r2_gp_j,
            "frac_better": frac_better_j,
        })

    metrics_df = pd.DataFrame(rows)

    print("\n=== PER-COMPONENT METRICS (residuals) ===")
    # Nice formatted printing
    with pd.option_context('display.float_format', lambda x: f"{x: .3e}"):
        print(metrics_df.to_string(index=False))

        # ------------------------------------------------------------------
        # Save results for report (CSV + LaTeX)
        # ------------------------------------------------------------------
        if save_prefix is not None:
            # save_prefix is a Path; we build filenames like:
            # <basename>_metrics.csv and <basename>_metrics.tex
            base = save_prefix
            csv_out = base.parent / f"{base.name}_metrics.csv"
            tex_out = base.parent / f"{base.name}_metrics.tex"

            try:
                metrics_df.to_csv(csv_out, index=False)
                print(f"\n💾 Saved per-component metrics to: {csv_out}")
            except Exception as e:
                print(f"⚠️ Failed to save CSV metrics: {e}")

            try:
                # LaTeX table (scientific notation)
                metrics_df.to_latex(tex_out, index=False, float_format="%.3e")
                print(f"💾 Saved LaTeX table to: {tex_out}")
            except Exception as e:
                print(f"⚠️ Failed to save LaTeX table: {e}")


    # Optional: return everything if you want to post-process in Python
    return {
        "global": {
            "mse_nom": mse_nom,
            "mse_gp": mse_gp,
            "rmse_nom": rmse_nom,
            "rmse_gp": rmse_gp,
            "improvement_%": rel_improvement,
        },
        "acc_only": {
            "mse_nom": mse_nom_acc,
            "mse_gp": mse_gp_acc,
            "rmse_nom": rmse_nom_acc,
            "rmse_gp": rmse_gp_acc,
            "improvement_%": rel_improvement_acc,
        },
        "fractions": {
            "frac_better": frac_better,
            "frac_worse": frac_worse,
            "frac_equal": frac_equal,
        },
        "per_component": metrics_df,
    }


# -------------------------------------------------------------------------
# 6. Main
# -------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Offline evaluation of GP-enhanced quadrotor model.")
    parser.add_argument(
        "--model-path",
        type=str,
        default="/home/grandediw/ros2_px4_offboard_example_ws/gp_models/gp_model_20251119_030043.pkl",
        help="Path to the GP model pickle file."
    )
    parser.add_argument(
        "--data-path",
        type=str,
        default="/home/grandediw/ros2_px4_offboard_example_ws/gp_datasets/gp_mpc_data_20251124_225535.csv",
        help="Path to the flight data CSV file."
    )

    args = parser.parse_args()

    model_path = Path(args.model_path)
    data_path = Path(args.data_path)

    if not model_path.exists():
        raise FileNotFoundError(f"GP model not found at: {model_path}")
    if not data_path.exists():
        raise FileNotFoundError(f"Dataset CSV not found at: {data_path}")

    gp = load_gp_model(model_path)
    df, X_feat, R_true, X_state, U_ctrl = load_dataset(data_path)

    # Use data_path stem as prefix for saving metrics, e.g. gp_mpc_data_..._metrics.csv
    save_prefix = data_path.with_suffix("")  # remove .csv

    evaluate_gp(gp, X_feat, R_true, X_state, U_ctrl, save_prefix=save_prefix)


if __name__ == "__main__":
    main()
