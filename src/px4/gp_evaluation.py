#!/usr/bin/env python3
"""
GP Model Evaluation and Testing
==============================
Comprehensive evaluation of trained GP models with visualization.

Supports two formats:

1) Multi-output dict:
   {
     "models": {...},          # per-output GP models
     "scalers_input": {...},   # per-output input scalers
     "scalers_output": {...},  # per-output output scalers
     "training_stats": {...}
   }

2) Single-output container (your current file):
   {
     "gp_model": GaussianProcessRegressor,
     "training_count": ...,
     "data_points_used": ...,
     "timestamp": ...,
     "is_trained": ...
   }
"""

import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import pickle
from typing import Dict, Any
import warnings

warnings.filterwarnings("ignore")

plt.style.use("seaborn-v0_8")
sns.set_palette("husl")


class _IdentityScaler:
    """Do-nothing scaler with same interface as sklearn StandardScaler for our usage."""

    def __init__(self):
        self.scale_ = np.array([1.0])

    def transform(self, X):
        return X

    def inverse_transform(self, X):
        return X


class GPModelEvaluator:
    def __init__(self, model_path: str):
        self.model_path = model_path

        # Mode: "multi" (many outputs) or "single" (one GPRegressor)
        self.mode: str = ""

        # Multi-output case
        self.gp_models: Dict[str, Any] = {}
        self.scalers_X: Dict[str, Any] = {}
        self.scalers_y: Dict[str, Any] = {}

        # Single-output case
        self.gp_model = None  # GaussianProcessRegressor
        self.n_features: int = 0

        self.training_stats: Dict[str, Any] = {}
        self.test_data: Dict[str, np.ndarray] = {}

        self.load_model()

    # ------------------------------------------------------------------
    # MODEL LOADING
    # ------------------------------------------------------------------
    def load_model(self):
        with open(self.model_path, "rb") as f:
            model_data = pickle.load(f)

        keys = list(model_data.keys())
        print(f"📁 Loaded pickle. Top-level keys: {keys}")

        # --- Case 1: flat multi-output dict -----------------------------------
        if "models" in model_data:
            print("✅ Detected multi-output dict format ('models' + scalers).")
            self.mode = "multi"
            self.gp_models = model_data["models"]

            if "scalers_input" in model_data:
                self.scalers_X = model_data["scalers_input"]
            else:
                print("⚠️ No 'scalers_input' found. Using identity scalers.")
                self.scalers_X = {k: _IdentityScaler() for k in self.gp_models.keys()}

            if "scalers_output" in model_data:
                self.scalers_y = model_data["scalers_output"]
            else:
                print("⚠️ No 'scalers_output' found. Using identity scalers.")
                self.scalers_y = {k: _IdentityScaler() for k in self.gp_models.keys()}

            self.training_stats = model_data.get("training_stats", {})

            print(f"✅ GP models loaded. Outputs: {list(self.gp_models.keys())}")
            return

        # --- Case 2: container with a single GaussianProcessRegressor ---------
        if "gp_model" in model_data:
            gp_obj = model_data["gp_model"]
            print("✅ Detected gp_model container format.")
            print(f"   gp_model type: {type(gp_obj)}")

            # If it has 'predict', treat it as a single GPRegressor
            if hasattr(gp_obj, "predict"):
                self.mode = "single"
                self.gp_model = gp_obj

                # Save training stats from wrapper
                self.training_stats = {
                    "training_count": model_data.get("training_count", None),
                    "data_points_used": model_data.get("data_points_used", None),
                    "timestamp": model_data.get("timestamp", None),
                    "is_trained": model_data.get("is_trained", None),
                }

                # Number of features learned by the GP
                if hasattr(gp_obj, "n_features_in_"):
                    self.n_features = gp_obj.n_features_in_
                elif hasattr(gp_obj, "X_train_"):
                    self.n_features = gp_obj.X_train_.shape[1]
                else:
                    raise ValueError("gp_model has no 'n_features_in_' or 'X_train_'")

                print(f"✅ Single-output GP loaded with {self.n_features} input features.")
                return

            # otherwise unknown structure
            print("❌ gp_model does not look like a sklearn GPRegressor.")
            print("   Available attrs:", dir(gp_obj))
            raise KeyError("Unsupported gp_model type inside pickle")

        # --- Fallback: unknown structure --------------------------------------
        print(f"❌ Unknown model file structure. Keys: {keys}")
        raise KeyError("Model file does not contain 'models' or 'gp_model'")

    # ------------------------------------------------------------------
    # TEST DATA GENERATION
    # ------------------------------------------------------------------
    def generate_physical_test_data(self, n_samples: int = 1000) -> Dict[str, np.ndarray]:
        """
        Generate physically meaningful test data assuming 10 inputs:
        [x, y, z, vx, vy, vz, ax, ay, az, yaw_rate].
        """
        np.random.seed(42)

        position_range = (-10, 10)
        velocity_range = (-5, 5)
        accel_range = (-8, 8)
        yaw_rate_range = (-1, 1)

        base = {
            "x": np.random.uniform(*position_range, n_samples),
            "y": np.random.uniform(*position_range, n_samples),
            "z": np.random.uniform(-2, 15, n_samples),
            "vx": np.random.uniform(*velocity_range, n_samples),
            "vy": np.random.uniform(*velocity_range, n_samples),
            "vz": np.random.uniform(-3, 3, n_samples),
            "ax": np.random.uniform(*accel_range, n_samples),
            "ay": np.random.uniform(*accel_range, n_samples),
            "az": np.random.uniform(1, 18, n_samples),
            "yaw_rate": np.random.uniform(*yaw_rate_range, n_samples),
        }

        # Hover-like cases
        hover = {}
        positions = np.random.uniform(-5, 5, (100, 3))
        hover["x"] = positions[:, 0]
        hover["y"] = positions[:, 1]
        hover["z"] = positions[:, 2] + 5.0
        hover["vx"] = np.random.normal(0, 0.5, 100)
        hover["vy"] = np.random.normal(0, 0.5, 100)
        hover["vz"] = np.random.normal(0, 0.2, 100)
        hover["ax"] = np.random.normal(0, 2, 100)
        hover["ay"] = np.random.normal(0, 2, 100)
        hover["az"] = np.random.normal(9.81, 1, 100)
        hover["yaw_rate"] = np.random.normal(0, 0.3, 100)

        # Trajectory cases (figure-8)
        t = np.linspace(0, 10, 200)
        traj = {
            "x": 3 * np.sin(0.5 * t),
            "y": 3 * np.sin(t),
            "z": 5 + 2 * np.sin(0.3 * t),
        }
        traj["vx"] = 1.5 * np.cos(0.5 * t)
        traj["vy"] = 3.0 * np.cos(t)
        traj["vz"] = 0.6 * np.cos(0.3 * t)
        traj["ax"] = -0.75 * np.sin(0.5 * t) + np.random.normal(0, 1, 200)
        traj["ay"] = -3.0 * np.sin(t) + np.random.normal(0, 1, 200)
        traj["az"] = -0.18 * np.sin(0.3 * t) + 9.81 + np.random.normal(0, 0.5, 200)
        traj["yaw_rate"] = np.random.normal(0, 0.2, 200)

        for k in base.keys():
            base[k] = np.concatenate([base[k], hover[k], traj[k]])

        return base

    def generate_generic_test_data(self, n_samples: int = 2000) -> Dict[str, np.ndarray]:
        """
        Fallback if we don't know semantics of the features: generate
        'feature_0', 'feature_1', ..., 'feature_{d-1}' in [-1, 1].
        """
        np.random.seed(42)
        X = np.random.uniform(-1, 1, size=(n_samples, self.n_features))
        data = {f"feature_{i}": X[:, i] for i in range(self.n_features)}
        return data

    # ------------------------------------------------------------------
    # PREDICTION
    # ------------------------------------------------------------------
    def predict_on_test_data(self, test_data: Dict[str, np.ndarray]) -> Dict[str, Dict[str, np.ndarray]]:
        """Run GP predictions for all available outputs."""
        predictions: Dict[str, Dict[str, np.ndarray]] = {}

        if self.mode == "multi":
            # Build X_test with the known 10D order
            X_test = np.column_stack(
                [
                    test_data["x"],
                    test_data["y"],
                    test_data["z"],
                    test_data["vx"],
                    test_data["vy"],
                    test_data["vz"],
                    test_data["ax"],
                    test_data["ay"],
                    test_data["az"],
                    test_data["yaw_rate"],
                ]
            )

            for out_name, model in self.gp_models.items():
                scaler_in = self.scalers_X.get(out_name, _IdentityScaler())
                scaler_out = self.scalers_y.get(out_name, _IdentityScaler())

                X_scaled = scaler_in.transform(X_test)
                y_scaled, y_std_scaled = model.predict(X_scaled, return_std=True)

                # y_scaled can be (N,) or (N, n_targets) → flatten
                y_scaled = np.asarray(y_scaled)
                y_std_scaled = np.asarray(y_std_scaled)

                if y_scaled.ndim > 1:
                    y_scaled = y_scaled.reshape(-1)
                if y_std_scaled.ndim > 1:
                    y_std_scaled = y_std_scaled.reshape(-1)

                y = scaler_out.inverse_transform(y_scaled.reshape(-1, 1)).flatten()

                if hasattr(scaler_out, "scale_"):
                    y_std = y_std_scaled * scaler_out.scale_[0]
                else:
                    y_std = y_std_scaled

                y_std = np.asarray(y_std).reshape(-1)

                predictions[out_name] = {
                    "mean": y,
                    "std": np.abs(y_std),
                    "upper": y + 2 * np.abs(y_std),
                    "lower": y - 2 * np.abs(y_std),
                }

                print(
                    f"✅ Predicted {out_name}: mean={np.mean(y):.4f}, "
                    f"std={np.mean(y_std):.4f}"
                )

        elif self.mode == "single":
            # Single GPRegressor -> one output
            if self.n_features == 10 and all(
                k in test_data for k in ["x", "y", "z", "vx", "vy", "vz", "ax", "ay", "az", "yaw_rate"]
            ):
                X_test = np.column_stack(
                    [
                        test_data["x"],
                        test_data["y"],
                        test_data["z"],
                        test_data["vx"],
                        test_data["vy"],
                        test_data["vz"],
                        test_data["ax"],
                        test_data["ay"],
                        test_data["az"],
                        test_data["yaw_rate"],
                    ]
                )
            else:
                # Generic fallback: use numeric features in sorted key order
                feature_keys = sorted([k for k in test_data.keys() if k.startswith("feature_")])
                X_test = np.column_stack([test_data[k] for k in feature_keys])

            y_pred, y_std = self.gp_model.predict(X_test, return_std=True)

            # 🔧 Fix: ensure 1D arrays even if GP is multi-output
            y_pred = np.asarray(y_pred)
            y_std = np.asarray(y_std)
            if y_pred.ndim > 1:
                y_pred = y_pred.reshape(-1)
            if y_std.ndim > 1:
                y_std = y_std.reshape(-1)

            predictions["output"] = {
                "mean": y_pred,
                "std": np.abs(y_std),
                "upper": y_pred + 2 * np.abs(y_std),
                "lower": y_pred - 2 * np.abs(y_std),
            }

            print(
                f"✅ Predicted single output: mean={np.mean(y_pred):.4f}, "
                f"std={np.mean(y_std):.4f}"
            )

        else:
            print("⚠️ Unknown mode, no predictions.")
            return {}

        return predictions

    # ------------------------------------------------------------------
    # PLOTTING / ANALYSIS
    # ------------------------------------------------------------------
    def plot_prediction_distributions(self, predictions: Dict[str, Dict[str, np.ndarray]]):
        if not predictions:
            print("⚠️ No predictions to plot distributions.")
            return

        output_names = list(predictions.keys())
        n_out = len(output_names)
        cols = min(3, n_out)
        rows = int(np.ceil(n_out / cols))

        fig, axes = plt.subplots(rows, cols, figsize=(6 * cols, 4 * rows))
        if isinstance(axes, np.ndarray):
            axes = axes.flatten()
        else:
            axes = [axes]

        for i, name in enumerate(output_names):
            pred_data = predictions[name]
            mean_vals = np.asarray(pred_data["mean"]).ravel()
            std_vals = np.asarray(pred_data["std"]).ravel()

            axes[i].hist(
                mean_vals,
                bins=50,
                alpha=0.7,
                label="Predicted mean",
                color="skyblue",
                density=True,
            )
            axes[i].hist(
                std_vals,
                bins=50,
                alpha=0.7,
                label="Predicted std",
                color="orange",
                density=True,
            )
            axes[i].set_title(f"{name} predictions")
            axes[i].set_xlabel("Value")
            axes[i].set_ylabel("Density")
            axes[i].legend()
            axes[i].grid(True, alpha=0.3)

            mean_val = float(np.mean(mean_vals))
            std_val = float(np.mean(std_vals))
            axes[i].text(
                0.02,
                0.98,
                f"μ={mean_val:.4f}\nσ={std_val:.4f}",
                transform=axes[i].transAxes,
                va="top",
                bbox=dict(boxstyle="round", facecolor="white", alpha=0.8),
            )

        for j in range(len(output_names), len(axes)):
            axes[j].set_visible(False)

        plt.tight_layout()
        out_path = "/tmp/gp_prediction_distributions.png"
        plt.savefig(out_path, dpi=300, bbox_inches="tight")
        print(f"📊 Saved prediction distributions: {out_path}")
        plt.show()

    def plot_uncertainty_analysis(self, test_data: Dict[str, np.ndarray],
                                  predictions: Dict[str, Dict[str, np.ndarray]]):
        """Analyze and plot prediction uncertainty."""
        if not predictions:
            print("⚠️ No predictions for uncertainty analysis.")
            return

        # Only do the nice “velocity/accel vs uncertainty” plots if we have physical keys
        needed_keys = {"vx", "vy", "vz", "ax", "ay", "az", "z"}
        if not needed_keys.issubset(test_data.keys()):
            print("⚠️ Missing physical keys for uncertainty vs state plots. Skipping.")
            return

        # --- Build state magnitudes ---
        vx = np.asarray(test_data["vx"]).ravel()
        vy = np.asarray(test_data["vy"]).ravel()
        vz = np.asarray(test_data["vz"]).ravel()
        ax_ = np.asarray(test_data["ax"]).ravel()
        ay_ = np.asarray(test_data["ay"]).ravel()
        az_ = np.asarray(test_data["az"]).ravel()
        z_ = np.asarray(test_data["z"]).ravel()

        velocities = np.sqrt(vx**2 + vy**2 + vz**2)
        accelerations = np.sqrt(ax_**2 + ay_**2 + az_**2)

        # --- Average uncertainty across all outputs ---
        std_list = [np.asarray(p["std"]).ravel() for p in predictions.values()]
        avg_uncertainty = np.mean(std_list, axis=0)

        # 🔧 Make sure all arrays have the same length
        N = min(
            velocities.shape[0],
            accelerations.shape[0],
            z_.shape[0],
            avg_uncertainty.shape[0],
        )
        velocities = velocities[:N]
        accelerations = accelerations[:N]
        z_ = z_[:N]
        avg_uncertainty = avg_uncertainty[:N]

        fig, axes = plt.subplots(2, 2, figsize=(16, 12))

        # 1. Uncertainty vs Velocity
        axes[0, 0].scatter(velocities, avg_uncertainty, alpha=0.6, s=20)
        axes[0, 0].set_xlabel("Velocity magnitude [m/s]")
        axes[0, 0].set_ylabel("Average uncertainty")
        axes[0, 0].set_title("Uncertainty vs velocity")
        axes[0, 0].grid(True, alpha=0.3)

        # 2. Uncertainty vs Acceleration
        axes[0, 1].scatter(accelerations, avg_uncertainty, alpha=0.6, s=20)
        axes[0, 1].set_xlabel("Acceleration magnitude [m/s²]")
        axes[0, 1].set_ylabel("Average uncertainty")
        axes[0, 1].set_title("Uncertainty vs acceleration")
        axes[0, 1].grid(True, alpha=0.3)

        # 3. Uncertainty vs Height
        axes[1, 0].scatter(z_, avg_uncertainty, alpha=0.6, s=20)
        axes[1, 0].set_xlabel("Height [m]")
        axes[1, 0].set_ylabel("Average uncertainty")
        axes[1, 0].set_title("Uncertainty vs height")
        axes[1, 0].grid(True, alpha=0.3)

        # 4. Uncertainty histogram
        axes[1, 1].hist(avg_uncertainty, bins=50, alpha=0.7, color="green")
        axes[1, 1].set_xlabel("Average uncertainty")
        axes[1, 1].set_ylabel("Frequency")
        axes[1, 1].set_title("Uncertainty distribution")
        axes[1, 1].grid(True, alpha=0.3)

        plt.tight_layout()
        out_path = "/tmp/gp_uncertainty_analysis.png"
        plt.savefig(out_path, dpi=300, bbox_inches="tight")
        print(f"📊 Saved uncertainty analysis: {out_path}")
        plt.show()


    def plot_residual_correlations(self, predictions: Dict[str, Dict[str, np.ndarray]]):
        if not predictions:
            print("⚠️ No predictions for correlation analysis.")
            return None

        df = pd.DataFrame(
            {name: np.asarray(p["mean"]).ravel() for name, p in predictions.items()}
        )
        corr = df.corr()

        plt.figure(figsize=(8, 6))
        sns.heatmap(
            corr,
            annot=True,
            cmap="coolwarm",
            square=True,
            fmt=".3f",
            center=0.0,
        )
        plt.title("Residual correlations between outputs")
        plt.tight_layout()
        out_path = "/tmp/gp_residual_correlations.png"
        plt.savefig(out_path, dpi=300, bbox_inches="tight")
        print(f"📊 Saved residual correlation heatmap: {out_path}")
        plt.show()
        return corr

    def analyze_gp_performance(self, predictions: Dict[str, Dict[str, np.ndarray]]):
        if not predictions:
            print("⚠️ No predictions for performance analysis.")
            return

        print("\n" + "=" * 60)
        print("🚀 GP MODEL PERFORMANCE ANALYSIS")
        print("=" * 60)

        names = list(predictions.keys())
        print("\n📊 Model coverage:")
        print(f"   Available outputs: {len(names)}")
        print(f"   Outputs: {names}")

        print("\n📈 Prediction statistics:")
        for name, p in predictions.items():
            m_vals = np.asarray(p["mean"]).ravel()
            s_vals = np.asarray(p["std"]).ravel()
            mean_pred = float(np.mean(m_vals))
            std_pred = float(np.mean(s_vals))
            max_std = float(np.max(s_vals))
            print(
                f"   {name:12s}: μ={mean_pred:+.4f}, "
                f"σ_avg={std_pred:.4f}, σ_max={max_std:.4f}"
            )

        all_unc = np.concatenate([np.asarray(p["std"]).ravel() for p in predictions.values()])
        print("\n🎯 Uncertainty analysis:")
        print(f"   Mean uncertainty: {np.mean(all_unc):.4f}")
        print(f"   Max  uncertainty: {np.max(all_unc):.4f}")
        print(f"   90th percentile:  {np.percentile(all_unc, 90):.4f}")

        hi = np.mean(all_unc < 0.1)
        mid = np.mean((all_unc >= 0.1) & (all_unc < 0.5))
        lo = np.mean(all_unc >= 0.5)
        print("\n🎪 Confidence regions:")
        print(f"   High (σ < 0.1):        {hi:.1%}")
        print(f"   Medium (0.1 ≤ σ < 0.5): {mid:.1%}")
        print(f"   Low (σ ≥ 0.5):          {lo:.1%}")

        if self.training_stats:
            print("\n📚 Training information:")
            for k, v in self.training_stats.items():
                print(f"   {k}: {v}")

    # ------------------------------------------------------------------
    # MAIN PIPELINE
    # ------------------------------------------------------------------
    def run_complete_evaluation(self):
        print("🧪 Starting GP model evaluation...")

        # Decide which test data generator to use
        if self.mode == "multi" or (self.mode == "single" and self.n_features == 10):
            print("📊 Generating physical test data (10D state+control)…")
            test_data = self.generate_physical_test_data(n_samples=2000)
        else:
            print("📊 Generating generic test data (no physical semantics)…")
            test_data = self.generate_generic_test_data(n_samples=2000)

        self.test_data = test_data

        print("🔮 Running GP predictions…")
        predictions = self.predict_on_test_data(test_data)

        print("📈 Generating plots & summaries…")
        self.plot_prediction_distributions(predictions)
        self.plot_uncertainty_analysis(test_data, predictions)
        correlations = self.plot_residual_correlations(predictions)
        self.analyze_gp_performance(predictions)

        print("\n✅ Evaluation complete! Check saved plots in /tmp")
        return {"test_data": test_data, "predictions": predictions, "correlations": correlations}


def main():
    # Update this path if needed
    model_path = (
        "/home/grandediw/ros2_px4_offboard_example_ws/gp_models/"
        "gp_mpc_trained_20251125_223658.pkl"
    )
    evaluator = GPModelEvaluator(model_path)
    return evaluator.run_complete_evaluation()


if __name__ == "__main__":
    results = main()
