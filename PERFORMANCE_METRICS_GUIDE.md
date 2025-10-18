# Performance Metrics and Visualization Guide

This guide explains how to use the GP and MPC performance metrics and visualization tools in your quadrotor control system.

## Overview

The `performance_metrics.py` module provides comprehensive tools for:

1. **Collecting Metrics**: Track performance during system operation
2. **Analyzing Performance**: Compute detailed metrics like RMSE, solve times, etc.
3. **Visualizing Results**: Generate professional plots of system performance
4. **Logging Data**: Save metrics to JSON for later analysis

## Quick Start

### 1. Run the Example

```bash
cd ~/quadrotor_gp_mpc_ws
source install/setup.bash
python3 -m quadrotor_gp_mpc.example_metrics
```

This will generate example plots and save them to `/tmp/`:
- `gp_metrics.png` - Gaussian Process performance
- `mpc_metrics.png` - Model Predictive Control performance
- `gp_mpc_comparison.png` - Comparison analysis

### 2. Integrate into Your System

#### For Gaussian Process Monitoring

```python
from quadrotor_gp_mpc.performance_metrics import GPMetrics

# Create metrics object
gp_metrics = GPMetrics()

# In your GP node's training loop:
gp_metrics.add_metrics(
    n_data=len(self.X_train),
    pred_err=prediction_errors,
    uncertainty=prediction_uncertainties,
    hyperparams={
        'length_scale': self.kernel.length_scale,
        'signal_variance': self.kernel.signal_variance
    },
    timestamp=current_time
)

# Periodically save and visualize
if time_step % 100 == 0:
    visualizer = PerformanceVisualizer()
    visualizer.plot_gp_metrics(gp_metrics, save_path='gp_metrics.png')
```

#### For MPC Monitoring

```python
from quadrotor_gp_mpc.performance_metrics import MPCMetrics

# Create metrics object
mpc_metrics = MPCMetrics()

# In your MPC controller's solve loop:
mpc_metrics.add_step(
    reference=reference_state,
    actual=current_state,
    control=control_input,
    tracking_error=current_state - reference_state,
    solve_time=optimization_time,
    constraint_violated=constraints_infeasible,
    timestamp=current_time
)

# Periodically visualize
if step % 50 == 0:
    visualizer = PerformanceVisualizer()
    visualizer.plot_mpc_metrics(mpc_metrics, save_path='mpc_metrics.png')
```

## Data Classes Reference

### GPMetrics

Tracks Gaussian Process learning performance.

**Key Attributes:**
- `training_data_count`: List of number of training points over time
- `prediction_errors`: List of prediction error arrays
- `uncertainties`: List of uncertainty variance arrays
- `hyperparameters`: List of hyperparameter dicts
- `timestamps`: Time steps

**Key Methods:**
- `add_metrics()`: Add metrics at a time step
- `rmse`: Root Mean Squared Error
- `mean_prediction_error`: Average error across all dimensions
- `mean_uncertainty`: Average uncertainty across all dimensions

**Example Usage:**
```python
gp_metrics = GPMetrics()
gp_metrics.add_metrics(100, np.random.randn(12), np.ones(12)*0.1, 
                       {'length_scale': 1.0}, timestamp=0.0)
print(f"RMSE: {gp_metrics.rmse()}")
print(f"Mean Uncertainty: {np.mean(gp_metrics.mean_uncertainty)}")
```

### MPCMetrics

Tracks Model Predictive Control tracking performance.

**Key Attributes:**
- `reference_trajectory`: List of reference states
- `actual_trajectory`: List of actual states
- `control_inputs`: List of control inputs
- `tracking_errors`: List of state tracking errors
- `solve_times`: List of optimization solve times
- `constraints_violated`: List of boolean constraint violation flags
- `timestamps`: Time steps

**Key Methods:**
- `add_step()`: Add metrics for one control step
- `position_rmse`: RMSE for position tracking
- `velocity_rmse`: RMSE for velocity tracking
- `attitude_rmse`: RMSE for attitude tracking
- `mean_solve_time`: Average optimization solve time
- `constraint_violation_rate`: Fraction of steps with violations
- `control_effort`: Total control effort

**Example Usage:**
```python
mpc_metrics = MPCMetrics()
mpc_metrics.add_step(
    reference=np.zeros(12),
    actual=np.random.randn(12)*0.05,
    control=np.array([0.5, 0, 0, 0]),
    tracking_error=np.random.randn(12)*0.05,
    solve_time=0.01,
    constraint_violated=False,
    timestamp=0.0
)
print(f"Position RMSE: {mpc_metrics.position_rmse}")
print(f"Mean Solve Time: {mpc_metrics.mean_solve_time*1000:.2f} ms")
```

## Visualization Reference

### PerformanceVisualizer

Creates publication-quality plots.

#### plot_gp_metrics()

Generates comprehensive GP performance plots:

```
Layout:
┌─────────────────────────────────────┐
│ Training Data  │ Prediction Errors  │
├─────────────┬───────────────────────┤
│ Mean Error  │  Uncertainties        │
├─────────────┼───────────────────────┤
│ Error Dist. │ Performance Summary   │
└─────────────┴───────────────────────┘
```

**Usage:**
```python
visualizer = PerformanceVisualizer(figsize=(14, 10))
visualizer.plot_gp_metrics(gp_metrics, save_path='gp_results.png')
```

#### plot_mpc_metrics()

Generates comprehensive MPC performance plots:

```
Layout:
┌──────────────┬───────────────┬──────────────┐
│ Position     │ Velocity      │ Attitude     │
├──────────────┼───────────────┼──────────────┤
│ Thrust       │ Torques       │ Solve Time   │
├──────────────┴───────────────┼──────────────┤
│    3D Trajectory             │ Performance  │
│                              │ Summary      │
└──────────────────────────────┴──────────────┘
```

**Usage:**
```python
visualizer = PerformanceVisualizer(figsize=(16, 12))
visualizer.plot_mpc_metrics(mpc_metrics, save_path='mpc_results.png')
```

#### plot_comparison()

Generates combined GP-MPC analysis:

```
Layout:
┌──────────────┬──────────────┐
│ GP Learning  │ MPC Tracking │
├──────────────┼──────────────┤
│ Uncertainty  │ Performance  │
│ vs Error     │ Summary      │
└──────────────┴──────────────┘
```

**Usage:**
```python
visualizer = PerformanceVisualizer(figsize=(14, 8))
visualizer.plot_comparison(gp_metrics, mpc_metrics, save_path='comparison.png')
```

## Metrics Logger Reference

### MetricsLogger

Saves metrics to JSON format for archival and analysis.

**Usage:**
```python
logger = MetricsLogger(output_dir='/tmp/quadrotor_results')

# Save GP metrics
logger.save_gp_metrics(gp_metrics, filename='run_001_gp.json')

# Save MPC metrics
logger.save_mpc_metrics(mpc_metrics, filename='run_001_mpc.json')
```

**Output Format (GP Metrics):**
```json
{
  "timestamp": "2025-10-19T12:34:56.789012",
  "training_data_count": [10, 20, 30, ...],
  "prediction_errors_mean": [0.123, 0.115, ...],
  "uncertainties_mean": [0.456, 0.445, ...],
  "rmse": 0.0892,
  "summary": {
    "final_training_points": 1000,
    "mean_uncertainty": 0.123
  }
}
```

**Output Format (MPC Metrics):**
```json
{
  "timestamp": "2025-10-19T12:34:56.789012",
  "position_rmse": 0.0523,
  "velocity_rmse": 0.0234,
  "attitude_rmse": 0.0156,
  "mean_solve_time": 0.00987,
  "max_solve_time": 0.01456,
  "constraint_violation_rate": 0.005,
  "control_effort": 0.234,
  "summary": {
    "total_steps": 1000,
    "total_duration": 100.0
  }
}
```

## Performance Metrics Explained

### Gaussian Process Metrics

- **Training Data Count**: Number of points used to train the GP
- **RMSE**: Root Mean Squared Error of GP predictions
- **Mean Uncertainty**: Average prediction variance across dimensions
- **Length Scale**: Hyperparameter controlling correlation length
- **Signal Variance**: Hyperparameter controlling output scale

### Model Predictive Control Metrics

- **Position RMSE**: Tracking error for x, y, z coordinates (meters)
- **Velocity RMSE**: Tracking error for vx, vy, vz velocities (m/s)
- **Attitude RMSE**: Tracking error for φ, θ, ψ angles (radians)
- **Mean/Max Solve Time**: Time to solve optimization problem (milliseconds)
- **Constraint Violation Rate**: Fraction of steps violating constraints (0-1)
- **Control Effort**: Total energy used in control inputs

## Integration with Your System

### Option 1: Real-time Monitoring

Add to your main loop:

```python
# Create metrics at startup
gp_metrics = GPMetrics()
mpc_metrics = MPCMetrics()

# In control loop:
while simulation_running:
    # ... your control code ...
    
    # Collect metrics
    gp_metrics.add_metrics(...)
    mpc_metrics.add_step(...)
    
    # Periodic visualization (every 100 steps)
    if step % 100 == 0:
        visualizer = PerformanceVisualizer()
        visualizer.plot_mpc_metrics(mpc_metrics)
        plt.pause(0.001)  # Update plots
```

### Option 2: Post-processing Analysis

After simulation:

```python
# Load saved metrics
with open('gp_metrics.json') as f:
    gp_data = json.load(f)

# Recreate metrics objects and visualize
visualizer = PerformanceVisualizer()
visualizer.plot_gp_metrics(gp_metrics, save_path='final_gp.png')
visualizer.plot_mpc_metrics(mpc_metrics, save_path='final_mpc.png')
visualizer.plot_comparison(gp_metrics, mpc_metrics, save_path='final_comparison.png')
```

## Troubleshooting

### Issue: Plots not showing

**Solution**: Use `plt.show()` in scripts or save to file:
```python
visualizer.plot_gp_metrics(gp_metrics, save_path='output.png')
```

### Issue: Memory usage high

**Solution**: Downsample data or use periodic clearing:
```python
# Keep only last 1000 points
if len(gp_metrics.timestamps) > 1000:
    gp_metrics = GPMetrics()  # Reset
```

### Issue: Correlation analysis showing NaN

**Solution**: Ensure both GP and MPC metrics have sufficient data points

## Examples

See `example_metrics.py` for complete working examples:

```bash
python3 -m quadrotor_gp_mpc.example_metrics
```

## References

- GP metrics based on Gaussian Process regression theory
- MPC metrics follow standard control performance evaluation methods
- Visualization uses matplotlib for publication-quality plots
