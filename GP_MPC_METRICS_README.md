# GP and MPC Performance Metrics - Integration Guide

## What You've Received

I've created a comprehensive performance metrics and visualization system for your Gaussian Process (GP) and Model Predictive Control (MPC) system. This includes:

### Files Created

1. **`performance_metrics.py`** - Main metrics module with:
   - `GPMetrics` class: Tracks GP learning performance
   - `MPCMetrics` class: Tracks MPC control performance
   - `PerformanceVisualizer` class: Generates professional plots
   - `MetricsLogger` class: Saves metrics to JSON

2. **`example_metrics.py`** - Complete working example demonstrating:
   - How to collect GP metrics
   - How to collect MPC metrics
   - How to generate plots
   - How to perform combined analysis

3. **`PERFORMANCE_METRICS_GUIDE.md`** - Comprehensive documentation

## Quick Start

### 1. Run the Example

```bash
cd ~/quadrotor_gp_mpc_ws
source install/setup.bash
python3 quadrotor_gp_mpc/quadrotor_gp_mpc/example_metrics.py
```

This generates example visualizations:
- `/tmp/gp_metrics.png` - 3x3 grid showing GP performance
- `/tmp/mpc_metrics.png` - 3x3 grid showing MPC performance
- `/tmp/gp_mpc_comparison.png` - Combined analysis plots
- `/tmp/example_gp_metrics.json` - GP metrics data
- `/tmp/example_mpc_metrics.json` - MPC metrics data

### 2. View Generated Plots

The plots have been created successfully! You can view them using:

```bash
# View individual plots
feh /tmp/gp_metrics.png
feh /tmp/mpc_metrics.png
feh /tmp/gp_mpc_comparison.png

# Or open in your preferred image viewer
```

## Integration into Your System

### Option 1: Add to Your Main Loop

```python
from quadrotor_gp_mpc.performance_metrics import (
    GPMetrics, MPCMetrics, PerformanceVisualizer
)

# At startup
gp_metrics = GPMetrics()
mpc_metrics = MPCMetrics()
visualizer = PerformanceVisualizer()

# In your control loop
while running:
    # ... your code ...
    
    # Collect GP metrics (in your GP node)
    gp_metrics.add_metrics(
        n_data=len(gp.X_train),
        pred_err=prediction_error,
        uncertainty=prediction_uncertainty,
        hyperparams={'length_scale': gp.kernel.length_scale,
                     'signal_variance': gp.kernel.signal_variance},
        timestamp=current_time
    )
    
    # Collect MPC metrics (in your MPC node)
    mpc_metrics.add_step(
        reference=reference_state,
        actual=current_state,
        control=control_input,
        tracking_error=current_state - reference_state,
        solve_time=mpc_solve_time,
        constraint_violated=False,
        timestamp=current_time
    )
    
    # Visualize every 100 steps
    if step % 100 == 0:
        visualizer.plot_gp_metrics(gp_metrics, save_path=f'gp_step_{step}.png')
        visualizer.plot_mpc_metrics(mpc_metrics, save_path=f'mpc_step_{step}.png')
```

### Option 2: Post-Simulation Analysis

```python
from quadrotor_gp_mpc.performance_metrics import PerformanceVisualizer, MetricsLogger

# Load your saved metrics (or create from simulation)
visualizer = PerformanceVisualizer(figsize=(16, 12))
logger = MetricsLogger(output_dir='./results')

# Generate plots
visualizer.plot_gp_metrics(gp_metrics, save_path='results/gp_final.png')
visualizer.plot_mpc_metrics(mpc_metrics, save_path='results/mpc_final.png')
visualizer.plot_comparison(gp_metrics, mpc_metrics, save_path='results/comparison.png')

# Save data for archival
logger.save_gp_metrics(gp_metrics, 'run_001_gp.json')
logger.save_mpc_metrics(mpc_metrics, 'run_001_mpc.json')
```

## Key Metrics Explained

### Gaussian Process

| Metric | Description | Unit |
|--------|-------------|------|
| **Training Data Count** | Number of points collected over time | count |
| **RMSE** | Root Mean Squared Error of predictions | - |
| **Mean Uncertainty** | Average prediction variance | - |
| **Length Scale** | RBF kernel correlation length (tuned) | - |
| **Signal Variance** | Output scale (tuned) | - |

### Model Predictive Control

| Metric | Description | Unit |
|--------|-------------|------|
| **Position RMSE** | Error in x, y, z tracking | meters |
| **Velocity RMSE** | Error in vx, vy, vz tracking | m/s |
| **Attitude RMSE** | Error in φ, θ, ψ tracking | radians |
| **Mean Solve Time** | Average optimization time | milliseconds |
| **Max Solve Time** | Worst-case optimization time | milliseconds |
| **Constraint Violation Rate** | % of steps violating constraints | % |
| **Control Effort** | Total control energy used | - |

## Generated Plots

### GP Metrics Plot (3×3 Layout)

```
┌─────────────────────────────────────────┐
│ Training Data   Prediction Errors       │
│ (growth curve)  (error over time)       │
├─────────────────────────────────────────┤
│ Mean Error      Uncertainties           │
│ (aggregate)     (variance over time)    │
├─────────────────────────────────────────┤
│ Error Distribution   Performance Summary │
│ (histogram)          (statistics)       │
└─────────────────────────────────────────┘
```

### MPC Metrics Plot (3×3 Layout)

```
┌────────────────────────────────────────┐
│ Position Error  Velocity Error  Attitude│
│ (x,y,z)         (vx,vy,vz)      (φ,θ,ψ)│
├────────────────────────────────────────┤
│ Thrust Input    Torques         Solve Time│
│ (T)             (τφ,τθ,τψ)      (ms)   │
├────────────────────────────────────────┤
│   XY Trajectory (top view)  Performance │
│                             Summary     │
└────────────────────────────────────────┘
```

### Comparison Plot (2×2 Layout)

```
┌──────────────────────────────┐
│ GP Learning    MPC Tracking  │
│ (data growth)  (error decay) │
├──────────────────────────────┤
│ Uncertainty vs Error Correlation
│ + Performance Summary        │
└──────────────────────────────┘
```

## Data Export

### JSON Format

Metrics are saved in human-readable JSON:

**GP Metrics:**
```json
{
  "timestamp": "2025-10-19T12:34:56",
  "training_data_count": [10, 20, ..., 500],
  "prediction_errors_mean": [0.5, 0.4, ..., 0.05],
  "uncertainties_mean": [1.0, 0.9, ..., 0.1],
  "rmse": 0.0523,
  "summary": {
    "final_training_points": 500,
    "mean_uncertainty": 0.234
  }
}
```

**MPC Metrics:**
```json
{
  "timestamp": "2025-10-19T12:34:56",
  "position_rmse": 0.0523,
  "velocity_rmse": 0.0234,
  "attitude_rmse": 0.0156,
  "mean_solve_time": 0.00987,
  "constraint_violation_rate": 0.005,
  "control_effort": 0.234
}
```

## Features

✅ **Real-time Monitoring**: Track metrics during simulation
✅ **Professional Plots**: Publication-ready visualizations
✅ **Data Export**: Save metrics to JSON for analysis
✅ **Comprehensive Analysis**: Multiple perspectives on performance
✅ **Easy Integration**: Simple API for adding to existing code
✅ **Correlation Analysis**: Understand GP uncertainty impact on MPC
✅ **Performance Summaries**: Quick reference statistics

## Dependencies

Required packages (already in your setup):
- `numpy` - numerical computing
- `matplotlib` - plotting
- `scipy` - scientific computing (optional, for advanced analysis)

## File Sizes

Generated plots are high-quality (300 DPI):
- GP metrics plot: ~870 KB
- MPC metrics plot: ~1.3 MB  
- Comparison plot: ~542 KB

## Next Steps

1. **Run the example** to see it in action
2. **Read `PERFORMANCE_METRICS_GUIDE.md`** for detailed API documentation
3. **Integrate into your system** using the provided patterns
4. **Customize plots** by modifying `PerformanceVisualizer` class
5. **Export and share** results using the JSON logger

## Troubleshooting

**Issue**: Plots not displaying
- **Solution**: Use `save_path` parameter to save to file

**Issue**: Memory usage high with long simulations
- **Solution**: Periodically clear metrics or downsample data

**Issue**: Missing 3D plots
- **Solution**: The current version uses 2D top-view trajectory (matplotlib compatibility)

## Files Structure

```
quadrotor_gp_mpc_ws/
├── quadrotor_gp_mpc/
│   └── quadrotor_gp_mpc/
│       ├── performance_metrics.py      ← Main metrics module
│       └── example_metrics.py           ← Usage example
├── PERFORMANCE_METRICS_GUIDE.md         ← Detailed guide
└── GP_MPC_METRICS_README.md             ← This file
```

## Support

For questions about the metrics system, refer to:
1. `PERFORMANCE_METRICS_GUIDE.md` - Comprehensive API docs
2. `example_metrics.py` - Working examples
3. Docstrings in `performance_metrics.py` - Inline documentation

---

**Generated by GitHub Copilot** | October 19, 2025
