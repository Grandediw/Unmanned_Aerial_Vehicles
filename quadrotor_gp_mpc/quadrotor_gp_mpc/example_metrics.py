#!/usr/bin/env python3
"""
Example script demonstrating GP and MPC performance metrics and visualization.

This script shows how to:
1. Collect metrics during simulation
2. Generate performance plots
3. Save metrics to files
4. Compare GP and MPC performance
"""

import numpy as np
import matplotlib.pyplot as plt
from quadrotor_gp_mpc.performance_metrics import (
    GPMetrics, MPCMetrics, PerformanceVisualizer, MetricsLogger
)


def example_gp_metrics():
    """Create example GP metrics and plot them."""
    print("="*60)
    print("Example: Gaussian Process Metrics")
    print("="*60)
    
    # Create GP metrics
    gp_metrics = GPMetrics()
    
    # Simulate data collection over time
    t_max = 100  # seconds
    dt = 0.5     # sampling interval
    time_steps = np.arange(0, t_max, dt)
    
    for t in time_steps:
        # Simulate data collection
        n_data = int(5 * t + 10)  # Linear growth
        pred_err = np.random.randn(12) * (1.0 / (1 + t/20))  # Decreasing error
        uncertainty = np.random.randn(12) * (2.0 / (1 + t/15))  # Decreasing uncertainty
        hyperparams = {
            'length_scale': 1.0 + 0.01 * t,
            'signal_variance': 1.0 - 0.005 * t
        }
        
        gp_metrics.add_metrics(n_data, pred_err, uncertainty, hyperparams, t)
    
    # Print metrics summary
    print(f"\nGP Metrics Summary:")
    print(f"  Total Training Points: {gp_metrics.training_data_count[-1]}")
    print(f"  RMSE: {gp_metrics.rmse():.4f}")
    print(f"  Mean Uncertainty: {np.mean(gp_metrics.mean_uncertainty):.4f}")
    print(f"  Final Length Scale: {gp_metrics.hyperparameters[-1]['length_scale']:.4f}")
    
    # Visualize
    visualizer = PerformanceVisualizer(figsize=(14, 10))
    visualizer.plot_gp_metrics(gp_metrics, save_path='/tmp/gp_metrics.png')
    print("\n✓ GP metrics plot saved to /tmp/gp_metrics.png")
    
    # Save to JSON
    logger = MetricsLogger('/tmp')
    logger.save_gp_metrics(gp_metrics, 'example_gp_metrics.json')
    print("✓ GP metrics saved to /tmp/example_gp_metrics.json")
    
    return gp_metrics


def example_mpc_metrics():
    """Create example MPC metrics and plot them."""
    print("\n" + "="*60)
    print("Example: Model Predictive Control Metrics")
    print("="*60)
    
    # Create MPC metrics
    mpc_metrics = MPCMetrics()
    
    # Simulate trajectory tracking
    t_max = 100  # seconds
    dt = 0.1     # sampling interval (10 Hz)
    time_steps = np.arange(0, t_max, dt)
    
    # Circular reference trajectory
    for i, t in enumerate(time_steps):
        radius = 2.0
        height = 1.5
        period = 20.0
        
        # Reference trajectory (circular motion)
        angle = 2 * np.pi * t / period
        ref_state = np.array([
            radius * np.cos(angle),                    # x
            radius * np.sin(angle),                    # y
            height,                                     # z
            -radius * 2 * np.pi / period * np.sin(angle),  # vx
            radius * 2 * np.pi / period * np.cos(angle),   # vy
            0.0,                                        # vz
            0.0, 0.0, angle,                           # φ, θ, ψ
            0.0, 0.0, 2 * np.pi / period               # p, q, r
        ])
        
        # Actual trajectory with tracking error
        error_decay = np.exp(-t / 30)  # Exponential decay of error
        actual_state = ref_state + error_decay * np.random.randn(12) * 0.1
        
        # Control inputs
        thrust = 0.5 + 0.1 * np.sin(t)
        torques = 0.01 * np.sin(t + np.array([0, np.pi/3, 2*np.pi/3]))
        control = np.concatenate([[thrust], torques])
        
        # Tracking error
        tracking_error = actual_state - ref_state
        
        # MPC solve time (realistic simulation)
        solve_time = 0.01 + 0.005 * np.random.randn()
        
        # Constraint violation (rare)
        constraint_violated = np.random.rand() < 0.01
        
        mpc_metrics.add_step(
            ref_state, actual_state, control, tracking_error,
            solve_time, constraint_violated, t
        )
    
    # Print metrics summary
    print(f"\nMPC Metrics Summary:")
    print(f"  Position RMSE: {mpc_metrics.position_rmse:.4f} m")
    print(f"  Velocity RMSE: {mpc_metrics.velocity_rmse:.4f} m/s")
    print(f"  Attitude RMSE: {mpc_metrics.attitude_rmse:.4f} rad")
    print(f"  Mean Solve Time: {mpc_metrics.mean_solve_time*1000:.2f} ms")
    print(f"  Max Solve Time: {mpc_metrics.max_solve_time*1000:.2f} ms")
    print(f"  Constraint Violation Rate: {mpc_metrics.constraint_violation_rate*100:.2f}%")
    print(f"  Control Effort: {mpc_metrics.control_effort:.4f}")
    
    # Visualize
    visualizer = PerformanceVisualizer(figsize=(16, 12))
    visualizer.plot_mpc_metrics(mpc_metrics, save_path='/tmp/mpc_metrics.png')
    print("\n✓ MPC metrics plot saved to /tmp/mpc_metrics.png")
    
    # Save to JSON
    logger = MetricsLogger('/tmp')
    logger.save_mpc_metrics(mpc_metrics, 'example_mpc_metrics.json')
    print("✓ MPC metrics saved to /tmp/example_mpc_metrics.json")
    
    return mpc_metrics


def example_combined_analysis(gp_metrics: GPMetrics, mpc_metrics: MPCMetrics):
    """Create combined GP-MPC analysis."""
    print("\n" + "="*60)
    print("Example: Combined GP-MPC Analysis")
    print("="*60)
    
    # Create comparison plot
    visualizer = PerformanceVisualizer(figsize=(14, 8))
    visualizer.plot_comparison(gp_metrics, mpc_metrics, save_path='/tmp/gp_mpc_comparison.png')
    print("✓ Comparison plot saved to /tmp/gp_mpc_comparison.png")
    
    # Analysis
    print(f"\nSystem Performance Analysis:")
    print(f"  GP learned from {gp_metrics.training_data_count[-1]} data points")
    print(f"  MPC achieved {mpc_metrics.position_rmse:.4f} m position RMSE")
    print(f"  System ran for {mpc_metrics.timestamps[-1]:.1f} seconds")
    
    # Correlation analysis
    if gp_metrics.uncertainties and mpc_metrics.tracking_errors:
        mean_gp_uncertainty = np.mean(np.array(gp_metrics.uncertainties), axis=1)
        pos_errors = np.linalg.norm(np.array(mpc_metrics.tracking_errors)[:, :3], axis=1)
        
        # Align lengths
        min_len = min(len(mean_gp_uncertainty), len(pos_errors))
        if min_len > 1:
            correlation = np.corrcoef(mean_gp_uncertainty[:min_len], pos_errors[:min_len])[0, 1]
            print(f"  GP Uncertainty-MPC Error Correlation: {correlation:.4f}")


def main():
    """Run all examples."""
    print("\n" + "="*60)
    print("GP and MPC Performance Metrics - Example Demonstration")
    print("="*60)
    
    # Run examples
    gp_metrics = example_gp_metrics()
    mpc_metrics = example_mpc_metrics()
    example_combined_analysis(gp_metrics, mpc_metrics)
    
    print("\n" + "="*60)
    print("Examples completed successfully!")
    print("Output files:")
    print("  - /tmp/gp_metrics.png")
    print("  - /tmp/mpc_metrics.png")
    print("  - /tmp/gp_mpc_comparison.png")
    print("  - /tmp/example_gp_metrics.json")
    print("  - /tmp/example_mpc_metrics.json")
    print("="*60 + "\n")
    
    # Display plots
    plt.show()


if __name__ == "__main__":
    main()
