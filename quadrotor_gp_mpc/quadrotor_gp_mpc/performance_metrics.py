#!/usr/bin/env python3
"""
Performance Metrics and Visualization for GP and MPC

This module provides comprehensive performance metrics and plotting
capabilities for the Gaussian Process and Model Predictive Control.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass, field
from datetime import datetime
import json


@dataclass
class GPMetrics:
    """Metrics for Gaussian Process performance."""
    
    training_data_count: List[int] = field(default_factory=list)
    prediction_errors: List[np.ndarray] = field(default_factory=list)
    uncertainties: List[np.ndarray] = field(default_factory=list)
    hyperparameters: List[Dict] = field(default_factory=list)
    timestamps: List[float] = field(default_factory=list)
    
    def add_metrics(self, n_data: int, pred_err: np.ndarray, uncertainty: np.ndarray,
                   hyperparams: Dict, timestamp: float):
        """Add metrics at a time step."""
        self.training_data_count.append(n_data)
        self.prediction_errors.append(pred_err)
        self.uncertainties.append(uncertainty)
        self.hyperparameters.append(hyperparams)
        self.timestamps.append(timestamp)
    
    @property
    def mean_prediction_error(self) -> np.ndarray:
        """Compute mean prediction error across time."""
        if not self.prediction_errors:
            return np.array([])
        return np.mean(self.prediction_errors, axis=0)
    
    @property
    def mean_uncertainty(self) -> np.ndarray:
        """Compute mean uncertainty across time."""
        if not self.uncertainties:
            return np.array([])
        return np.mean(self.uncertainties, axis=0)
    
    def rmse(self) -> float:
        """Root Mean Squared Error of GP predictions."""
        if not self.prediction_errors:
            return 0.0
        return np.sqrt(np.mean(np.concatenate(self.prediction_errors)**2))


@dataclass
class MPCMetrics:
    """Metrics for Model Predictive Control performance."""
    
    reference_trajectory: List[np.ndarray] = field(default_factory=list)
    actual_trajectory: List[np.ndarray] = field(default_factory=list)
    control_inputs: List[np.ndarray] = field(default_factory=list)
    tracking_errors: List[np.ndarray] = field(default_factory=list)
    solve_times: List[float] = field(default_factory=list)
    constraints_violated: List[bool] = field(default_factory=list)
    timestamps: List[float] = field(default_factory=list)
    
    def add_step(self, reference: np.ndarray, actual: np.ndarray, control: np.ndarray,
                tracking_error: np.ndarray, solve_time: float, 
                constraint_violated: bool, timestamp: float):
        """Add metrics for one MPC step."""
        self.reference_trajectory.append(reference)
        self.actual_trajectory.append(actual)
        self.control_inputs.append(control)
        self.tracking_errors.append(tracking_error)
        self.solve_times.append(solve_time)
        self.constraints_violated.append(constraint_violated)
        self.timestamps.append(timestamp)
    
    @property
    def position_rmse(self) -> float:
        """RMSE for position tracking (x, y, z)."""
        if not self.tracking_errors:
            return 0.0
        errors = np.array([e[:3] for e in self.tracking_errors])
        return np.sqrt(np.mean(errors**2))
    
    @property
    def velocity_rmse(self) -> float:
        """RMSE for velocity tracking (vx, vy, vz)."""
        if not self.tracking_errors:
            return 0.0
        errors = np.array([e[3:6] for e in self.tracking_errors])
        return np.sqrt(np.mean(errors**2))
    
    @property
    def attitude_rmse(self) -> float:
        """RMSE for attitude tracking (φ, θ, ψ)."""
        if not self.tracking_errors:
            return 0.0
        errors = np.array([e[6:9] for e in self.tracking_errors])
        return np.sqrt(np.mean(errors**2))
    
    @property
    def mean_solve_time(self) -> float:
        """Mean time to solve MPC optimization."""
        if not self.solve_times:
            return 0.0
        return np.mean(self.solve_times)
    
    @property
    def max_solve_time(self) -> float:
        """Maximum time to solve MPC optimization."""
        if not self.solve_times:
            return 0.0
        return np.max(self.solve_times)
    
    @property
    def constraint_violation_rate(self) -> float:
        """Fraction of steps where constraints were violated."""
        if not self.constraints_violated:
            return 0.0
        return sum(self.constraints_violated) / len(self.constraints_violated)
    
    @property
    def control_effort(self) -> float:
        """Total control effort (sum of squared control inputs)."""
        if not self.control_inputs:
            return 0.0
        efforts = [np.sum(u**2) for u in self.control_inputs]
        return np.mean(efforts)


class PerformanceVisualizer:
    """Visualization tools for GP and MPC performance."""
    
    def __init__(self, figsize: Tuple[int, int] = (16, 12)):
        self.figsize = figsize
        plt.style.use('seaborn-v0_8-darkgrid')
    
    def plot_gp_metrics(self, gp_metrics: GPMetrics, save_path: Optional[str] = None):
        """
        Plot comprehensive GP metrics.
        
        Args:
            gp_metrics: GPMetrics object
            save_path: Optional path to save figure
        """
        fig = plt.figure(figsize=self.figsize)
        gs = gridspec.GridSpec(3, 3, figure=fig, hspace=0.3, wspace=0.3)
        
        # 1. Training data count over time
        ax1 = fig.add_subplot(gs[0, 0])
        ax1.plot(gp_metrics.timestamps, gp_metrics.training_data_count, 'b-', linewidth=2)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Number of Training Points')
        ax1.set_title('GP Training Data Collection')
        ax1.grid(True, alpha=0.3)
        
        # 2. Prediction error per dimension
        ax2 = fig.add_subplot(gs[0, 1:])
        if gp_metrics.prediction_errors:
            errors_array = np.array(gp_metrics.prediction_errors)
            for i in range(min(3, errors_array.shape[1])):  # Plot first 3 dimensions
                ax2.plot(gp_metrics.timestamps, errors_array[:, i], label=f'Dim {i}', alpha=0.7)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Prediction Error')
        ax2.set_title('GP Prediction Errors')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 3. Mean prediction error across dimensions
        ax3 = fig.add_subplot(gs[1, 0])
        if gp_metrics.prediction_errors:
            mean_errors = np.mean(np.array(gp_metrics.prediction_errors), axis=1)
            ax3.plot(gp_metrics.timestamps, mean_errors, 'g-', linewidth=2)
            ax3.fill_between(gp_metrics.timestamps, 0, mean_errors, alpha=0.3)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Mean Error')
        ax3.set_title('Mean Prediction Error')
        ax3.grid(True, alpha=0.3)
        
        # 4. Uncertainties over time
        ax4 = fig.add_subplot(gs[1, 1:])
        if gp_metrics.uncertainties:
            uncertainties_array = np.array(gp_metrics.uncertainties)
            for i in range(min(3, uncertainties_array.shape[1])):
                ax4.plot(gp_metrics.timestamps, uncertainties_array[:, i], 
                        label=f'Dim {i}', alpha=0.7)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Uncertainty Variance')
        ax4.set_title('GP Prediction Uncertainties')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        # 5. Error distribution histogram
        ax5 = fig.add_subplot(gs[2, 0])
        if gp_metrics.prediction_errors:
            all_errors = np.concatenate(gp_metrics.prediction_errors)
            ax5.hist(all_errors, bins=30, edgecolor='black', alpha=0.7)
        ax5.set_xlabel('Prediction Error')
        ax5.set_ylabel('Frequency')
        ax5.set_title('Error Distribution')
        ax5.grid(True, alpha=0.3)
        
        # 6. Performance summary text
        ax6 = fig.add_subplot(gs[2, 1:])
        ax6.axis('off')
        
        summary_text = f"""
        GP Performance Summary
        {'='*40}
        Total Training Points: {gp_metrics.training_data_count[-1] if gp_metrics.training_data_count else 0}
        RMSE: {gp_metrics.rmse():.4f}
        Mean Uncertainty: {np.mean(gp_metrics.mean_uncertainty):.4f}
        Final Length Scale: {gp_metrics.hyperparameters[-1].get('length_scale', 0):.4f}
        Final Signal Variance: {gp_metrics.hyperparameters[-1].get('signal_variance', 0):.4f}
        """
        
        ax6.text(0.1, 0.5, summary_text, fontsize=11, family='monospace',
                verticalalignment='center', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        fig.suptitle('Gaussian Process Performance Metrics', fontsize=16, fontweight='bold')
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"GP metrics saved to {save_path}")
        
        return fig
    
    def plot_mpc_metrics(self, mpc_metrics: MPCMetrics, save_path: Optional[str] = None):
        """
        Plot comprehensive MPC metrics.
        
        Args:
            mpc_metrics: MPCMetrics object
            save_path: Optional path to save figure
        """
        fig = plt.figure(figsize=self.figsize)
        gs = gridspec.GridSpec(3, 3, figure=fig, hspace=0.3, wspace=0.3)
        
        # 1. Position tracking error
        ax1 = fig.add_subplot(gs[0, 0])
        if mpc_metrics.tracking_errors:
            errors_array = np.array(mpc_metrics.tracking_errors)
            pos_errors = errors_array[:, :3]
            for i in range(3):
                ax1.plot(mpc_metrics.timestamps, pos_errors[:, i], 
                        label=['x', 'y', 'z'][i], alpha=0.7)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Position Error (m)')
        ax1.set_title('Position Tracking Error')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # 2. Velocity tracking error
        ax2 = fig.add_subplot(gs[0, 1])
        if mpc_metrics.tracking_errors:
            errors_array = np.array(mpc_metrics.tracking_errors)
            vel_errors = errors_array[:, 3:6]
            for i in range(3):
                ax2.plot(mpc_metrics.timestamps, vel_errors[:, i], 
                        label=['vx', 'vy', 'vz'][i], alpha=0.7)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Velocity Error (m/s)')
        ax2.set_title('Velocity Tracking Error')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 3. Attitude tracking error
        ax3 = fig.add_subplot(gs[0, 2])
        if mpc_metrics.tracking_errors:
            errors_array = np.array(mpc_metrics.tracking_errors)
            att_errors = errors_array[:, 6:9]
            for i in range(3):
                ax3.plot(mpc_metrics.timestamps, att_errors[:, i], 
                        label=['φ', 'θ', 'ψ'][i], alpha=0.7)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Attitude Error (rad)')
        ax3.set_title('Attitude Tracking Error')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # 4. Control inputs (thrust and torques)
        ax4 = fig.add_subplot(gs[1, 0])
        if mpc_metrics.control_inputs:
            controls_array = np.array(mpc_metrics.control_inputs)
            ax4.plot(mpc_metrics.timestamps, controls_array[:, 0], 'b-', label='Thrust', linewidth=2)
            ax4.set_xlabel('Time (s)')
            ax4.set_ylabel('Thrust (N)')
            ax4.set_title('Thrust Control Input')
            ax4.grid(True, alpha=0.3)
        
        # 5. Torque control inputs
        ax5 = fig.add_subplot(gs[1, 1])
        if mpc_metrics.control_inputs:
            controls_array = np.array(mpc_metrics.control_inputs)
            for i in range(1, 4):
                ax5.plot(mpc_metrics.timestamps, controls_array[:, i], 
                        label=['τφ', 'τθ', 'τψ'][i-1], alpha=0.7)
            ax5.set_xlabel('Time (s)')
            ax5.set_ylabel('Torque (N·m)')
            ax5.set_title('Torque Control Inputs')
            ax5.legend()
            ax5.grid(True, alpha=0.3)
        
        # 6. MPC solver time
        ax6 = fig.add_subplot(gs[1, 2])
        ax6.plot(mpc_metrics.timestamps, mpc_metrics.solve_times, 'r-', linewidth=2)
        ax6.axhline(y=mpc_metrics.mean_solve_time, color='g', linestyle='--', 
                   label=f'Mean: {mpc_metrics.mean_solve_time*1000:.2f} ms')
        ax6.set_xlabel('Time (s)')
        ax6.set_ylabel('Solve Time (s)')
        ax6.set_title('MPC Optimization Time')
        ax6.legend()
        ax6.grid(True, alpha=0.3)
        
        # 7. XY trajectory projection
        ax7 = fig.add_subplot(gs[2, :2])
        if mpc_metrics.reference_trajectory and mpc_metrics.actual_trajectory:
            ref_traj = np.array(mpc_metrics.reference_trajectory)
            act_traj = np.array(mpc_metrics.actual_trajectory)
            ax7.plot(ref_traj[:, 0], ref_traj[:, 1], 'g--', 
                    label='Reference', linewidth=2, alpha=0.7)
            ax7.plot(act_traj[:, 0], act_traj[:, 1], 'b-', 
                    label='Actual', linewidth=2, alpha=0.7)
            # Mark start and end
            ax7.plot(ref_traj[0, 0], ref_traj[0, 1], 'go', markersize=10, label='Start')
            ax7.plot(ref_traj[-1, 0], ref_traj[-1, 1], 'rs', markersize=10, label='End')
        ax7.set_xlabel('X (m)')
        ax7.set_ylabel('Y (m)')
        ax7.set_title('XY Trajectory Tracking (Top View)')
        ax7.legend()
        ax7.grid(True, alpha=0.3)
        ax7.axis('equal')
        
        # 8. Performance summary
        ax8 = fig.add_subplot(gs[2, 2])
        ax8.axis('off')
        
        summary_text = f"""
        MPC Performance Summary
        {'='*40}
        Position RMSE: {mpc_metrics.position_rmse:.4f} m
        Velocity RMSE: {mpc_metrics.velocity_rmse:.4f} m/s
        Attitude RMSE: {mpc_metrics.attitude_rmse:.4f} rad
        Mean Solve Time: {mpc_metrics.mean_solve_time*1000:.2f} ms
        Max Solve Time: {mpc_metrics.max_solve_time*1000:.2f} ms
        Constraint Violation: {mpc_metrics.constraint_violation_rate*100:.2f}%
        Control Effort: {mpc_metrics.control_effort:.4f}
        """
        
        ax8.text(0.1, 0.5, summary_text, fontsize=10, family='monospace',
                verticalalignment='center', bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))
        
        fig.suptitle('Model Predictive Control Performance Metrics', fontsize=16, fontweight='bold')
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"MPC metrics saved to {save_path}")
        
        return fig
    
    def plot_comparison(self, gp_metrics: GPMetrics, mpc_metrics: MPCMetrics, 
                       save_path: Optional[str] = None):
        """
        Plot comparison between GP and MPC performance.
        
        Args:
            gp_metrics: GPMetrics object
            mpc_metrics: MPCMetrics object
            save_path: Optional path to save figure
        """
        fig = plt.figure(figsize=(14, 8))
        gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.3, wspace=0.3)
        
        # 1. GP learning curve
        ax1 = fig.add_subplot(gs[0, 0])
        ax1.plot(gp_metrics.timestamps, gp_metrics.training_data_count, 'b-', linewidth=2)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Training Points')
        ax1.set_title('GP Learning Progress')
        ax1.grid(True, alpha=0.3)
        
        # 2. MPC tracking performance
        ax2 = fig.add_subplot(gs[0, 1])
        if mpc_metrics.tracking_errors:
            errors_array = np.array(mpc_metrics.tracking_errors)
            pos_errors = np.linalg.norm(errors_array[:, :3], axis=1)
            ax2.plot(mpc_metrics.timestamps, pos_errors, 'r-', linewidth=2)
            ax2.fill_between(mpc_metrics.timestamps, 0, pos_errors, alpha=0.3)
            ax2.axhline(y=np.mean(pos_errors), color='g', linestyle='--', label='Mean')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Position Error Magnitude (m)')
        ax2.set_title('MPC Tracking Error')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 3. GP uncertainty vs MPC error correlation
        ax3 = fig.add_subplot(gs[1, 0])
        if gp_metrics.uncertainties and mpc_metrics.tracking_errors:
            mean_uncer = np.mean(np.array(gp_metrics.uncertainties), axis=1)
            pos_errors = np.linalg.norm(np.array(mpc_metrics.tracking_errors)[:, :3], axis=1)
            # Align lengths
            min_len = min(len(mean_uncer), len(pos_errors))
            ax3.scatter(mean_uncer[:min_len], pos_errors[:min_len], alpha=0.6)
            ax3.set_xlabel('Mean GP Uncertainty')
            ax3.set_ylabel('MPC Position Error (m)')
            ax3.set_title('GP Uncertainty vs Tracking Error')
            ax3.grid(True, alpha=0.3)
        
        # 4. Combined metrics summary
        ax4 = fig.add_subplot(gs[1, 1])
        ax4.axis('off')
        
        metrics_text = f"""
        System Performance Summary
        {'='*45}
        
        Gaussian Process:
        - Training Points: {gp_metrics.training_data_count[-1] if gp_metrics.training_data_count else 0}
        - RMSE: {gp_metrics.rmse():.4f}
        - Mean Uncertainty: {np.mean(gp_metrics.mean_uncertainty):.4f}
        
        Model Predictive Control:
        - Position RMSE: {mpc_metrics.position_rmse:.4f} m
        - Velocity RMSE: {mpc_metrics.velocity_rmse:.4f} m/s
        - Mean Solve Time: {mpc_metrics.mean_solve_time*1000:.2f} ms
        
        Overall:
        - Total Duration: {mpc_metrics.timestamps[-1] if mpc_metrics.timestamps else 0:.2f} s
        """
        
        ax4.text(0.05, 0.5, metrics_text, fontsize=10, family='monospace',
                verticalalignment='center', bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.5))
        
        fig.suptitle('GP-MPC System Performance Comparison', fontsize=16, fontweight='bold')
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Comparison plot saved to {save_path}")
        
        return fig


class MetricsLogger:
    """Log metrics to JSON format for later analysis."""
    
    def __init__(self, output_dir: str = '/tmp/quadrotor_metrics'):
        self.output_dir = output_dir
        import os
        os.makedirs(output_dir, exist_ok=True)
    
    def save_gp_metrics(self, gp_metrics: GPMetrics, filename: Optional[str] = None):
        """Save GP metrics to JSON file."""
        if filename is None:
            filename = f"gp_metrics_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        
        filepath = f"{self.output_dir}/{filename}"
        
        data = {
            'timestamp': datetime.now().isoformat(),
            'training_data_count': gp_metrics.training_data_count,
            'prediction_errors_mean': [float(np.mean(e)) for e in gp_metrics.prediction_errors],
            'uncertainties_mean': [float(np.mean(u)) for u in gp_metrics.uncertainties],
            'rmse': float(gp_metrics.rmse()),
            'summary': {
                'final_training_points': gp_metrics.training_data_count[-1] if gp_metrics.training_data_count else 0,
                'mean_uncertainty': float(np.mean(gp_metrics.mean_uncertainty)) if len(gp_metrics.mean_uncertainty) > 0 else 0.0,
            }
        }
        
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"GP metrics saved to {filepath}")
    
    def save_mpc_metrics(self, mpc_metrics: MPCMetrics, filename: Optional[str] = None):
        """Save MPC metrics to JSON file."""
        if filename is None:
            filename = f"mpc_metrics_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        
        filepath = f"{self.output_dir}/{filename}"
        
        data = {
            'timestamp': datetime.now().isoformat(),
            'position_rmse': float(mpc_metrics.position_rmse),
            'velocity_rmse': float(mpc_metrics.velocity_rmse),
            'attitude_rmse': float(mpc_metrics.attitude_rmse),
            'mean_solve_time': float(mpc_metrics.mean_solve_time),
            'max_solve_time': float(mpc_metrics.max_solve_time),
            'constraint_violation_rate': float(mpc_metrics.constraint_violation_rate),
            'control_effort': float(mpc_metrics.control_effort),
            'summary': {
                'total_steps': len(mpc_metrics.tracking_errors),
                'total_duration': float(mpc_metrics.timestamps[-1]) if mpc_metrics.timestamps else 0.0,
            }
        }
        
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"MPC metrics saved to {filepath}")


if __name__ == "__main__":
    # Example usage
    print("Performance metrics and visualization module loaded successfully.")
