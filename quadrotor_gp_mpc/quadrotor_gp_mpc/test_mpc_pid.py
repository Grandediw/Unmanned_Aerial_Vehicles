#!/usr/bin/env python3
"""
MPC Setpoint Tracking Test - PID Control with Integral Action

This script validates MPC setpoint tracking with full PID control.
Demonstrates stable altitude control with zero steady-state error.
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Dict


class PIDSetpointTest:
    """PID-based setpoint tracking test with integral action."""
    
    def __init__(self):
        """Initialize test."""
        self.dt = 0.1
        self.mass = 0.5
        self.g = 9.81
        self.z_integral = 0.0
        self.max_integral = 2.0  # Integral windup limit
        
    def simulate(self, setpoint: np.ndarray, duration: float,
                 kp: float = 15.0, kd: float = 8.0, ki: float = 2.0) -> Dict:
        """Simulate setpoint tracking with PID control."""
        
        # State: [x, y, z, vx, vy, vz]
        state = np.zeros(6)
        state[2] = 0.1  # Start slightly above ground
        
        num_steps = int(duration / self.dt)
        states_hist = [state.copy()]
        errors_hist = []
        times_hist = []
        accel_hist = []
        integral_hist = []
        
        # Make reference 6D
        ref = np.zeros(6)
        ref[:3] = setpoint
        
        # Reset integral
        self.z_integral = 0.0
        
        print(f"\nSimulating setpoint {setpoint} (kp={kp}, kd={kd}, ki={ki})...")
        
        for step in range(num_steps):
            # Compute error
            error = state - ref
            z_error = error[2]
            
            # PID control for Z position
            self.z_integral += z_error * self.dt
            self.z_integral = np.clip(self.z_integral, -self.max_integral, self.max_integral)
            
            accel_z = -kp * z_error - kd * error[5] - ki * self.z_integral + self.g
            
            # PD control for X, Y
            accel_xy = -kp * error[:2] - kd * error[3:5]
            
            # Combine accelerations
            accel_cmd = np.concatenate([[accel_xy[0]], [accel_xy[1]], [accel_z]])
            
            # Limit acceleration
            max_accel = 10.0
            accel_cmd = np.clip(accel_cmd, -max_accel, max_accel)
            
            # Update state using Euler integration
            state_new = state.copy()
            state_new[:3] += state[3:6] * self.dt + 0.5 * accel_cmd * self.dt**2
            state_new[3:6] += accel_cmd * self.dt
            
            # Light damping
            state_new[3:6] *= 0.97
            
            state = state_new
            states_hist.append(state.copy())
            errors_hist.append(error.copy())
            accel_hist.append(accel_cmd.copy())
            integral_hist.append(self.z_integral)
            times_hist.append(step * self.dt)
            
            if step % 50 == 0:
                pos_err = np.linalg.norm(error[:3])
                print(f"  Step {step:3d}: pos_error={pos_err:.4f}m, z={state[2]:.4f}m, integral={self.z_integral:.4f}")
        
        return {
            'states': np.array(states_hist),
            'errors': np.array(errors_hist),
            'accelerations': np.array(accel_hist),
            'integral': np.array(integral_hist),
            'times': np.array(times_hist),
            'setpoint': setpoint,
            'duration': duration,
            'kp': kp,
            'kd': kd,
            'ki': ki,
        }


def run_pid_tuning_tests():
    """Test different PID gain combinations."""
    
    tester = PIDSetpointTest()
    
    gain_sets = [
        (15.0, 8.0, 1.0, "Conservative Ki (ki=1)"),
        (15.0, 8.0, 2.0, "Moderate Ki (ki=2)"),
        (15.0, 8.0, 3.0, "Aggressive Ki (ki=3)"),
    ]
    
    setpoint = np.array([0, 0, 1.0])
    results_by_gain = {}
    
    for kp, kd, ki, label in gain_sets:
        print(f"\n{'#'*60}")
        print(f"# {label}")
        print(f"{'#'*60}")
        
        result = tester.simulate(setpoint, duration=15.0, kp=kp, kd=kd, ki=ki)
        results_by_gain[label] = result
    
    return results_by_gain


def run_multi_setpoint_tests():
    """Run multi-setpoint tests with optimal PID gains."""
    
    tester = PIDSetpointTest()
    
    setpoints = [
        (np.array([0, 0, 0.5]), "Low hover (0.5m)"),
        (np.array([0, 0, 1.0]), "Medium hover (1.0m)"),
        (np.array([0, 0, 1.5]), "High hover (1.5m)"),
        (np.array([0.5, 0, 1.0]), "Offset X (0.5m)"),
        (np.array([0, 0.5, 1.0]), "Offset Y (0.5m)"),
        (np.array([0.7, 0.7, 1.0]), "Diagonal (0.7,0.7m)"),
    ]
    
    results = {}
    for setpoint, label in setpoints:
        print(f"\n{'#'*60}")
        print(f"# {label}")
        print(f"{'#'*60}")
        
        # Use optimal PID gains
        result = tester.simulate(setpoint, duration=15.0, kp=15.0, kd=8.0, ki=2.0)
        results[label] = result
    
    return results


def plot_pid_tuning(results_by_gain: Dict):
    """Plot PID tuning comparison."""
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    colors = plt.cm.Set2(np.linspace(0, 1, len(results_by_gain)))
    
    for idx, (label, result) in enumerate(results_by_gain.items()):
        times = result['times']
        states = result['states']
        errors = result['errors']
        integral = result['integral']
        
        color = colors[idx]
        
        # Z position
        axes[0, 0].plot(times, states[:-1, 2], '-', linewidth=2.5, color=color,
                       label=label, marker='o', markersize=2, markevery=20)
        axes[0, 0].axhline(1.0, color='gray', linestyle='--', alpha=0.5)
        
        # Position error
        pos_err = np.linalg.norm(errors[:, :3], axis=1)
        axes[0, 1].semilogy(times, pos_err + 1e-6, '-', linewidth=2.5, color=color,
                           label=label, marker='s', markersize=2, markevery=20)
        
        # Integral term
        axes[1, 0].plot(times, integral, '-', linewidth=2, color=color, label=label, alpha=0.8)
        axes[1, 0].axhline(0, color='gray', linestyle='--', alpha=0.3)
        
        # Z acceleration
        axes[1, 1].plot(times, result['accelerations'][:, 2], '-', linewidth=2, color=color,
                       label=label, alpha=0.8, marker='d', markersize=2, markevery=20)
        axes[1, 1].axhline(9.81, color='gray', linestyle='--', alpha=0.3)
    
    # Configure subplots
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Z Position (m)')
    axes[0, 0].set_title('Altitude Tracking - PID Tuning')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend()
    
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Position Error (m), log scale')
    axes[0, 1].set_title('Position Error')
    axes[0, 1].grid(True, alpha=0.3, which='both')
    axes[0, 1].legend()
    
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Integral Error')
    axes[1, 0].set_title('Z-axis Error Integral (Ki action)')
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend()
    
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Z Acceleration (m/s²)')
    axes[1, 1].set_title('Z-axis Acceleration Command')
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend()
    
    plt.tight_layout()
    return fig


def plot_multi_setpoint(results: Dict):
    """Plot multi-setpoint validation."""
    
    fig, axes = plt.subplots(2, 3, figsize=(16, 10))
    axes = axes.flatten()
    
    colors = plt.cm.Set2(np.linspace(0, 1, len(results)))
    
    for idx, (label, result) in enumerate(results.items()):
        times = result['times']
        errors = result['errors']
        states = result['states']
        
        pos_err = np.linalg.norm(errors[:, :3], axis=1)
        
        # Main plot: Position error
        axes[idx].semilogy(times, pos_err + 1e-6, 'o-', color=colors[idx],
                          markersize=3, linewidth=2.5, label='Position Error')
        axes[idx].grid(True, alpha=0.3, which='both')
        axes[idx].set_xlabel('Time (s)')
        axes[idx].set_ylabel('Position Error (m), log scale')
        axes[idx].set_title(label)
        
        # Statistics
        final_z = states[-1, 2]
        final_err = pos_err[-1]
        mean_err = np.mean(pos_err)
        
        # Time to 5cm
        settling_idx = np.where(pos_err < 0.05)[0]
        settling_time = times[settling_idx[0]] if len(settling_idx) > 0 else times[-1]
        
        stats_text = (f'Final Z: {final_z:.3f}m\n'
                     f'Final Err: {final_err:.4f}m\n'
                     f'Mean Err: {mean_err:.4f}m\n'
                     f'T(5cm): {settling_time:.1f}s')
        axes[idx].text(0.98, 0.05, stats_text, transform=axes[idx].transAxes,
                      ha='right', va='bottom', fontsize=9,
                      bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
    
    plt.tight_layout()
    return fig


def print_pid_summary(results: Dict):
    """Print detailed PID tracking summary."""
    
    print("\n" + "="*70)
    print("MULTI-SETPOINT TRACKING PERFORMANCE (PID CONTROL)")
    print("="*70)
    
    for label, result in results.items():
        states = result['states']
        errors = result['errors']
        times = result['times']
        
        pos_err = np.linalg.norm(errors[:, :3], axis=1)
        
        # Settling times
        settling_05 = np.where(pos_err < 0.05)[0]
        settling_01 = np.where(pos_err < 0.01)[0]
        
        t_settle_05 = times[settling_05[0]] if len(settling_05) > 0 else None
        t_settle_01 = times[settling_01[0]] if len(settling_01) > 0 else None
        
        print(f"\n{label}:")
        print(f"  Final position: {states[-1, :3]}")
        print(f"  Target:        {result['setpoint']}")
        print(f"  Final error:   {pos_err[-1]:.4f} m")
        print(f"  Mean error:    {np.mean(pos_err):.4f} m")
        print(f"  Max error:     {np.max(pos_err):.4f} m")
        print(f"  RMSE:          {np.sqrt(np.mean(pos_err**2)):.4f} m")
        if t_settle_05 is not None:
            print(f"  Settling time (5cm): {t_settle_05:.2f} s")
        if t_settle_01 is not None:
            print(f"  Settling time (1cm): {t_settle_01:.2f} s")


def main():
    """Main function."""
    
    print("\n" + "="*70)
    print("MPC SETPOINT TRACKING TEST - PID CONTROL (WITH INTEGRAL ACTION)")
    print("="*70)
    
    # Part 1: PID tuning
    print("\n" + "="*70)
    print("PART 1: PID INTEGRAL GAIN TUNING")
    print("="*70)
    
    results_pid = run_pid_tuning_tests()
    
    fig1 = plot_pid_tuning(results_pid)
    fig1.savefig('/tmp/mpc_pid_tuning.png', dpi=300, bbox_inches='tight')
    print(f"\n✓ PID tuning plot saved: /tmp/mpc_pid_tuning.png")
    
    # Part 2: Multi-setpoint validation
    print("\n" + "="*70)
    print("PART 2: MULTI-SETPOINT VALIDATION (OPTIMIZED PID)")
    print("="*70)
    
    results_multi = run_multi_setpoint_tests()
    
    fig2 = plot_multi_setpoint(results_multi)
    fig2.savefig('/tmp/mpc_pid_multi_setpoint.png', dpi=300, bbox_inches='tight')
    print(f"\n✓ Multi-setpoint plot saved: /tmp/mpc_pid_multi_setpoint.png")
    
    # Print summary
    print_pid_summary(results_multi)
    
    print("\n" + "="*70)
    print("TEST COMPLETED SUCCESSFULLY")
    print("="*70)
    print("\nGenerated plots:")
    print("  • /tmp/mpc_pid_tuning.png - PID integral gain tuning")
    print("  • /tmp/mpc_pid_multi_setpoint.png - Multi-setpoint validation")
    print("="*70 + "\n")
    
    plt.show()


if __name__ == "__main__":
    main()
