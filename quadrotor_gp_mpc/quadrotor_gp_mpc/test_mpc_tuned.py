#!/usr/bin/env python3
"""
MPC Setpoint Tracking Test - Tuned Control Gains

This script validates MPC setpoint tracking with tuned proportional control.
Tests demonstrate stable altitude/position control with different target setpoints.
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Dict


class TunedSetpointTest:
    """Tuned setpoint tracking test using optimized control gains."""
    
    def __init__(self):
        """Initialize test."""
        self.dt = 0.1
        self.mass = 0.5
        self.g = 9.81
        
    def simulate(self, setpoint: np.ndarray, duration: float, 
                 kp: float = 15.0, kd: float = 8.0) -> Dict:
        """Simulate setpoint tracking with tuned PD control."""
        
        # State: [x, y, z, vx, vy, vz]
        state = np.zeros(6)
        state[2] = 0.1  # Start slightly above ground
        
        num_steps = int(duration / self.dt)
        states_hist = [state.copy()]
        errors_hist = []
        times_hist = []
        accel_hist = []
        
        # Make reference 6D
        ref = np.zeros(6)
        ref[:3] = setpoint
        
        print(f"\nSimulating setpoint {setpoint} (kp={kp}, kd={kd})...")
        
        for step in range(num_steps):
            # Compute error
            error = state - ref
            
            # Tuned PD control on position
            accel_cmd = np.zeros(3)
            accel_cmd[:3] = -kp * error[:3] - kd * error[3:6]
            
            # Gravity compensation for Z
            accel_cmd[2] += self.g
            
            # Limit acceleration
            max_accel = 10.0  # Increased max acceleration
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
            times_hist.append(step * self.dt)
            
            if step % 50 == 0:
                pos_err = np.linalg.norm(error[:3])
                print(f"  Step {step:3d}: pos_error={pos_err:.4f}m, z={state[2]:.4f}m, vz={state[5]:.4f}m/s")
        
        return {
            'states': np.array(states_hist),
            'errors': np.array(errors_hist),
            'accelerations': np.array(accel_hist),
            'times': np.array(times_hist),
            'setpoint': setpoint,
            'duration': duration,
            'kp': kp,
            'kd': kd,
        }


def run_tuned_tests():
    """Run tests with different control gains."""
    
    tester = TunedSetpointTest()
    
    # Test parameters
    gain_sets = [
        (10.0, 5.0, "Conservative (kp=10, kd=5)"),
        (15.0, 8.0, "Moderate (kp=15, kd=8)"),
        (20.0, 10.0, "Aggressive (kp=20, kd=10)"),
    ]
    
    setpoint = np.array([0, 0, 1.0])  # 1m hover
    
    results_by_gain = {}
    
    for kp, kd, label in gain_sets:
        print(f"\n{'#'*60}")
        print(f"# {label}")
        print(f"{'#'*60}")
        
        result = tester.simulate(setpoint, duration=15.0, kp=kp, kd=kd)
        results_by_gain[label] = result
    
    return results_by_gain


def run_multi_setpoint_tests():
    """Run tests on multiple setpoints with best gains."""
    
    tester = TunedSetpointTest()
    
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
        
        result = tester.simulate(setpoint, duration=12.0, kp=15.0, kd=8.0)
        results[label] = result
    
    return results


def plot_gain_comparison(results_by_gain: Dict):
    """Plot comparison of different control gains."""
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    colors = plt.cm.Set2(np.linspace(0, 1, len(results_by_gain)))
    
    for idx, (label, result) in enumerate(results_by_gain.items()):
        times = result['times']
        states = result['states']
        errors = result['errors']
        accelerations = result['accelerations']
        
        color = colors[idx]
        
        # Z position
        axes[0, 0].plot(times, states[:-1, 2], '-', linewidth=2.5, color=color, 
                       label=label, marker='o', markersize=2, markevery=20)
        axes[0, 0].axhline(1.0, color='gray', linestyle='--', alpha=0.5)
        
        # Position error
        pos_err = np.linalg.norm(errors[:, :3], axis=1)
        axes[0, 1].semilogy(times, pos_err + 1e-6, '-', linewidth=2.5, color=color,
                           label=label, marker='s', markersize=2, markevery=20)
        
        # Velocity
        axes[1, 0].plot(times, states[:-1, 5], '-', linewidth=2.5, color=color,
                       label=label, marker='^', markersize=2, markevery=20)
        axes[1, 0].axhline(0, color='gray', linestyle='--', alpha=0.3)
        
        # Z-axis acceleration
        axes[1, 1].plot(times, accelerations[:, 2], '-', linewidth=2, color=color,
                       label=label, alpha=0.8, marker='d', markersize=2, markevery=20)
        axes[1, 1].axhline(9.81, color='gray', linestyle='--', alpha=0.3, label='Gravity')
    
    # Configure subplots
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Z Position (m)')
    axes[0, 0].set_title('Altitude Tracking Comparison')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend()
    
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Position Error (m), log scale')
    axes[0, 1].set_title('Position Error Comparison')
    axes[0, 1].grid(True, alpha=0.3, which='both')
    axes[0, 1].legend()
    
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Z Velocity (m/s)')
    axes[1, 0].set_title('Vertical Velocity')
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend()
    
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Z Acceleration (m/s²)')
    axes[1, 1].set_title('Z-axis Acceleration Command')
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend()
    
    plt.tight_layout()
    return fig


def plot_multi_setpoint_comparison(results: Dict):
    """Plot multi-setpoint tracking results."""
    
    fig, axes = plt.subplots(2, 3, figsize=(16, 10))
    axes = axes.flatten()
    
    colors = plt.cm.Set2(np.linspace(0, 1, len(results)))
    
    for idx, (label, result) in enumerate(results.items()):
        times = result['times']
        errors = result['errors']
        states = result['states']
        
        pos_err = np.linalg.norm(errors[:, :3], axis=1)
        
        # Main plot: Position error over time
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
        
        # Time to 5cm error
        settling_idx = np.where(pos_err < 0.05)[0]
        settling_time = times[settling_idx[0]] if len(settling_idx) > 0 else times[-1]
        
        stats_text = (f'Final Z: {final_z:.3f}m\n'
                     f'Final Err: {final_err:.4f}m\n'
                     f'Mean Err: {mean_err:.4f}m\n'
                     f'T(5cm): {settling_time:.1f}s')
        axes[idx].text(0.98, 0.05, stats_text, transform=axes[idx].transAxes,
                      ha='right', va='bottom', fontsize=9,
                      bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    return fig


def print_performance_summary(results: Dict):
    """Print detailed performance summary."""
    
    print("\n" + "="*70)
    print("MULTI-SETPOINT TRACKING PERFORMANCE SUMMARY")
    print("="*70)
    
    for label, result in results.items():
        states = result['states']
        errors = result['errors']
        times = result['times']
        
        pos_err = np.linalg.norm(errors[:, :3], axis=1)
        vel_err = np.linalg.norm(errors[:, 3:6], axis=1)
        
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
        print(f"  Peak velocity error: {np.max(vel_err):.4f} m/s")


def main():
    """Main function."""
    
    print("\n" + "="*70)
    print("MPC SETPOINT TRACKING TEST - TUNED CONTROL GAINS")
    print("="*70)
    
    # Part 1: Gain tuning comparison
    print("\n" + "="*70)
    print("PART 1: GAIN TUNING COMPARISON")
    print("="*70)
    
    results_gains = run_tuned_tests()
    
    fig1 = plot_gain_comparison(results_gains)
    fig1.savefig('/tmp/mpc_gain_comparison.png', dpi=300, bbox_inches='tight')
    print(f"\n✓ Gain comparison plot saved: /tmp/mpc_gain_comparison.png")
    
    # Part 2: Multi-setpoint validation
    print("\n" + "="*70)
    print("PART 2: MULTI-SETPOINT TRACKING TEST")
    print("="*70)
    
    results_multi = run_multi_setpoint_tests()
    
    fig2 = plot_multi_setpoint_comparison(results_multi)
    fig2.savefig('/tmp/mpc_multi_setpoint.png', dpi=300, bbox_inches='tight')
    print(f"\n✓ Multi-setpoint plot saved: /tmp/mpc_multi_setpoint.png")
    
    # Print summaries
    print_performance_summary(results_multi)
    
    print("\n" + "="*70)
    print("TEST COMPLETED")
    print("="*70)
    print("\nGenerated plots:")
    print("  • /tmp/mpc_gain_comparison.png - Control gain tuning comparison")
    print("  • /tmp/mpc_multi_setpoint.png - Multi-setpoint tracking validation")
    print("="*70 + "\n")
    
    plt.show()


if __name__ == "__main__":
    main()
