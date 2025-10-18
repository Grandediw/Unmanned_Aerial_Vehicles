#!/usr/bin/env python3
"""
Simplified MPC Setpoint Tracking Test

Tests trajectory tracking with a simple, stable linear system model.
This provides a clean baseline for control law validation.
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Dict


class SimplifiedSetpointTest:
    """Simplified setpoint tracking test using linear dynamics."""
    
    def __init__(self):
        """Initialize test."""
        self.dt = 0.1
        self.mass = 0.5
        self.g = 9.81
        
    def simulate(self, setpoint: np.ndarray, duration: float) -> Dict:
        """Simulate setpoint tracking with simple linear model."""
        
        # State: [x, y, z, vx, vy, vz]
        state = np.zeros(6)
        state[2] = 0.1  # Start slightly above ground
        
        num_steps = int(duration / self.dt)
        states_hist = [state.copy()]
        errors_hist = []
        times_hist = []
        
        # Make reference 6D
        ref = np.zeros(6)
        ref[:3] = setpoint
        
        print(f"\nSimulating setpoint {setpoint} for {duration}s...")
        
        for step in range(num_steps):
            # Compute error
            error = state - ref
            
            # Simple PD control on position
            kp_pos = 10.0  # Position gain
            kd_vel = 5.0   # Velocity gain
            
            accel_cmd = np.zeros(3)
            accel_cmd[:3] = -kp_pos * error[:3] - kd_vel * error[3:6]
            
            # Gravity compensation for Z
            accel_cmd[2] += self.g
            
            # Limit acceleration
            max_accel = 5.0
            accel_cmd = np.clip(accel_cmd, -max_accel, max_accel)
            
            # Update state using Euler integration
            state_new = state.copy()
            state_new[:3] += state[3:6] * self.dt + 0.5 * accel_cmd * self.dt**2
            state_new[3:6] += accel_cmd * self.dt
            
            # Damping to prevent oscillation
            state_new[3:6] *= 0.95
            
            state = state_new
            states_hist.append(state.copy())
            errors_hist.append(error.copy())
            times_hist.append(step * self.dt)
            
            if step % 50 == 0:
                pos_err = np.linalg.norm(error[:3])
                print(f"  Step {step:3d}: pos_error={pos_err:.4f}m, z={state[2]:.4f}m")
        
        return {
            'states': np.array(states_hist),
            'errors': np.array(errors_hist),
            'times': np.array(times_hist),
            'setpoint': setpoint,
            'duration': duration,
        }


def run_tests():
    """Run multiple setpoint tests."""
    
    tester = SimplifiedSetpointTest()
    
    setpoints = [
        np.array([0, 0, 0.5]),
        np.array([0, 0, 1.0]),
        np.array([0, 0, 1.5]),
        np.array([1.0, 0, 1.0]),
        np.array([0, 1.0, 1.0]),
        np.array([1.0, 1.0, 1.0]),
    ]
    
    labels = [
        "Low hover (0.5m)",
        "Medium hover (1.0m)",
        "High hover (1.5m)",
        "Offset X (1,0,1)",
        "Offset Y (0,1,1)",
        "Offset XY (1,1,1)",
    ]
    
    results = {}
    for setpoint, label in zip(setpoints, labels):
        result = tester.simulate(setpoint, duration=10.0)
        results[label] = result
    
    return results


def plot_results(results: Dict):
    """Plot all results."""
    
    fig, axes = plt.subplots(2, 3, figsize=(16, 10))
    axes = axes.flatten()
    
    colors = plt.cm.Set2(np.linspace(0, 1, len(results)))
    
    for idx, (label, result) in enumerate(results.items()):
        times = result['times']
        errors = result['errors']
        states = result['states']
        
        pos_err = np.linalg.norm(errors[:, :3], axis=1)
        
        # Plot
        axes[idx].semilogy(times, pos_err + 1e-6, 'o-', color=colors[idx], 
                          markersize=4, linewidth=2, label='Position Error')
        axes[idx].grid(True, alpha=0.3, which='both')
        axes[idx].set_xlabel('Time (s)')
        axes[idx].set_ylabel('Position Error (m), log scale')
        axes[idx].set_title(label)
        
        # Add statistics
        final_z = states[-1, 2]
        final_err = pos_err[-1]
        stats_text = f'Final Z: {final_z:.3f}m\nFinal Err: {final_err:.4f}m'
        axes[idx].text(0.98, 0.05, stats_text, transform=axes[idx].transAxes,
                      ha='right', va='bottom', fontsize=9,
                      bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7))
    
    plt.tight_layout()
    return fig


def plot_detailed(results: Dict):
    """Plot detailed results for first case."""
    
    first_result = list(results.values())[0]
    first_label = list(results.keys())[0]
    
    states = first_result['states']
    times = first_result['times']
    errors = first_result['errors']
    setpoint = first_result['setpoint']
    
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    # Position tracking (states has one extra point)
    axes[0].plot(times, states[:-1, 0], 'o-', markersize=3, label='X', linewidth=2)
    axes[0].plot(times, states[:-1, 1], 's-', markersize=3, label='Y', linewidth=2)
    axes[0].plot(times, states[:-1, 2], '^-', markersize=3, label='Z', linewidth=2)
    axes[0].axhline(setpoint[0], color='C0', linestyle='--', alpha=0.5, label='X ref')
    axes[0].axhline(setpoint[1], color='C1', linestyle='--', alpha=0.5, label='Y ref')
    axes[0].axhline(setpoint[2], color='C2', linestyle='--', alpha=0.5, label='Z ref')
    axes[0].set_xlabel('Time (s)')
    axes[0].set_ylabel('Position (m)')
    axes[0].set_title(f'Position Tracking - {first_label}')
    axes[0].legend(loc='best')
    axes[0].grid(True, alpha=0.3)
    
    # Position error
    pos_err = np.linalg.norm(errors[:, :3], axis=1)
    vel_err = np.linalg.norm(errors[:, 3:6], axis=1)
    
    axes[1].semilogy(times, pos_err + 1e-6, 'o-', linewidth=2, markersize=4, label='Position Error')
    axes[1].semilogy(times, vel_err + 1e-6, 's-', linewidth=2, markersize=4, label='Velocity Error')
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Error (m or m/s), log scale')
    axes[1].set_title('Tracking Errors')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3, which='both')
    
    plt.tight_layout()
    return fig


def main():
    """Main function."""
    
    print("\n" + "="*70)
    print("SIMPLIFIED MPC SETPOINT TRACKING TEST")
    print("="*70)
    print("\nTesting quadrotor control on multiple setpoints using")
    print("simple linear dynamics model for stability verification.\n")
    
    # Run simulations
    results = run_tests()
    
    # Plot results
    fig1 = plot_results(results)
    fig1.savefig('/tmp/mpc_setpoint_test.png', dpi=300, bbox_inches='tight')
    print(f"\n✓ Comparison plot saved: /tmp/mpc_setpoint_test.png")
    
    fig2 = plot_detailed(results)
    fig2.savefig('/tmp/mpc_setpoint_detailed.png', dpi=300, bbox_inches='tight')
    print(f"✓ Detailed plot saved: /tmp/mpc_setpoint_detailed.png")
    
    # Print summary
    print("\n" + "="*70)
    print("TRACKING PERFORMANCE SUMMARY")
    print("="*70)
    
    for label, result in results.items():
        states = result['states']
        errors = result['errors']
        
        pos_err = np.linalg.norm(errors[:, :3], axis=1)
        final_z = states[-1, 2]
        
        print(f"\n{label}:")
        print(f"  Initial error: {pos_err[0]:8.4f} m")
        print(f"  Final error: {pos_err[-1]:8.4f} m")
        print(f"  Mean error: {np.mean(pos_err):8.4f} m")
        print(f"  Final Z position: {final_z:8.4f} m (target: {result['setpoint'][2]:.3f} m)")
    
    print("\n" + "="*70)
    print("\nTest completed! Check /tmp/mpc_setpoint_*.png for plots")
    print("="*70 + "\n")
    
    plt.show()


if __name__ == "__main__":
    main()
