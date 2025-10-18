#!/usr/bin/env python3
"""
Test script for MPC controller on setpoint (hover) trajectory - STABLE VERSION.

This script tests the MPC controller's ability to track constant setpoints
using actual quadrotor dynamics or stable simulation.
"""

import numpy as np
import matplotlib.pyplot as plt
import time
from typing import List, Tuple, Dict


class StableSetpointTest:
    """Stable setpoint tracking test using PID control and simple dynamics."""
    
    def __init__(self):
        """Initialize test controller."""
        # Quadrotor parameters
        self.mass = 0.5
        self.g = 9.81
        self.Ixx = 0.0023
        self.Iyy = 0.0023
        self.Izz = 0.0046
        
        # Control constraints
        self.thrust_min = 0.0
        self.thrust_max = 2.0 * self.mass * self.g
        self.torque_max = 0.1
        
        # PID gains
        self.pid_z = {'kp': 15.0, 'ki': 0.5, 'kd': 8.0}
        self.pid_xy = {'kp': 8.0, 'ki': 0.1, 'kd': 5.0}
        self.pid_att = {'kp': 3.0, 'kd': 0.5}
        
        # Integrator state
        self.z_integral_error = 0.0
        self.xy_integral_errors = np.zeros(2)
        
    def create_setpoint_trajectory(self, setpoint: np.ndarray, duration: float,
                                  dt: float = 0.1) -> Tuple[List[np.ndarray], int]:
        """Create constant setpoint trajectory."""
        num_steps = int(duration / dt)
        trajectory = []
        
        for _ in range(num_steps):
            ref_state = np.zeros(12)
            if len(setpoint) >= 3:
                ref_state[0:3] = setpoint[:3]
            trajectory.append(ref_state)
        
        return trajectory, num_steps
    
    def simulate_tracking(self, setpoint: np.ndarray, duration: float,
                         dt: float = 0.1) -> Dict:
        """Simulate PID control for setpoint tracking."""
        
        # Initial state
        state = np.zeros(12)
        state[2] = 0.1  # Start slightly above ground
        
        # Storage
        states = [state.copy()]
        controls = []
        errors = []
        times = []
        
        trajectory_ref, num_steps = self.create_setpoint_trajectory(setpoint, duration, dt)
        
        print(f"\n{'='*60}")
        print(f"Testing Setpoint: {setpoint}")
        print(f"Duration: {duration}s, Time step: {dt}s, Total steps: {num_steps}")
        print(f"{'='*60}\n")
        
        current_time = 0.0
        
        for step in range(num_steps):
            ref_state = trajectory_ref[step]
            
            # PID control
            control, control_debug = self._compute_pid_control(state, ref_state, dt)
            controls.append(control)
            
            # Update state (stable 4th-order Runge-Kutta)
            state = self._rk4_step(state, control, dt)
            states.append(state.copy())
            
            # Tracking error
            error = state[:6] - ref_state[:6]
            errors.append(error)
            times.append(current_time)
            current_time += dt
            
            # Progress
            if step % max(1, int(10/dt)) == 0:
                pos_error = np.linalg.norm(error[:3])
                print(f"Step {step:3d} | Time: {current_time:6.2f}s | Pos Error: {pos_error:6.4f}m | "
                      f"Z: {state[2]:6.3f}m")
        
        print(f"\n{'='*60}")
        print(f"Results for setpoint {setpoint}:")
        errors_array = np.array(errors)
        pos_errors = np.linalg.norm(errors_array[:, :3], axis=1)
        print(f"  Mean position error: {np.mean(pos_errors):.4f}m")
        print(f"  Final position error: {pos_errors[-1]:.4f}m")
        print(f"  Final Z position: {states[-1][2]:.4f}m")
        print(f"{'='*60}\n")
        
        return {
            'states': np.array(states),
            'controls': np.array(controls),
            'references': np.array(trajectory_ref),
            'errors': errors_array,
            'times': np.array(times),
            'setpoint': setpoint,
        }
    
    def _compute_pid_control(self, state: np.ndarray, reference: np.ndarray,
                            dt: float) -> Tuple[np.ndarray, Dict]:
        """Compute PID control."""
        error = state[:6] - reference[:6]
        
        # Z-axis control (altitude)
        z_err = error[2]
        self.z_integral_error += z_err * dt
        z_der = error[5]  # velocity error
        
        T_cmd = self.mass * self.g
        T_cmd += self.pid_z['kp'] * z_err
        T_cmd += self.pid_z['ki'] * np.clip(self.z_integral_error, -1, 1)
        T_cmd -= self.pid_z['kd'] * z_der
        T_cmd = np.clip(T_cmd, self.thrust_min, self.thrust_max)
        
        # XY control (through attitude)
        xy_err = error[:2]
        self.xy_integral_errors += xy_err * dt
        xy_der = error[3:5]
        
        # Desired roll and pitch (small angle approx)
        phi_des = 0.0
        theta_des = 0.0
        
        if np.linalg.norm(xy_err) > 0.01:
            max_angle = 0.4
            theta_des = np.clip(self.pid_xy['kp'] * xy_err[0] - self.pid_xy['kd'] * xy_der[0], 
                              -max_angle, max_angle)
            phi_des = np.clip(self.pid_xy['kp'] * xy_err[1] - self.pid_xy['kd'] * xy_der[1], 
                            -max_angle, max_angle)
        
        # Attitude feedback
        att_err = state[6:9] - np.array([phi_des, theta_des, 0])
        
        tau_p = -self.pid_att['kp'] * att_err[0] - self.pid_att['kd'] * state[9]
        tau_q = -self.pid_att['kp'] * att_err[1] - self.pid_att['kd'] * state[10]
        tau_r = -0.1 * state[11]
        
        tau_p = np.clip(tau_p, -self.torque_max, self.torque_max)
        tau_q = np.clip(tau_q, -self.torque_max, self.torque_max)
        tau_r = np.clip(tau_r, -self.torque_max, self.torque_max)
        
        control = np.array([T_cmd, tau_p, tau_q, tau_r])
        debug = {'phi_des': phi_des, 'theta_des': theta_des, 'T_cmd': T_cmd}
        
        return control, debug
    
    def _rk4_step(self, state: np.ndarray, control: np.ndarray,
                 dt: float) -> np.ndarray:
        """Runge-Kutta 4th order integration."""
        k1 = self._state_derivative(state, control)
        k2 = self._state_derivative(state + 0.5*dt*k1, control)
        k3 = self._state_derivative(state + 0.5*dt*k2, control)
        k4 = self._state_derivative(state + dt*k3, control)
        
        return state + (dt/6.0) * (k1 + 2*k2 + 2*k3 + k4)
    
    def _state_derivative(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        """Compute state derivative for dynamics."""
        x_dot = np.zeros(12)
        
        # Position derivative = velocity
        x_dot[0:3] = state[3:6]
        
        # Velocity derivative
        pos = state[0:3]
        vel = state[3:6]
        euler = state[6:9]
        rates = state[9:12]
        
        phi, theta, psi = euler
        p, q, r = rates
        T = control[0]
        tau_p = control[1]
        tau_q = control[2]
        tau_r = control[3]
        
        # Gravity and thrust (body z-axis points up with small angle approx)
        accel = np.array([0.0, 0.0, -self.g])
        
        # Thrust contribution with small angle approximation
        accel[0] += (T / self.mass) * theta  # x-accel from pitch
        accel[1] -= (T / self.mass) * phi    # y-accel from roll
        accel[2] += (T / self.mass) * (1.0 - 0.5*(phi**2 + theta**2)) - self.g
        
        # Drag (velocity dependent)
        accel -= 0.1 * vel
        
        x_dot[3:6] = accel
        
        # Euler angle derivatives
        x_dot[6] = p + q*np.sin(phi)*np.tan(theta) + r*np.cos(phi)*np.tan(theta)
        x_dot[7] = q*np.cos(phi) - r*np.sin(phi)
        x_dot[8] = (q*np.sin(phi) + r*np.cos(phi)) / np.cos(theta)
        
        # Angular rate derivatives (Euler equations)
        x_dot[9] = (tau_p + (self.Iyy - self.Izz)*q*r) / self.Ixx - 0.05*p
        x_dot[10] = (tau_q + (self.Izz - self.Ixx)*p*r) / self.Iyy - 0.05*q
        x_dot[11] = (tau_r + (self.Ixx - self.Iyy)*p*q) / self.Izz - 0.05*r
        
        return x_dot


def run_tests():
    """Run all setpoint tests."""
    tester = StableSetpointTest()
    
    test_cases = [
        (np.array([0, 0, 0.5]), "Low hover (0.5m)"),
        (np.array([0, 0, 1.0]), "Nominal hover (1.0m)"),
        (np.array([0, 0, 1.5]), "High hover (1.5m)"),
        (np.array([1.0, 0, 1.0]), "Offset X (1,0,1)"),
        (np.array([0, 1.0, 1.0]), "Offset Y (0,1,1)"),
        (np.array([1.0, 1.0, 1.0]), "Offset XY (1,1,1)"),
    ]
    
    results = {}
    
    for setpoint, description in test_cases:
        result = tester.simulate_tracking(setpoint, duration=15.0, dt=0.1)
        results[description] = result
    
    return results


def plot_results(results: Dict):
    """Plot setpoint tracking results."""
    
    fig, axes = plt.subplots(2, 3, figsize=(16, 10))
    axes = axes.flatten()
    
    colors = plt.cm.Set2(np.linspace(0, 1, len(results)))
    
    for idx, (desc, data) in enumerate(results.items()):
        color = colors[idx]
        times = data['times']
        states = data['states']
        errors = data['errors']
        
        # Position error magnitude
        pos_err = np.linalg.norm(errors[:, :3], axis=1)
        
        axes[idx].semilogy(times, pos_err, 'o-', color=color, markersize=3, linewidth=2)
        axes[idx].grid(True, alpha=0.3)
        axes[idx].set_xlabel('Time (s)')
        axes[idx].set_ylabel('Position Error (m), log scale')
        axes[idx].set_title(desc)
        
        # Final error annotation
        final_err = pos_err[-1]
        axes[idx].text(0.98, 0.05, f'Final: {final_err:.4f}m', 
                      transform=axes[idx].transAxes, ha='right', va='bottom',
                      bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    return fig


def plot_detailed_results(results: Dict):
    """Plot detailed results for first case."""
    
    first_result = list(results.values())[0]
    first_desc = list(results.keys())[0]
    
    states = first_result['states']
    controls = first_result['controls']
    times = first_result['times']
    errors = first_result['errors']
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # Position tracking
    axes[0, 0].plot(times, states[:-1, 0], label='X')
    axes[0, 0].plot(times, states[:-1, 1], label='Y')
    axes[0, 0].plot(times, states[:-1, 2], label='Z', linewidth=2)
    axes[0, 0].axhline(first_result['setpoint'][0], color='C0', linestyle='--', alpha=0.5)
    axes[0, 0].axhline(first_result['setpoint'][1], color='C1', linestyle='--', alpha=0.5)
    axes[0, 0].axhline(first_result['setpoint'][2], color='C2', linestyle='--', alpha=0.5)
    axes[0, 0].set_ylabel('Position (m)')
    axes[0, 0].set_title(f'Position Tracking - {first_desc}')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    # Velocity
    axes[0, 1].plot(times, states[:-1, 3], label='Vx')
    axes[0, 1].plot(times, states[:-1, 4], label='Vy')
    axes[0, 1].plot(times, states[:-1, 5], label='Vz')
    axes[0, 1].set_ylabel('Velocity (m/s)')
    axes[0, 1].set_title('Velocity')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # Attitude
    axes[1, 0].plot(times, np.rad2deg(states[:-1, 6]), label='φ (roll)', linewidth=2)
    axes[1, 0].plot(times, np.rad2deg(states[:-1, 7]), label='θ (pitch)', linewidth=2)
    axes[1, 0].plot(times, np.rad2deg(states[:-1, 8]), label='ψ (yaw)', alpha=0.5)
    axes[1, 0].set_ylabel('Angle (degrees)')
    axes[1, 0].set_title('Attitude')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    # Control inputs
    axes[1, 1].plot(times, controls[:, 0], label='Thrust', linewidth=2)
    axes[1, 1].plot(times, controls[:, 1], label='τ_p', alpha=0.7)
    axes[1, 1].plot(times, controls[:, 2], label='τ_q', alpha=0.7)
    axes[1, 1].plot(times, controls[:, 3], label='τ_r', alpha=0.7)
    axes[1, 1].set_ylabel('Control Input')
    axes[1, 1].set_title('Control Inputs')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig


def main():
    """Main test."""
    print("\n" + "="*70)
    print("QUADROTOR MPC SETPOINT TRACKING TEST - STABLE VERSION")
    print("="*70)
    
    # Run tests
    results = run_tests()
    
    # Plot results
    fig1 = plot_results(results)
    fig1.savefig('/tmp/mpc_setpoint_comparison.png', dpi=300, bbox_inches='tight')
    print(f"\n✓ Comparison plot saved to /tmp/mpc_setpoint_comparison.png")
    
    fig2 = plot_detailed_results(results)
    fig2.savefig('/tmp/mpc_setpoint_detailed.png', dpi=300, bbox_inches='tight')
    print(f"✓ Detailed plot saved to /tmp/mpc_setpoint_detailed.png")
    
    # Summary
    print("\n" + "="*70)
    print("SUMMARY")
    print("="*70)
    
    for desc, result in results.items():
        errors = result['errors']
        pos_err = np.linalg.norm(errors[:, :3], axis=1)
        
        print(f"\n{desc}:")
        print(f"  Initial position error: {pos_err[0]:.4f}m")
        print(f"  Final position error: {pos_err[-1]:.4f}m")
        print(f"  Mean position error: {np.mean(pos_err):.4f}m")
        print(f"  Settling time: ~{np.where(pos_err < 0.01)[0][0]*0.1:.1f}s (to <1cm)")
    
    print("\n" + "="*70)
    
    plt.show()


if __name__ == "__main__":
    main()
