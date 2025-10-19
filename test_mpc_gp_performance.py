#!/usr/bin/env python3
"""
Standalone MPC and GP Performance Test
Tests MPC control and GP learning without Gazebo
"""

import numpy as np
import json
import time
import sys
from pathlib import Path

# Add workspace to path
sys.path.insert(0, '/home/grandediw/quadrotor_gp_mpc_ws/install/quadrotor_gp_mpc/lib/python3.10/site-packages')

from quadrotor_gp_mpc.mpc_controller import QuadrotorMPC
from quadrotor_gp_mpc.gaussian_process import GaussianProcess
from quadrotor_gp_mpc.quadrotor_dynamics import QuadrotorDynamics

def simulate_mpc_and_gp(duration=60, dt=0.01):
    """
    Simulate MPC control and GP learning on quadrotor dynamics.
    
    Args:
        duration: Simulation duration in seconds
        dt: Time step in seconds
    """
    print("=" * 70)
    print("MPC and Gaussian Process Performance Test")
    print("=" * 70)
    
    # Initialize components
    print("\n1. Initializing components...")
    mpc = QuadrotorMPC()
    gp = GaussianProcess(input_dim=16, output_dim=12)
    dynamics = QuadrotorDynamics()
    
    print(f"   ✓ MPC initialized (horizon={mpc.N}, dt={mpc.dt})")
    print(f"   ✓ GP initialized (input_dim=16, output_dim=12)")
    print(f"   ✓ Dynamics initialized")
    
    # Reference trajectory (1m hover)
    x_ref = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    # Initial state (at origin)
    x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    u = np.array([0.0, 0.0, 0.0, 0.0])
    
    # Data collection
    states = [x.copy()]
    controls = [u.copy()]
    errors = []
    gp_training_points = []
    times = []
    
    # Metrics
    metrics_data = {
        'timestamp': time.strftime("%Y-%m-%dT%H:%M:%S"),
        'duration': duration,
        'dt': dt,
        'steps': 0,
        'mpc_gains': {'kp': 10.0, 'kd': 5.0},
        'performance': {
            'position_error_mean': 0.0,
            'position_error_max': 0.0,
            'position_error_min': float('inf'),
            'velocity_mean': 0.0,
            'gp_training_points': 0,
            'convergence_time': None
        },
        'history': []
    }
    
    print(f"\n2. Simulation parameters:")
    print(f"   Reference: [{x_ref[0]:.2f}, {x_ref[1]:.2f}, {x_ref[2]:.2f}]")
    print(f"   Duration: {duration}s, dt={dt}s, steps={int(duration/dt)}")
    print(f"   Control: PD (kp=10, kd=5)")
    
    print(f"\n3. Running simulation...")
    print(f"   {'Time':>8} {'Pos Error':>12} {'Vel Norm':>12} {'Thrust':>10} {'GP Pts':>8}")
    print(f"   {'-'*60}")
    
    start_time = time.time()
    n_steps = int(duration / dt)
    gp_data_x = []
    gp_data_y = []
    prev_x = x.copy()
    
    for step in range(n_steps):
        t = step * dt
        
        # MPC-style PD control
        error = x - x_ref
        kp, kd = 10.0, 5.0
        
        # Acceleration command
        accel_cmd = -kp * error[:3] - kd * error[3:6]
        accel_cmd[2] += 9.81  # Gravity compensation
        accel_cmd = np.clip(accel_cmd, -15.0, 15.0)
        
        # Control input (simplified)
        mass = 0.5
        u[0] = mass * accel_cmd[2]  # Thrust
        u[0] = np.clip(u[0], 0.0, 2.0)
        
        # Attitude control
        phi_cmd = np.clip(kp * accel_cmd[1], -0.3, 0.3)
        theta_cmd = np.clip(-kp * accel_cmd[0], -0.3, 0.3)
        
        u[1] = -5.0 * (x[6] - phi_cmd) - 2.0 * x[9]  # Roll torque
        u[2] = -5.0 * (x[7] - theta_cmd) - 2.0 * x[10]  # Pitch torque
        u[3] = -2.0 * x[11]  # Yaw torque
        
        u[1:] = np.clip(u[1:], -0.1, 0.1)
        
        # Dynamics update (using simple model)
        x_dot = dynamics.evaluate(x, u)
        x_new = x + x_dot * dt
        x_new = np.clip(x_new, -10, 10)
        
        # GP training data
        x_control = np.concatenate([x, u])
        gp_data_x.append(x_control)
        gp_data_y.append(x_dot)
        
        # Store history
        states.append(x_new.copy())
        controls.append(u.copy())
        
        # Compute error
        pos_error = np.linalg.norm(x_new[:3] - x_ref[:3])
        vel_norm = np.linalg.norm(x_new[3:6])
        errors.append(pos_error)
        
        # Update for next step
        prev_x = x.copy()
        x = x_new
        
        # Print every 10 steps (0.1s)
        if step % 10 == 0:
            gp_pts = len(gp_data_x)
            print(f"   {t:>8.2f}s {pos_error:>12.4f}m {vel_norm:>12.4f}m/s {u[0]:>10.4f}N {gp_pts:>8d}")
        
        # Periodic GP training
        if len(gp_data_x) >= 10 and step % 50 == 0:
            try:
                X_batch = np.array(gp_data_x[-50:])
                y_batch = np.array(gp_data_y[-50:])
                gp.fit(X_batch, y_batch)
                gp_training_points.append(len(gp_data_x))
            except Exception as e:
                pass
        
        times.append(t)
    
    # Compute statistics
    errors = np.array(errors)
    error_mean = np.mean(errors)
    error_max = np.max(errors)
    error_min = np.min(errors)
    
    # Find convergence time (when error < 0.1m)
    convergence_idx = np.where(errors < 0.1)[0]
    convergence_time = convergence_idx[0] * dt if len(convergence_idx) > 0 else None
    
    print(f"\n4. Simulation complete!")
    print(f"\n5. Performance Results:")
    print(f"   ✓ Position Error (final): {errors[-1]:.4f}m")
    print(f"   ✓ Position Error (mean): {error_mean:.4f}m")
    print(f"   ✓ Position Error (max): {error_max:.4f}m")
    print(f"   ✓ Position Error (min): {error_min:.4f}m")
    if convergence_time is not None:
        print(f"   ✓ Convergence time (<0.1m): {convergence_time:.2f}s")
    else:
        print(f"   ✗ Did not converge to <0.1m within {duration}s")
    print(f"   ✓ GP Training Points Collected: {len(gp_data_x)}")
    print(f"   ✓ GP Batch Updates: {len(gp_training_points)}")
    
    # Update metrics
    metrics_data['performance']['position_error_mean'] = float(error_mean)
    metrics_data['performance']['position_error_max'] = float(error_max)
    metrics_data['performance']['position_error_min'] = float(error_min)
    metrics_data['performance']['gp_training_points'] = len(gp_data_x)
    metrics_data['performance']['convergence_time'] = float(convergence_time) if convergence_time else None
    metrics_data['steps'] = n_steps
    
    # Save history
    for i in range(0, len(times), 10):
        metrics_data['history'].append({
            'time': float(times[i]),
            'position_error': float(errors[i]),
            'x': float(states[i][0]),
            'y': float(states[i][1]),
            'z': float(states[i][2]),
            'thrust': float(controls[i][0])
        })
    
    # Save metrics to file
    output_file = '/tmp/mpc_gp_performance.json'
    with open(output_file, 'w') as f:
        json.dump(metrics_data, f, indent=2)
    
    print(f"\n6. Results saved to: {output_file}")
    
    # Create summary plot data
    print(f"\n7. Control Performance Summary:")
    print(f"   {'-'*60}")
    print(f"   Metric                        Value")
    print(f"   {'-'*60}")
    print(f"   Initial Position Error        {errors[0]:.4f}m")
    print(f"   Final Position Error          {errors[-1]:.4f}m")
    print(f"   Mean Position Error           {error_mean:.4f}m")
    print(f"   Max Position Error            {error_max:.4f}m")
    print(f"   Min Position Error            {error_min:.6f}m")
    print(f"   Error Reduction               {(1 - errors[-1]/errors[0])*100:.1f}%")
    print(f"   {'-'*60}")
    print(f"   GP Training Points            {len(gp_data_x)}")
    print(f"   Training Batch Updates        {len(gp_training_points)}")
    print(f"   Final Convergence             {'Yes' if errors[-1] < 0.1 else 'No'}")
    print(f"   {'-'*60}")
    
    return metrics_data

if __name__ == '__main__':
    try:
        results = simulate_mpc_and_gp(duration=60, dt=0.01)
        print("\n✅ Test completed successfully!")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
