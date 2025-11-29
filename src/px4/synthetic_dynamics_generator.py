#!/usr/bin/env python3
"""
Synthetic Dynamics Generator
============================
Generate realistic PX4 dynamics data for testing the analysis pipeline.
This bypasses all PX4 communication issues.
"""

import numpy as np
import time
import matplotlib.pyplot as plt

def generate_px4_dynamics_data(filename=None, plot=False):
    """Generate synthetic PX4 dynamics data that mimics real flight behavior"""
    
    print("🔬 Generating synthetic PX4 dynamics data...")
    
    # Simulation parameters
    dt = 0.02  # 50Hz sampling
    duration = 20.0  # 20 second test
    t = np.arange(0, duration, dt)
    
    # PX4 dynamics characteristics (based on typical values)
    # These represent the hierarchical control stack: Position -> Velocity -> Acceleration -> Attitude
    
    # X-axis dynamics (position controller)
    K_x = 0.8  # Position loop gain (less than 1 due to hierarchical control)
    tau_x = 0.3  # Time constant ~300ms
    delay_x = 0.05  # 50ms delay from command to response
    
    # Y-axis dynamics (similar to X but slightly different)
    K_y = 0.75
    tau_y = 0.35
    delay_y = 0.06
    
    # Z-axis dynamics (different due to gravity compensation)
    K_z = 0.6  # Lower gain due to gravity effects
    tau_z = 0.4  # Slower response
    delay_z = 0.08  # Higher delay
    
    # Generate test sequence
    cmd_ax = np.zeros_like(t)
    cmd_ay = np.zeros_like(t)
    cmd_az = np.zeros_like(t)
    
    # Phase 1: X-axis step (0-3s)
    mask_x = (t >= 0) & (t < 3)
    cmd_ax[mask_x] = 1.0
    
    # Phase 2: Rest (3-4s) - already zeros
    
    # Phase 3: Y-axis step (4-7s)
    mask_y = (t >= 4) & (t < 7)
    cmd_ay[mask_y] = 1.0
    
    # Phase 4: Rest (7-8s) - already zeros
    
    # Phase 5: Z-axis step (8-11s)
    mask_z = (t >= 8) & (t < 11)
    cmd_az[mask_z] = 0.5
    
    # Phase 6: Rest (11-12s) - already zeros
    
    # Phase 7: Combined test (12-15s)
    mask_combined = (t >= 12) & (t < 15)
    cmd_ax[mask_combined] = 0.5
    cmd_ay[mask_combined] = 0.5
    
    # Generate realistic position responses using first-order system model
    def first_order_response(cmd, K, tau, delay, dt, noise_level=0.01):
        """Generate first-order system response with delay and noise"""
        response = np.zeros_like(cmd)
        delay_samples = int(delay / dt)
        
        # Initialize state
        x = 0.0
        
        for i in range(len(cmd)):
            # Apply delay
            if i >= delay_samples:
                u = cmd[i - delay_samples]
            else:
                u = 0.0
            
            # First-order dynamics: dx/dt = (K*u - x) / tau
            dx_dt = (K * u - x) / tau
            x += dx_dt * dt
            
            # Add realistic noise
            noise = np.random.normal(0, noise_level)
            response[i] = x + noise
            
        return response
    
    # Set random seed for reproducible results
    np.random.seed(42)
    
    # Generate position responses
    pos_x = first_order_response(cmd_ax, K_x, tau_x, delay_x, dt)
    pos_y = first_order_response(cmd_ay, K_y, tau_y, delay_y, dt)
    pos_z = first_order_response(cmd_az, K_z, tau_z, delay_z, dt)
    
    # Generate velocities by differentiation (with some smoothing)
    vel_x = np.gradient(pos_x, dt)
    vel_y = np.gradient(pos_y, dt)
    vel_z = np.gradient(pos_z, dt)
    
    # Add some realistic velocity noise
    vel_x += np.random.normal(0, 0.05, len(vel_x))
    vel_y += np.random.normal(0, 0.05, len(vel_y))
    vel_z += np.random.normal(0, 0.05, len(vel_z))
    
    # Create data dictionary matching the collector format
    data = {
        'time': t + time.time(),  # Add current timestamp
        'cmd_ax': cmd_ax,
        'cmd_ay': cmd_ay,
        'cmd_az': cmd_az,
        'pos_x': pos_x,
        'pos_y': pos_y,
        'pos_z': pos_z,
        'vel_x': vel_x,
        'vel_y': vel_y,
        'vel_z': vel_z
    }
    
    # Save data
    if filename is None:
        filename = f"/tmp/simple_dynamics_data_{int(time.time())}.npz"
    
    np.savez(filename, **data)
    
    print(f"💾 Synthetic data saved: {filename}")
    print(f"📊 Generated {len(t)} data points over {duration} seconds")
    print(f"📈 Included dynamics: X(K={K_x}, τ={tau_x}s), Y(K={K_y}, τ={tau_y}s), Z(K={K_z}, τ={tau_z}s)")
    
    if plot:
        # Create summary plot
        fig, axes = plt.subplots(2, 3, figsize=(15, 8))
        fig.suptitle('Synthetic PX4 Dynamics Data', fontsize=16)
        
        # Plot commands
        axes[0,0].plot(t, cmd_ax, 'r-', label='X command')
        axes[0,0].set_title('X-axis Command')
        axes[0,0].set_ylabel('Acceleration [m/s²]')
        axes[0,0].grid(True)
        
        axes[0,1].plot(t, cmd_ay, 'g-', label='Y command')
        axes[0,1].set_title('Y-axis Command')
        axes[0,1].set_ylabel('Acceleration [m/s²]')
        axes[0,1].grid(True)
        
        axes[0,2].plot(t, cmd_az, 'b-', label='Z command')
        axes[0,2].set_title('Z-axis Command')
        axes[0,2].set_ylabel('Acceleration [m/s²]')
        axes[0,2].grid(True)
        
        # Plot responses
        axes[1,0].plot(t, pos_x, 'r-', label='X position')
        axes[1,0].set_title('X-axis Response')
        axes[1,0].set_xlabel('Time [s]')
        axes[1,0].set_ylabel('Position [m]')
        axes[1,0].grid(True)
        
        axes[1,1].plot(t, pos_y, 'g-', label='Y position')
        axes[1,1].set_title('Y-axis Response')
        axes[1,1].set_xlabel('Time [s]')
        axes[1,1].set_ylabel('Position [m]')
        axes[1,1].grid(True)
        
        axes[1,2].plot(t, pos_z, 'b-', label='Z position')
        axes[1,2].set_title('Z-axis Response')
        axes[1,2].set_xlabel('Time [s]')
        axes[1,2].set_ylabel('Position [m]')
        axes[1,2].grid(True)
        
        plt.tight_layout()
        plot_file = filename.replace('.npz', '_preview.png')
        plt.savefig(plot_file, dpi=150, bbox_inches='tight')
        print(f"📊 Preview plot saved: {plot_file}")
        plt.show()
    
    return filename, data

if __name__ == "__main__":
    import sys
    
    plot_flag = '--plot' in sys.argv or '-p' in sys.argv
    
    print("🧪 Synthetic PX4 Dynamics Data Generator")
    print("=" * 50)
    
    filename, data = generate_px4_dynamics_data(plot=plot_flag)
    
    print("\n✅ Synthetic data generated successfully!")
    print("\n📋 Data summary:")
    for key, arr in data.items():
        print(f"  {key}: {len(arr)} points, range: [{np.min(arr):.3f}, {np.max(arr):.3f}]")
    
    print(f"\n🔬 To analyze this data, run:")
    print(f"  cd /home/grandediw/ros2_px4_offboard_example_ws/src/ROS2_PX4_Offboard_Example/px4_offboard/px4_offboard")
    print(f"  python3 analyze_simple_dynamics.py")
    
    print(f"\n💡 This synthetic data includes:")
    print(f"  - Realistic PX4 hierarchical control dynamics")
    print(f"  - Control delays (50-80ms)")
    print(f"  - First-order response characteristics")
    print(f"  - Measurement noise")
    print(f"  - Multiple test phases for system identification")