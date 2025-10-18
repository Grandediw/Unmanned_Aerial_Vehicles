#!/usr/bin/env python3
"""
Results Visualization for Quadrotor GP-MPC System

This script creates plots to visualize the performance of the quadrotor system.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time


class ResultsVisualizer(Node):
    """Visualizes quadrotor performance in real-time."""

    def __init__(self):
        super().__init__('results_visualizer')

        # Data storage
        self.max_points = 1000
        self.time_data = []
        self.position_data = []
        self.velocity_data = []
        self.attitude_data = []
        self.control_data = []
        
        self.start_time = time.time()

        # Subscribers
        self.state_sub = self.create_subscription(
            Float64MultiArray,
            '/quadrotor/state',
            self.state_callback,
            10
        )

        self.control_sub = self.create_subscription(
            Float64MultiArray,
            '/quadrotor/control',
            self.control_callback,
            10
        )

        # Current data
        self.current_state = np.zeros(12)
        self.current_control = np.zeros(4)

        self.get_logger().info("Results Visualizer initialized")

    def state_callback(self, msg):
        """Update state data."""
        self.current_state = np.array(msg.data)
        current_time = time.time() - self.start_time
        
        # Store data
        self.time_data.append(current_time)
        self.position_data.append(self.current_state[0:3].copy())
        self.velocity_data.append(self.current_state[3:6].copy())
        self.attitude_data.append(self.current_state[6:9].copy())
        
        # Keep only recent data
        if len(self.time_data) > self.max_points:
            self.time_data.pop(0)
            self.position_data.pop(0)
            self.velocity_data.pop(0)
            self.attitude_data.pop(0)

    def control_callback(self, msg):
        """Update control data."""
        self.current_control = np.array(msg.data)
        if len(self.time_data) > 0:
            self.control_data.append(self.current_control.copy())
            
            # Keep only recent data
            if len(self.control_data) > self.max_points:
                self.control_data.pop(0)

    def create_static_plots(self):
        """Create static plots of the data."""
        if len(self.time_data) < 2:
            self.get_logger().warning("Not enough data for plotting")
            return

        # Convert to numpy arrays
        times = np.array(self.time_data)
        positions = np.array(self.position_data)
        velocities = np.array(self.velocity_data)
        attitudes = np.array(self.attitude_data)
        
        if len(self.control_data) > 0:
            controls = np.array(self.control_data[-len(times):])  # Match lengths
        else:
            controls = np.zeros((len(times), 4))

        # Create figure with subplots
        fig, axes = plt.subplots(2, 3, figsize=(15, 10))
        fig.suptitle('Quadrotor GP-MPC Performance Results', fontsize=16)

        # Position plot
        axes[0, 0].plot(times, positions[:, 0], 'r-', label='X', linewidth=2)
        axes[0, 0].plot(times, positions[:, 1], 'g-', label='Y', linewidth=2)
        axes[0, 0].plot(times, positions[:, 2], 'b-', label='Z', linewidth=2)
        axes[0, 0].axhline(y=1.0, color='b', linestyle='--', alpha=0.5, label='Z target')
        axes[0, 0].set_title('Position')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Position (m)')
        axes[0, 0].legend()
        axes[0, 0].grid(True)

        # Velocity plot
        axes[0, 1].plot(times, velocities[:, 0], 'r-', label='Vx', linewidth=2)
        axes[0, 1].plot(times, velocities[:, 1], 'g-', label='Vy', linewidth=2)
        axes[0, 1].plot(times, velocities[:, 2], 'b-', label='Vz', linewidth=2)
        axes[0, 1].set_title('Velocity')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Velocity (m/s)')
        axes[0, 1].legend()
        axes[0, 1].grid(True)

        # Attitude plot
        axes[0, 2].plot(times, np.degrees(attitudes[:, 0]), 'r-', label='Roll', linewidth=2)
        axes[0, 2].plot(times, np.degrees(attitudes[:, 1]), 'g-', label='Pitch', linewidth=2)
        axes[0, 2].plot(times, np.degrees(attitudes[:, 2]), 'b-', label='Yaw', linewidth=2)
        axes[0, 2].set_title('Attitude')
        axes[0, 2].set_xlabel('Time (s)')
        axes[0, 2].set_ylabel('Angle (degrees)')
        axes[0, 2].legend()
        axes[0, 2].grid(True)

        # Control inputs
        if len(controls) > 0:
            axes[1, 0].plot(times, controls[:, 0], 'k-', label='Thrust', linewidth=2)
            axes[1, 0].axhline(y=4.905, color='k', linestyle='--', alpha=0.5, label='Hover thrust')
            axes[1, 0].set_title('Thrust')
            axes[1, 0].set_xlabel('Time (s)')
            axes[1, 0].set_ylabel('Thrust (N)')
            axes[1, 0].legend()
            axes[1, 0].grid(True)

            axes[1, 1].plot(times, controls[:, 1], 'r-', label='τ_x', linewidth=2)
            axes[1, 1].plot(times, controls[:, 2], 'g-', label='τ_y', linewidth=2)
            axes[1, 1].plot(times, controls[:, 3], 'b-', label='τ_z', linewidth=2)
            axes[1, 1].set_title('Torques')
            axes[1, 1].set_xlabel('Time (s)')
            axes[1, 1].set_ylabel('Torque (N⋅m)')
            axes[1, 1].legend()
            axes[1, 1].grid(True)

        # 3D trajectory
        axes[1, 2].remove()  # Remove the 2D axis
        ax_3d = fig.add_subplot(2, 3, 6, projection='3d')
        ax_3d.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b-', linewidth=2, alpha=0.7)
        ax_3d.scatter(positions[0, 0], positions[0, 1], positions[0, 2], color='g', s=100, label='Start')
        ax_3d.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], color='r', s=100, label='End')
        ax_3d.scatter(0, 0, 1, color='orange', s=100, marker='*', label='Target')
        ax_3d.set_title('3D Trajectory')
        ax_3d.set_xlabel('X (m)')
        ax_3d.set_ylabel('Y (m)')
        ax_3d.set_zlabel('Z (m)')
        ax_3d.legend()

        plt.tight_layout()
        plt.savefig('/home/grandediw/quadrotor_gp_mpc_ws/quadrotor_results.png', dpi=300, bbox_inches='tight')
        plt.show()

        # Print summary statistics
        self.print_performance_summary(times, positions, velocities, controls)

    def print_performance_summary(self, times, positions, velocities, controls):
        """Print performance summary statistics."""
        print("\n" + "="*60)
        print("QUADROTOR GP-MPC PERFORMANCE SUMMARY")
        print("="*60)
        
        # Flight duration
        print(f"Flight Duration: {times[-1]:.1f} seconds")
        
        # Position tracking
        target = np.array([0, 0, 1])
        pos_errors = np.linalg.norm(positions - target, axis=1)
        print(f"\nPosition Tracking:")
        print(f"  Mean Error: {np.mean(pos_errors):.3f} m")
        print(f"  Max Error:  {np.max(pos_errors):.3f} m")
        print(f"  RMS Error:  {np.sqrt(np.mean(pos_errors**2)):.3f} m")
        
        # Final position
        final_pos = positions[-1]
        print(f"\nFinal Position: [{final_pos[0]:.3f}, {final_pos[1]:.3f}, {final_pos[2]:.3f}] m")
        
        # Velocity statistics
        speeds = np.linalg.norm(velocities, axis=1)
        print(f"\nVelocity Statistics:")
        print(f"  Mean Speed: {np.mean(speeds):.3f} m/s")
        print(f"  Max Speed:  {np.max(speeds):.3f} m/s")
        
        # Control effort
        if len(controls) > 0:
            thrust_mean = np.mean(controls[:, 0])
            thrust_std = np.std(controls[:, 0])
            torque_rms = np.sqrt(np.mean(np.sum(controls[:, 1:4]**2, axis=1)))
            print(f"\nControl Effort:")
            print(f"  Mean Thrust: {thrust_mean:.3f} N (hover: 4.905 N)")
            print(f"  Thrust Std:  {thrust_std:.3f} N")
            print(f"  RMS Torque:  {torque_rms:.3f} N⋅m")
        
        print("="*60)


def main(args=None):
    rclpy.init(args=args)
    visualizer = ResultsVisualizer()
    
    print("Results Visualizer started. Collecting data...")
    print("Press Ctrl+C when you want to generate plots.")
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        print("\nGenerating performance plots...")
        visualizer.create_static_plots()
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()