#!/usr/bin/env python3
"""
Reference Trajectory Generator Node.

Generates reference trajectories for the MPC controller to track.
Supports multiple trajectory types:
- Setpoint (constant hover)
- Circular trajectory
- Figure-8 trajectory
- Custom waypoints
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64MultiArray


class ReferenceTrajectoryNode(Node):
    """Generate reference trajectories for MPC."""
    
    def __init__(self):
        super().__init__('reference_trajectory_node')
        
        # Declare parameters
        self.declare_parameter('trajectory_type', 'setpoint')
        self.declare_parameter('setpoint_x', 0.0)
        self.declare_parameter('setpoint_y', 0.0)
        self.declare_parameter('setpoint_z', 1.0)
        self.declare_parameter('reference_rate', 50)
        self.declare_parameter('circular_radius', 1.0)
        self.declare_parameter('circular_height', 1.0)
        self.declare_parameter('circular_period', 10.0)
        
        # Get parameters
        self.trajectory_type = self.get_parameter('trajectory_type').value
        self.setpoint = np.array([
            self.get_parameter('setpoint_x').value,
            self.get_parameter('setpoint_y').value,
            self.get_parameter('setpoint_z').value
        ])
        self.reference_rate = self.get_parameter('reference_rate').value
        self.circular_radius = self.get_parameter('circular_radius').value
        self.circular_height = self.get_parameter('circular_height').value
        self.circular_period = self.get_parameter('circular_period').value
        
        # Time tracking
        self.current_time = 0.0
        self.dt = 1.0 / self.reference_rate
        
        # QoS profile
        qos_profile = QoSProfile(depth=10)
        
        # Publisher for reference trajectory
        self.ref_traj_pub = self.create_publisher(
            Float64MultiArray, '/reference_trajectory', qos_profile
        )
        
        # Timer for trajectory generation
        self.traj_timer = self.create_timer(
            self.dt, self.trajectory_callback
        )
        
        self.get_logger().info(
            f"Reference Trajectory Generator initialized\n"
            f"  Trajectory type: {self.trajectory_type}\n"
            f"  Rate: {self.reference_rate} Hz\n"
            f"  Setpoint: {self.setpoint}"
        )
    
    def trajectory_callback(self):
        """Generate and publish reference trajectory."""
        try:
            # Generate reference state (12D: pos, vel, attitude, rates)
            ref_state = np.zeros(12)
            
            if self.trajectory_type == 'setpoint':
                # Constant setpoint
                ref_state[:3] = self.setpoint
                ref_state[3:] = 0.0  # Zero velocity and attitude
            
            elif self.trajectory_type == 'circular':
                # Circular trajectory in XY plane at constant height
                omega = 2 * np.pi / self.circular_period
                angle = omega * self.current_time
                
                ref_state[0] = self.circular_radius * np.cos(angle)
                ref_state[1] = self.circular_radius * np.sin(angle)
                ref_state[2] = self.circular_height
                
                # Velocity (tangential)
                ref_state[3] = -self.circular_radius * omega * np.sin(angle)
                ref_state[4] = self.circular_radius * omega * np.cos(angle)
                ref_state[5] = 0.0
            
            elif self.trajectory_type == 'figure8':
                # Figure-8 trajectory
                omega = 2 * np.pi / self.circular_period
                angle = omega * self.current_time
                
                # Lissajous curve (figure-8)
                ref_state[0] = self.circular_radius * np.sin(angle)
                ref_state[1] = self.circular_radius * np.sin(angle) * np.cos(angle)
                ref_state[2] = self.circular_height
                
                # Velocity
                ref_state[3] = self.circular_radius * omega * np.cos(angle)
                ref_state[4] = self.circular_radius * omega * (np.cos(2*angle) - np.sin(angle)**2)
                ref_state[5] = 0.0
            
            elif self.trajectory_type == 'hover':
                # Simple hover at setpoint
                ref_state[:3] = self.setpoint
                ref_state[3:] = 0.0
            
            else:
                # Default: setpoint
                ref_state[:3] = self.setpoint
                ref_state[3:] = 0.0
            
            # Publish reference trajectory
            msg = Float64MultiArray()
            msg.data = ref_state.tolist()
            self.ref_traj_pub.publish(msg)
            
            # Update time
            self.current_time += self.dt
            
        except Exception as e:
            self.get_logger().error(f"Trajectory generation error: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = ReferenceTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
