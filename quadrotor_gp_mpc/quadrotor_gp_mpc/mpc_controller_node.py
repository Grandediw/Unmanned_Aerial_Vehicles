#!/usr/bin/env python3
"""
MPC Controller Node for Gazebo x500 quadrotor.

Subscribes to:
- /state: Current quadrotor state
- /reference_trajectory: Target trajectory

Publishes to:
- /control_input: Thrust and torque commands
- /mpc_metrics: Performance metrics
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Imu
from tf2_ros import TransformListener, Buffer

from quadrotor_gp_mpc.mpc_controller import QuadrotorMPC
from quadrotor_gp_mpc.performance_metrics import MPCMetrics


class MPCControllerNode(Node):
    """MPC Controller Node for quadrotor control in Gazebo."""
    
    def __init__(self):
        super().__init__('mpc_controller_node')
        
        # Declare parameters
        self.declare_parameter('control_rate', 100)
        self.declare_parameter('enabled', True)
        
        # Get parameters
        self.control_rate = self.get_parameter('control_rate').value
        self.enabled = self.get_parameter('enabled').value
        
        # Initialize MPC controller (uses default parameters from class)
        self.mpc = QuadrotorMPC()
        
        # State tracking
        self.current_state = np.zeros(12)
        self.reference_state = np.zeros(12)
        self.control_input = np.array([0.0, 0.0, 0.0, 0.0])  # [thrust, tau_p, tau_q, tau_r]
        
        # Metrics tracking
        self.metrics = MPCMetrics()
        self.step_count = 0
        
        # TF2 for state estimation
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # QoS profile for real-time communication
        qos_profile = QoSProfile(depth=10)
        
        # Publishers
        self.control_pub = self.create_publisher(
            Float64MultiArray, '/gz_x500/control_input', qos_profile
        )
        self.metrics_pub = self.create_publisher(
            Float64MultiArray, '/mpc/metrics', qos_profile
        )
        
        # Subscribers
        self.state_sub = self.create_subscription(
            Float64MultiArray, '/gz_x500/state', self.state_callback, qos_profile
        )
        self.ref_traj_sub = self.create_subscription(
            Float64MultiArray, '/reference_trajectory', self.reference_callback, qos_profile
        )
        self.imu_sub = self.create_subscription(
            Imu, '/gz_x500/imu', self.imu_callback, qos_profile
        )
        
        # Timer for control loop
        self.control_timer = self.create_timer(
            1.0 / self.control_rate, self.control_loop_callback
        )
        
        # Metrics timer
        self.metrics_timer = self.create_timer(1.0, self.publish_metrics_callback)
        
        self.get_logger().info(
            f"MPC Controller initialized\n"
            f"  Control rate: {self.control_rate} Hz\n"
            f"  Enabled: {self.enabled}"
        )
    
    def state_callback(self, msg):
        """Update current state from Gazebo."""
        if len(msg.data) >= 12:
            self.current_state = np.array(msg.data[:12])
        else:
            self.get_logger().warn(f"Invalid state message length: {len(msg.data)}")
    
    def reference_callback(self, msg):
        """Update reference trajectory."""
        if len(msg.data) >= 12:
            self.reference_state = np.array(msg.data[:12])
        elif len(msg.data) >= 3:
            # If only position is provided, keep zero velocity/attitude
            self.reference_state[:3] = msg.data[:3]
            self.reference_state[3:] = 0.0
    
    def imu_callback(self, msg):
        """Update IMU data if needed."""
        # Can be used for additional state estimation
        pass
    
    def control_loop_callback(self):
        """Main control loop - runs at control_rate Hz."""
        if not self.enabled:
            return
        
        try:
            # Compute MPC control
            # For simplified version, use proportional-derivative control
            error = self.current_state - self.reference_state
            
            # Simple PD control (can be replaced with full MPC)
            kp = 10.0
            kd = 5.0
            
            # Position error to acceleration
            accel_cmd = -kp * error[:3] - kd * error[3:6]
            
            # Gravity compensation for Z
            accel_cmd[2] += 9.81
            
            # Limit acceleration
            max_accel = 15.0
            accel_cmd = np.clip(accel_cmd, -max_accel, max_accel)
            
            # Convert acceleration to thrust and torques
            # Simplified model: thrust is total vertical acceleration
            mass = 0.5  # kg
            max_thrust = 2.0  # N
            thrust = mass * accel_cmd[2]
            thrust = np.clip(thrust, 0.0, max_thrust)
            
            # Attitude errors
            phi_cmd = np.clip(kp * accel_cmd[1], -0.3, 0.3)
            theta_cmd = np.clip(-kp * accel_cmd[0], -0.3, 0.3)
            psi_cmd = 0.0
            
            # Torque commands (attitude control)
            tau_p = -5.0 * (self.current_state[6] - phi_cmd) - 2.0 * self.current_state[9]
            tau_q = -5.0 * (self.current_state[7] - theta_cmd) - 2.0 * self.current_state[10]
            tau_r = -2.0 * self.current_state[11]
            
            # Saturate torques
            max_torque = 0.1  # N*m
            tau_p = np.clip(tau_p, -max_torque, max_torque)
            tau_q = np.clip(tau_q, -max_torque, max_torque)
            tau_r = np.clip(tau_r, -max_torque, max_torque)
            
            self.control_input = np.array([thrust, tau_p, tau_q, tau_r])
            
            # Publish control input
            self.publish_control()
            
            # Update metrics
            self.step_count += 1
            
        except Exception as e:
            self.get_logger().error(f"Control loop error: {str(e)}")
    
    def publish_control(self):
        """Publish control input to Gazebo."""
        msg = Float64MultiArray()
        msg.data = self.control_input.tolist()
        self.control_pub.publish(msg)
    
    def publish_metrics_callback(self):
        """Publish MPC metrics periodically."""
        try:
            error = self.current_state[:3] - self.reference_state[:3]
            pos_error = np.linalg.norm(error)
            
            msg = Float64MultiArray()
            msg.data = [
                pos_error,
                np.linalg.norm(self.current_state[3:6]),
                self.control_input[0],  # thrust
                float(self.step_count)
            ]
            self.metrics_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Metrics publication error: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = MPCControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
