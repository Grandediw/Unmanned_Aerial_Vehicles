#!/usr/bin/env python3
"""
Simple PID Controller for Quadrotor

A basic PID controller as a fallback option for testing the system integration.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Path


class SimpleQuadrotorController(Node):
    """
    Simple PID controller for quadrotor trajectory tracking.
    """

    def __init__(self):
        super().__init__('simple_quadrotor_controller')

        # PID gains (conservative values)
        self.kp_pos = 2.0   # Reduced from 5.0
        self.ki_pos = 0.01  # Reduced from 0.1
        self.kd_pos = 1.0   # Reduced from 2.0
        
        self.kp_att = 5.0   # Reduced from 10.0
        self.ki_att = 0.01  # Reduced from 0.1
        self.kd_att = 2.0   # Reduced from 5.0

        # Physical parameters
        self.mass = 0.5  # kg
        self.g = 9.81    # m/s^2

        # State variables
        self.current_state = np.zeros(12)
        self.desired_state = np.array([0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0])  # Hover at 1m
        
        # Integral error accumulators
        self.pos_error_integral = np.zeros(3)
        self.att_error_integral = np.zeros(3)
        
        # Previous errors for derivative
        self.prev_pos_error = np.zeros(3)
        self.prev_att_error = np.zeros(3)

        # Control limits
        self.thrust_min = 0.0
        self.thrust_max = 2.0 * self.mass * self.g
        self.torque_max = 0.5

        # Publishers
        self.control_pub = self.create_publisher(
            Float64MultiArray, 
            '/quadrotor/control', 
            10
        )

        # Subscribers
        self.state_sub = self.create_subscription(
            Float64MultiArray,
            '/quadrotor/state',
            self.state_callback,
            10
        )

        self.trajectory_sub = self.create_subscription(
            Path,
            'trajectory',
            self.trajectory_callback,
            10
        )

        # Control timer (100 Hz)
        self.dt = 0.01
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        
        # Debug counter
        self.debug_counter = 0

        self.get_logger().info("Simple Quadrotor Controller initialized")

    def state_callback(self, msg):
        """Update current state."""
        self.current_state = np.array(msg.data)

    def trajectory_callback(self, msg):
        """Update desired trajectory."""
        if len(msg.poses) > 0:
            pose = msg.poses[0].pose
            self.desired_state[0] = pose.position.x
            self.desired_state[1] = pose.position.y
            self.desired_state[2] = pose.position.z
            # Keep velocities and higher derivatives at zero for now

    def control_loop(self):
        """Main control loop."""
        # Extract current state
        pos = self.current_state[0:3]
        vel = self.current_state[3:6]
        att = self.current_state[6:9]  # [phi, theta, psi]
        omega = self.current_state[9:12]

        # Extract desired state
        pos_des = self.desired_state[0:3]
        vel_des = self.desired_state[3:6]
        
        # Position control
        pos_error = pos_des - pos
        self.pos_error_integral += pos_error * self.dt
        pos_error_derivative = (pos_error - self.prev_pos_error) / self.dt
        
        # PID position control
        acc_des = (self.kp_pos * pos_error + 
                  self.ki_pos * self.pos_error_integral + 
                  self.kd_pos * pos_error_derivative)
        
        # Desired thrust and attitude
        thrust_vec = acc_des + np.array([0, 0, self.g])
        thrust = np.linalg.norm(thrust_vec)
        
        # Desired attitude from thrust vector
        if thrust > 0.1:
            z_body_des = thrust_vec / thrust
            
            # Compute desired roll and pitch
            phi_des = np.arcsin(-z_body_des[1])
            theta_des = np.arctan2(z_body_des[0], z_body_des[2])
            psi_des = 0.0  # Keep yaw at zero for simplicity
            
            att_des = np.array([phi_des, theta_des, psi_des])
        else:
            att_des = np.array([0, 0, 0])
            thrust = self.mass * self.g
        
        # Attitude control
        att_error = att_des - att
        self.att_error_integral += att_error * self.dt
        att_error_derivative = (att_error - self.prev_att_error) / self.dt
        
        # PID attitude control
        torque = (self.kp_att * att_error + 
                 self.ki_att * self.att_error_integral + 
                 self.kd_att * att_error_derivative)
        
        # Limit controls
        thrust = np.clip(thrust * self.mass, self.thrust_min, self.thrust_max)
        torque = np.clip(torque, -self.torque_max, self.torque_max)
        
        # Publish control
        control_msg = Float64MultiArray()
        control_msg.data = [float(thrust), float(torque[0]), float(torque[1]), float(torque[2])]
        self.control_pub.publish(control_msg)
        
        # Debug output every 100 iterations (1 second)
        self.debug_counter += 1
        if self.debug_counter % 100 == 0:
            self.get_logger().info(f"Pos: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}] | "
                                 f"Error: [{pos_error[0]:.2f}, {pos_error[1]:.2f}, {pos_error[2]:.2f}] | "
                                 f"Thrust: {thrust:.2f}")
        
        # Update previous errors
        self.prev_pos_error = pos_error
        self.prev_att_error = att_error


def main(args=None):
    rclpy.init(args=args)
    controller = SimpleQuadrotorController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()