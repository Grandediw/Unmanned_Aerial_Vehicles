#!/usr/bin/env python3
"""
Quadrotor Dynamics Model with Gaussian Process

This module implements the nonlinear dynamics of a quadrotor with
aerodynamic effects and includes a Gaussian Process for learning
model uncertainties.
"""

import numpy as np
from typing import Tuple, Optional
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped, TransformStamped


class QuadrotorDynamics(Node):
    """
    Quadrotor dynamics model with aerodynamic effects and GP learning.

    State vector: [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
    - Position: (x, y, z)
    - Velocity: (vx, vy, vz)
    - Euler angles: (phi, theta, psi) - roll, pitch, yaw
    - Angular rates: (p, q, r)

    Control inputs: [T, tau_phi, tau_theta, tau_psi]
    - T: Total thrust
    - tau_phi, tau_theta, tau_psi: Torques about body axes
    """

    def __init__(self):
        super().__init__('quadrotor_dynamics')

        # Physical parameters
        self.mass = 0.5  # kg
        self.g = 9.81    # m/s^2
        self.Ixx = 0.0023  # kg*m^2
        self.Iyy = 0.0023  # kg*m^2
        self.Izz = 0.0046  # kg*m^2

        # Aerodynamic drag coefficients
        self.k_drag_linear = 0.25   # Linear drag coefficient
        self.k_drag_angular = 0.01  # Angular drag coefficient

        # State and control dimensions
        self.state_dim = 12  # [x,y,z,vx,vy,vz,phi,theta,psi,p,q,r]
        self.control_dim = 4 # [T, tau_phi, tau_theta, tau_psi]

        # Current state
        self.state = np.zeros(12)
        # Default to hover thrust to prevent immediate falling
        self.control = np.array([self.mass * self.g, 0, 0, 0])  # Hover thrust

        # Gaussian Process for model uncertainty
        self.gp_enabled = True
        self.gp_uncertainty = np.zeros(12)  # GP predictions for each state derivative

        # ROS Publishers and Subscribers
        self.state_pub = self.create_publisher(
            Float64MultiArray,
            '/quadrotor/state',
            10
        )

        self.control_sub = self.create_subscription(
            Float64MultiArray,
            '/quadrotor/control',
            self.control_callback,
            10
        )

        self.imu_pub = self.create_publisher(
            Imu,
            '/quadrotor/imu',
            10
        )

        # Timer for dynamics integration
        self.dt = 0.01  # 100 Hz
        self.timer = self.create_timer(self.dt, self.integrate_dynamics)

        # TF broadcaster for visualization
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("Quadrotor Dynamics node initialized")

    def control_callback(self, msg: Float64MultiArray):
        """Callback for control input."""
        if len(msg.data) >= self.control_dim:
            self.control = np.array(msg.data[:self.control_dim])

    def rotation_matrix(self, phi: float, theta: float, psi: float) -> np.ndarray:
        """
        Compute rotation matrix from body frame to world frame.

        Args:
            phi: Roll angle (rad)
            theta: Pitch angle (rad)
            psi: Yaw angle (rad)

        Returns:
            3x3 rotation matrix
        """
        cphi, sphi = np.cos(phi), np.sin(phi)
        cth, sth = np.cos(theta), np.sin(theta)
        cpsi, spsi = np.cos(psi), np.sin(psi)

        R = np.array([
            [cth * cpsi, sphi * sth * cpsi - cphi * spsi, cphi * sth * cpsi + sphi * spsi],
            [cth * spsi, sphi * sth * spsi + cphi * cpsi, cphi * sth * spsi - sphi * cpsi],
            [-sth,       sphi * cth,                      cphi * cth]
        ])

        return R

    def angular_velocity_transform(self, phi: float, theta: float) -> np.ndarray:
        """
        Transform from angular velocities to Euler angle rates.

        Args:
            phi: Roll angle (rad)
            theta: Pitch angle (rad)

        Returns:
            3x3 transformation matrix
        """
        cphi, sphi = np.cos(phi), np.sin(phi)
        cth, sth = np.cos(theta), np.sin(theta)

        # Avoid singularity at theta = ±π/2
        if abs(cth) < 1e-6:
            cth = 1e-6 * np.sign(cth)

        W = np.array([
            [1, sphi * np.tan(theta), cphi * np.tan(theta)],
            [0, cphi,                 -sphi],
            [0, sphi / cth,           cphi / cth]
        ])

        return W

    def dynamics(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        """
        Compute state derivatives using quadrotor dynamics.

        Args:
            state: Current state [x,y,z,vx,vy,vz,phi,theta,psi,p,q,r]
            control: Control input [T, tau_phi, tau_theta, tau_psi]

        Returns:
            State derivatives
        """
        # Extract state variables
        x, y, z = state[0:3]
        vx, vy, vz = state[3:6]
        phi, theta, psi = state[6:9]
        p, q, r = state[9:12]

        # Extract control inputs
        T = control[0]  # Total thrust
        tau_phi, tau_theta, tau_psi = control[1:4]

        # Position derivatives (velocities)
        pos_dot = state[3:6]

        # Rotation matrix from body to world frame
        R = self.rotation_matrix(phi, theta, psi)

        # Forces in world frame
        thrust_world = R @ np.array([0, 0, T])
        gravity = np.array([0, 0, -self.mass * self.g])

        # Aerodynamic drag (opposing velocity)
        velocity = np.array([vx, vy, vz])
        drag_force = -self.k_drag_linear * np.linalg.norm(velocity) * velocity

        # Acceleration
        acceleration = (thrust_world + gravity + drag_force) / self.mass

        # Angular velocity transformation
        W = self.angular_velocity_transform(phi, theta)
        attitude_dot = W @ np.array([p, q, r])

        # Angular acceleration (Euler's equations)
        angular_velocity = np.array([p, q, r])
        torques = np.array([tau_phi, tau_theta, tau_psi])

        # Gyroscopic effects and angular drag
        I = np.diag([self.Ixx, self.Iyy, self.Izz])
        gyroscopic = np.cross(angular_velocity, I @ angular_velocity)
        angular_drag = -self.k_drag_angular * angular_velocity

        angular_acceleration = np.linalg.inv(I) @ (torques - gyroscopic + angular_drag)

        # Combine all derivatives
        state_dot = np.concatenate([
            pos_dot,           # [x_dot, y_dot, z_dot]
            acceleration,      # [vx_dot, vy_dot, vz_dot]
            attitude_dot,      # [phi_dot, theta_dot, psi_dot]
            angular_acceleration  # [p_dot, q_dot, r_dot]
        ])

        # Add Gaussian Process uncertainty if enabled
        if self.gp_enabled:
            state_dot += self.gp_uncertainty

        return state_dot

    def integrate_dynamics(self):
        """Integrate dynamics using RK4 method."""
        # RK4 integration
        k1 = self.dynamics(self.state, self.control)
        k2 = self.dynamics(self.state + 0.5 * self.dt * k1, self.control)
        k3 = self.dynamics(self.state + 0.5 * self.dt * k2, self.control)
        k4 = self.dynamics(self.state + self.dt * k3, self.control)

        self.state += (self.dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

        # Publish state
        self.publish_state()
        self.publish_imu()
        self.publish_transform()

    def publish_state(self):
        """Publish current state."""
        msg = Float64MultiArray()
        msg.data = self.state.tolist()
        self.state_pub.publish(msg)

    def publish_imu(self):
        """Publish IMU data."""
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "base_link"

        # Extract relevant state
        phi, theta, psi = self.state[6:9]
        p, q, r = self.state[9:12]

        # Convert Euler angles to quaternion
        cy = np.cos(psi * 0.5)
        sy = np.sin(psi * 0.5)
        cp = np.cos(theta * 0.5)
        sp = np.sin(theta * 0.5)
        cr = np.cos(phi * 0.5)
        sr = np.sin(phi * 0.5)

        imu_msg.orientation.w = cr * cp * cy + sr * sp * sy
        imu_msg.orientation.x = sr * cp * cy - cr * sp * sy
        imu_msg.orientation.y = cr * sp * cy + sr * cp * sy
        imu_msg.orientation.z = cr * cp * sy - sr * sp * cy

        # Angular velocity
        imu_msg.angular_velocity.x = p
        imu_msg.angular_velocity.y = q
        imu_msg.angular_velocity.z = r

        # Linear acceleration (in body frame)
        R = self.rotation_matrix(phi, theta, psi)
        acceleration_world = self.dynamics(self.state, self.control)[3:6]
        acceleration_body = R.T @ (acceleration_world + np.array([0, 0, self.g]))

        imu_msg.linear_acceleration.x = acceleration_body[0]
        imu_msg.linear_acceleration.y = acceleration_body[1]
        imu_msg.linear_acceleration.z = acceleration_body[2]

        self.imu_pub.publish(imu_msg)

    def publish_transform(self):
        """Publish TF transform for visualization."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"

        # Position
        t.transform.translation.x = self.state[0]
        t.transform.translation.y = self.state[1]
        t.transform.translation.z = self.state[2]

        # Orientation (Euler to quaternion)
        phi, theta, psi = self.state[6:9]

        cy = np.cos(psi * 0.5)
        sy = np.sin(psi * 0.5)
        cp = np.cos(theta * 0.5)
        sp = np.sin(theta * 0.5)
        cr = np.cos(phi * 0.5)
        sr = np.sin(phi * 0.5)

        t.transform.rotation.w = cr * cp * cy + sr * sp * sy
        t.transform.rotation.x = sr * cp * cy - cr * sp * sy
        t.transform.rotation.y = cr * sp * cy + sr * cp * sy
        t.transform.rotation.z = cr * cp * sy - sr * sp * cy

        self.tf_broadcaster.sendTransform(t)

    def update_gp_uncertainty(self, uncertainty: np.ndarray):
        """Update Gaussian Process uncertainty estimates."""
        if len(uncertainty) == self.state_dim:
            self.gp_uncertainty = uncertainty

    def set_state(self, new_state: np.ndarray):
        """Set the current state (for initialization or reset)."""
        if len(new_state) == self.state_dim:
            self.state = new_state.copy()

    def get_state(self) -> np.ndarray:
        """Get current state."""
        return self.state.copy()


def main(args=None):
    rclpy.init(args=args)

    dynamics_node = QuadrotorDynamics()

    # Initialize at hover condition
    initial_state = np.zeros(12)
    initial_state[2] = 1.0  # 1m altitude
    dynamics_node.set_state(initial_state)

    try:
        rclpy.spin(dynamics_node)
    except KeyboardInterrupt:
        pass
    finally:
        dynamics_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
