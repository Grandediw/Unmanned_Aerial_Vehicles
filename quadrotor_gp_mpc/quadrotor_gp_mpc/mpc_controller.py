#!/usr/bin/env python3
"""
Model Predictive Control (MPC) for Quadrotor

This module implements a nonlinear MPC controller for quadrotor trajectory tracking.
It uses the quadrotor dynamics model and Gaussian Process uncertainty estimates
for robust control.
"""

import numpy as np
from typing import Tuple, List, Optional
from scipy.optimize import minimize
import cvxpy as cp
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Path


class QuadrotorMPC(Node):
    """
    Model Predictive Controller for quadrotor trajectory tracking.

    Solves the optimization problem:
    min sum_{k=0}^{N-1} [||x_k - x_ref||_Q + ||u_k||_R] + ||x_N - x_ref||_P
    s.t. x_{k+1} = f(x_k, u_k) + w_k
         u_min <= u_k <= u_max
         ||x_k - x_obstacles|| >= d_safe
    """

    def __init__(self):
        super().__init__('quadrotor_mpc')

        # MPC parameters
        self.N = 20  # Prediction horizon
        self.dt = 0.1  # Discretization time step

        # State and control dimensions
        self.nx = 12  # [x,y,z,vx,vy,vz,phi,theta,psi,p,q,r]
        self.nu = 4   # [T, tau_phi, tau_theta, tau_psi]

        # Physical parameters (should match dynamics model)
        self.mass = 0.5  # kg
        self.g = 9.81    # m/s^2
        self.Ixx = 0.0023
        self.Iyy = 0.0023
        self.Izz = 0.0046

        # Control constraints
        self.thrust_min = 0.0
        self.thrust_max = 2.0 * self.mass * self.g  # 2g max thrust
        self.torque_max = 0.1  # Maximum torque

        # Cost function weights (more conservative)
        self.Q = np.diag([100, 100, 100,  # Position weights (increased)
                         10, 10, 10,       # Velocity weights (increased)
                         50, 50, 50,       # Attitude weights (increased)
                         5, 5, 5])         # Angular rate weights (increased)

        self.R = np.diag([0.01, 0.1, 0.1, 0.1])  # Control weights (smaller thrust penalty)

        self.P = self.Q * 5  # Terminal cost weight (increased)

        # Current state and reference
        self.current_state = np.zeros(self.nx)
        self.reference_trajectory = np.zeros((self.N+1, self.nx))

        # GP uncertainty estimates
        self.gp_uncertainty = np.zeros(self.nx)
        self.use_gp = True

        # Obstacle avoidance
        self.obstacles = []  # List of [x, y, z, radius]
        self.safety_margin = 0.5  # meters

        # Solver settings
        self.solver_verbose = False
        self.max_iter = 100

        # ROS interfaces
        self.control_pub = self.create_publisher(
            Float64MultiArray,
            '/quadrotor/control',
            10
        )

        self.state_sub = self.create_subscription(
            Float64MultiArray,
            '/quadrotor/state',
            self.state_callback,
            10
        )

        self.reference_sub = self.create_subscription(
            Float64MultiArray,
            '/mpc/reference',
            self.reference_callback,
            10
        )

        self.gp_prediction_sub = self.create_subscription(
            Float64MultiArray,
            '/gp/prediction',
            self.gp_prediction_callback,
            10
        )

        self.trajectory_pub = self.create_publisher(
            Path,
            '/mpc/predicted_trajectory',
            10
        )

        # Control timer
        self.control_timer = self.create_timer(self.dt, self.control_loop)

        # Initialize with hover reference
        self.set_hover_reference(x=0, y=0, z=1.0)

        self.get_logger().info("Quadrotor MPC controller initialized")

    def dynamics_discrete(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """
        Discrete-time dynamics using RK4 integration.

        Args:
            x: Current state
            u: Control input

        Returns:
            Next state
        """
        # RK4 integration of continuous dynamics
        k1 = self.dynamics_continuous(x, u)
        k2 = self.dynamics_continuous(x + 0.5 * self.dt * k1, u)
        k3 = self.dynamics_continuous(x + 0.5 * self.dt * k2, u)
        k4 = self.dynamics_continuous(x + self.dt * k3, u)

        x_next = x + (self.dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

        # Add GP uncertainty if available
        if self.use_gp:
            x_next += self.dt * self.gp_uncertainty

        return x_next

    def dynamics_continuous(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """
        Continuous-time dynamics (same as in dynamics node).

        Args:
            x: State [x,y,z,vx,vy,vz,phi,theta,psi,p,q,r]
            u: Control [T, tau_phi, tau_theta, tau_psi]

        Returns:
            State derivative
        """
        # Extract state
        pos = x[0:3]
        vel = x[3:6]
        att = x[6:9]  # [phi, theta, psi]
        omega = x[9:12]  # [p, q, r]

        phi, theta, psi = att
        p, q, r = omega

        # Extract control
        T = u[0]
        tau = u[1:4]

        # Rotation matrix
        R = self.rotation_matrix(phi, theta, psi)

        # Position dynamics
        pos_dot = vel

        # Translational dynamics
        thrust_world = R @ np.array([0, 0, T])
        gravity = np.array([0, 0, -self.mass * self.g])

        # Simple drag model
        drag = -0.25 * np.linalg.norm(vel) * vel

        vel_dot = (thrust_world + gravity + drag) / self.mass

        # Attitude dynamics
        W = self.angular_velocity_transform(phi, theta)
        att_dot = W @ omega

        # Angular dynamics
        I = np.diag([self.Ixx, self.Iyy, self.Izz])
        omega_dot = np.linalg.inv(I) @ (tau - np.cross(omega, I @ omega) - 0.01 * omega)

        return np.concatenate([pos_dot, vel_dot, att_dot, omega_dot])

    def rotation_matrix(self, phi: float, theta: float, psi: float) -> np.ndarray:
        """Rotation matrix from body to world frame."""
        cphi, sphi = np.cos(phi), np.sin(phi)
        cth, sth = np.cos(theta), np.sin(theta)
        cpsi, spsi = np.cos(psi), np.sin(psi)

        return np.array([
            [cth * cpsi, sphi * sth * cpsi - cphi * spsi, cphi * sth * cpsi + sphi * spsi],
            [cth * spsi, sphi * sth * spsi + cphi * cpsi, cphi * sth * spsi - sphi * cpsi],
            [-sth,       sphi * cth,                      cphi * cth]
        ])

    def angular_velocity_transform(self, phi: float, theta: float) -> np.ndarray:
        """Transform from angular velocities to Euler angle rates."""
        cphi, sphi = np.cos(phi), np.sin(phi)
        cth, sth = np.cos(theta), np.sin(theta)

        if abs(cth) < 1e-6:
            cth = 1e-6 * np.sign(cth)

        return np.array([
            [1, sphi * np.tan(theta), cphi * np.tan(theta)],
            [0, cphi,                 -sphi],
            [0, sphi / cth,           cphi / cth]
        ])

    def solve_mpc(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Solve MPC optimization problem.

        Returns:
            Tuple of (optimal control sequence, predicted trajectory)
        """
        # Decision variables
        x = cp.Variable((self.N+1, self.nx))  # States
        u = cp.Variable((self.N, self.nu))    # Controls

        # Cost function
        cost = 0

        # Stage costs
        for k in range(self.N):
            state_error = x[k] - self.reference_trajectory[k]
            cost += cp.quad_form(state_error, self.Q)
            cost += cp.quad_form(u[k], self.R)

        # Terminal cost
        terminal_error = x[self.N] - self.reference_trajectory[self.N]
        cost += cp.quad_form(terminal_error, self.P)

        # Constraints
        constraints = []

        # Initial condition
        constraints.append(x[0] == self.current_state)

        # Dynamics constraints (linearized)
        for k in range(self.N):
            x_next_nominal = self.dynamics_discrete(
                self.reference_trajectory[k],
                self.get_nominal_control(k)
            )

            # First-order approximation around reference
            A, B = self.linearize_dynamics(
                self.reference_trajectory[k],
                self.get_nominal_control(k)
            )

            constraints.append(
                x[k+1] == x_next_nominal + A @ (x[k] - self.reference_trajectory[k]) +
                B @ (u[k] - self.get_nominal_control(k))
            )

        # Control constraints
        for k in range(self.N):
            constraints.append(u[k, 0] >= self.thrust_min)  # Thrust lower bound
            constraints.append(u[k, 0] <= self.thrust_max)  # Thrust upper bound
            constraints.append(cp.norm(u[k, 1:4], 'inf') <= self.torque_max)  # Torque bounds

        # State constraints (attitude limits)
        for k in range(self.N+1):
            constraints.append(cp.abs(x[k, 6]) <= np.pi/4)   # Roll limit
            constraints.append(cp.abs(x[k, 7]) <= np.pi/4)   # Pitch limit

        # Obstacle avoidance (if obstacles exist)
        for obs in self.obstacles:
            obs_pos = obs[:3]
            obs_radius = obs[3]
            for k in range(self.N+1):
                # Simplified spherical obstacle constraint
                constraints.append(
                    cp.norm(x[k, 0:3] - obs_pos, 2) >= obs_radius + self.safety_margin
                )

        # Solve optimization problem
        problem = cp.Problem(cp.Minimize(cost), constraints)

        # Try multiple solvers in order of preference
        solvers_to_try = [cp.ECOS, cp.OSQP, cp.SCS]
        
        for solver in solvers_to_try:
            try:
                problem.solve(solver=solver, verbose=self.solver_verbose, max_iters=self.max_iter)

                if problem.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
                    optimal_controls = u.value
                    predicted_trajectory = x.value
                    return optimal_controls, predicted_trajectory
                else:
                    self.get_logger().warning(f"MPC solver {solver} failed with status: {problem.status}")
                    continue  # Try next solver

            except Exception as e:
                self.get_logger().warning(f"MPC solver {solver} error: {e}. Trying next solver...")
                continue  # Try next solver

        # If all solvers failed
        self.get_logger().error("All MPC solvers failed. Using emergency control.")
        return self.get_emergency_control(), None

    def linearize_dynamics(self, x_ref: np.ndarray, u_ref: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Linearize dynamics around reference point.

        Returns:
            Tuple of (A matrix, B matrix)
        """
        # Numerical differentiation
        eps = 1e-8

        f_ref = self.dynamics_continuous(x_ref, u_ref)

        # A matrix (df/dx)
        A = np.zeros((self.nx, self.nx))
        for i in range(self.nx):
            x_pert = x_ref.copy()
            x_pert[i] += eps
            f_pert = self.dynamics_continuous(x_pert, u_ref)
            A[:, i] = (f_pert - f_ref) / eps

        # B matrix (df/du)
        B = np.zeros((self.nx, self.nu))
        for i in range(self.nu):
            u_pert = u_ref.copy()
            u_pert[i] += eps
            f_pert = self.dynamics_continuous(x_ref, u_pert)
            B[:, i] = (f_pert - f_ref) / eps

        # Convert to discrete time
        A_d = np.eye(self.nx) + self.dt * A
        B_d = self.dt * B

        return A_d, B_d

    def get_nominal_control(self, k: int) -> np.ndarray:
        """Get nominal control for reference trajectory."""
        # Approximate hover control for reference trajectory
        ref_state = self.reference_trajectory[k]

        # Hover thrust to counteract gravity
        T_hover = self.mass * self.g

        # Zero torques for reference (assuming reference is smooth)
        return np.array([T_hover, 0, 0, 0])

    def get_emergency_control(self) -> np.ndarray:
        """Get emergency control sequence (hover in place)."""
        emergency_controls = np.zeros((self.N, self.nu))
        for k in range(self.N):
            emergency_controls[k] = np.array([self.mass * self.g, 0, 0, 0])
        return emergency_controls

    def control_loop(self):
        """Main control loop called by timer."""
        # Solve MPC
        optimal_controls, predicted_trajectory = self.solve_mpc()

        # Apply first control input
        if optimal_controls is not None and len(optimal_controls) > 0:
            control_input = optimal_controls[0]
        else:
            # Fallback to hover
            control_input = np.array([self.mass * self.g, 0, 0, 0])

        # Publish control
        control_msg = Float64MultiArray()
        control_msg.data = control_input.tolist()
        self.control_pub.publish(control_msg)

        # Publish predicted trajectory for visualization
        if predicted_trajectory is not None:
            self.publish_predicted_trajectory(predicted_trajectory)

        # Shift reference trajectory
        self.shift_reference_trajectory()

    def publish_predicted_trajectory(self, trajectory: np.ndarray):
        """Publish predicted trajectory for visualization."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "world"

        for k in range(len(trajectory)):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = trajectory[k, 0]
            pose.pose.position.y = trajectory[k, 1]
            pose.pose.position.z = trajectory[k, 2]

            # Convert Euler angles to quaternion
            phi, theta, psi = trajectory[k, 6:9]
            qw, qx, qy, qz = self.euler_to_quaternion(phi, theta, psi)

            pose.pose.orientation.w = qw
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz

            path_msg.poses.append(pose)

        self.trajectory_pub.publish(path_msg)

    def euler_to_quaternion(self, phi: float, theta: float, psi: float) -> Tuple[float, float, float, float]:
        """Convert Euler angles to quaternion (w, x, y, z)."""
        cy = np.cos(psi * 0.5)
        sy = np.sin(psi * 0.5)
        cp = np.cos(theta * 0.5)
        sp = np.sin(theta * 0.5)
        cr = np.cos(phi * 0.5)
        sr = np.sin(phi * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return w, x, y, z

    def set_hover_reference(self, x: float = 0, y: float = 0, z: float = 1.0):
        """Set reference to hover at specified position."""
        hover_state = np.zeros(self.nx)
        hover_state[0] = x   # x position
        hover_state[1] = y   # y position
        hover_state[2] = z   # z position

        # Fill entire reference trajectory with hover state
        for k in range(self.N+1):
            self.reference_trajectory[k] = hover_state

    def shift_reference_trajectory(self):
        """Shift reference trajectory by one time step."""
        # Shift all states forward
        self.reference_trajectory[:-1] = self.reference_trajectory[1:]
        # Keep last reference the same (steady state)
        self.reference_trajectory[-1] = self.reference_trajectory[-2]

    def add_obstacle(self, x: float, y: float, z: float, radius: float):
        """Add spherical obstacle for avoidance."""
        self.obstacles.append([x, y, z, radius])
        self.get_logger().info(f"Added obstacle at ({x}, {y}, {z}) with radius {radius}")

    def state_callback(self, msg: Float64MultiArray):
        """Callback for current state."""
        if len(msg.data) >= self.nx:
            self.current_state = np.array(msg.data[:self.nx])

    def reference_callback(self, msg: Float64MultiArray):
        """Callback for reference trajectory."""
        data = np.array(msg.data)
        if len(data) >= self.nx:
            # Single reference point - extend to full horizon
            ref_state = data[:self.nx]
            for k in range(self.N+1):
                self.reference_trajectory[k] = ref_state

    def gp_prediction_callback(self, msg: Float64MultiArray):
        """Callback for GP uncertainty predictions."""
        if len(msg.data) >= self.nx:
            self.gp_uncertainty = np.array(msg.data[:self.nx])


def main(args=None):
    rclpy.init(args=args)

    mpc_node = QuadrotorMPC()

    # Add some example obstacles
    # mpc_node.add_obstacle(2.0, 2.0, 1.0, 0.5)

    try:
        rclpy.spin(mpc_node)
    except KeyboardInterrupt:
        pass
    finally:
        mpc_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
