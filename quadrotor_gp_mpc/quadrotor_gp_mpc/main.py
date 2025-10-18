#!/usr/bin/env python3
"""
Main launch script for the quadrotor GP-MPC system.

This script coordinates the launch of all components:
- Quadrotor dynamics simulation
- Gaussian Process learning
- MPC controller
- Data collection and training pipeline
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import time
import threading
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped

# Import our modules
from .quadrotor_dynamics import QuadrotorDynamics
from .gaussian_process import GaussianProcess
from .mpc_controller import QuadrotorMPC


class DataCollector:
    """Collects training data for the Gaussian Process."""

    def __init__(self, dynamics_node, gp_node):
        self.dynamics_node = dynamics_node
        self.gp_node = gp_node
        self.collecting = False

        # Storage for true dynamics vs model prediction
        self.prev_state = None
        self.prev_control = None
        self.collection_timer = None

    def start_collection(self):
        """Start collecting training data."""
        self.collecting = True
        print("Started GP training data collection")

    def stop_collection(self):
        """Stop collecting training data."""
        self.collecting = False
        print("Stopped GP training data collection")

    def collect_data_point(self):
        """Collect a single training data point."""
        if not self.collecting or self.prev_state is None:
            return

        current_state = self.dynamics_node.get_state()

        # Compute true state change
        true_state_change = current_state - self.prev_state

        # Compute nominal model prediction
        nominal_state_change = self.dynamics_node.dynamics(self.prev_state, self.prev_control) * self.dynamics_node.dt

        # GP learns the residual: true - nominal
        residual = true_state_change - nominal_state_change

        # Training input: [state, control]
        X_train = np.concatenate([self.prev_state, self.prev_control])

        # Training output: residual
        Y_train = residual

        # Add to GP training data
        self.gp_node.add_training_data(X_train.reshape(1, -1), Y_train.reshape(1, -1))

        print(f"Collected training point. Total data: {len(self.gp_node.X_train)}")

    def update(self, state, control):
        """Update with current state and control."""
        if self.prev_state is not None:
            self.collect_data_point()

        self.prev_state = state.copy()
        self.prev_control = control.copy()


class TrajectoryGenerator:
    """Generates reference trajectories for the MPC controller."""

    def __init__(self, mpc_node):
        self.mpc_node = mpc_node
        self.trajectory_type = "hover"  # "hover", "circle", "figure8", "waypoints"
        self.time = 0.0

        # Trajectory parameters
        self.circle_radius = 2.0
        self.circle_height = 1.5
        self.circle_period = 10.0  # seconds

        self.figure8_radius = 1.5
        self.figure8_height = 1.5
        self.figure8_period = 15.0

        self.waypoints = [
            [0, 0, 1.0],
            [2, 0, 1.5],
            [2, 2, 2.0],
            [0, 2, 1.5],
            [0, 0, 1.0]
        ]
        self.waypoint_index = 0
        self.waypoint_hold_time = 5.0  # seconds at each waypoint

    def set_trajectory_type(self, traj_type: str):
        """Set the type of reference trajectory."""
        self.trajectory_type = traj_type
        self.time = 0.0
        print(f"Set trajectory type to: {traj_type}")

    def get_reference_state(self) -> np.ndarray:
        """Get current reference state."""
        ref_state = np.zeros(12)

        if self.trajectory_type == "hover":
            ref_state[0:3] = [0, 0, 1.0]  # Hover at origin, 1m height

        elif self.trajectory_type == "circle":
            omega = 2 * np.pi / self.circle_period
            ref_state[0] = self.circle_radius * np.cos(omega * self.time)
            ref_state[1] = self.circle_radius * np.sin(omega * self.time)
            ref_state[2] = self.circle_height

            # Reference velocities
            ref_state[3] = -self.circle_radius * omega * np.sin(omega * self.time)
            ref_state[4] = self.circle_radius * omega * np.cos(omega * self.time)
            ref_state[5] = 0

        elif self.trajectory_type == "figure8":
            omega = 2 * np.pi / self.figure8_period
            ref_state[0] = self.figure8_radius * np.sin(omega * self.time)
            ref_state[1] = self.figure8_radius * np.sin(2 * omega * self.time)
            ref_state[2] = self.figure8_height

            # Reference velocities
            ref_state[3] = self.figure8_radius * omega * np.cos(omega * self.time)
            ref_state[4] = 2 * self.figure8_radius * omega * np.cos(2 * omega * self.time)
            ref_state[5] = 0

        elif self.trajectory_type == "waypoints":
            waypoint_time = self.time % (len(self.waypoints) * self.waypoint_hold_time)
            waypoint_idx = int(waypoint_time // self.waypoint_hold_time)
            waypoint_idx = min(waypoint_idx, len(self.waypoints) - 1)

            ref_state[0:3] = self.waypoints[waypoint_idx]

        return ref_state

    def update(self, dt: float):
        """Update trajectory time."""
        self.time += dt


class QuadrotorGPMPCSystem:
    """Main system coordinator."""

    def __init__(self):
        # Initialize ROS
        rclpy.init()

        # Create nodes
        self.dynamics_node = QuadrotorDynamics()
        self.gp_node = GaussianProcess()
        self.mpc_node = QuadrotorMPC()

        # Create subsystems
        self.data_collector = DataCollector(self.dynamics_node, self.gp_node)
        self.traj_generator = TrajectoryGenerator(self.mpc_node)

        # System state
        self.running = False
        self.dt = 0.1  # Main loop rate

        # Performance metrics
        self.tracking_error_history = []
        self.control_effort_history = []

        print("Quadrotor GP-MPC System initialized")

    def start(self):
        """Start the complete system."""
        self.running = True

        # Initialize quadrotor at hover
        initial_state = np.zeros(12)
        initial_state[2] = 1.0  # 1m altitude
        self.dynamics_node.set_state(initial_state)

        # Set initial reference
        self.traj_generator.set_trajectory_type("hover")

        # Start data collection
        self.data_collector.start_collection()

        print("System started")

    def stop(self):
        """Stop the system."""
        self.running = False
        self.data_collector.stop_collection()
        print("System stopped")

    def run_simulation(self, duration: float = 60.0):
        """Run simulation for specified duration."""

        # Use MultiThreadedExecutor for concurrent node execution
        executor = MultiThreadedExecutor()
        executor.add_node(self.dynamics_node)
        executor.add_node(self.gp_node)
        executor.add_node(self.mpc_node)

        # Start executor in separate thread
        executor_thread = threading.Thread(target=executor.spin)
        executor_thread.daemon = True
        executor_thread.start()

        start_time = time.time()

        try:
            while self.running and (time.time() - start_time) < duration:
                # Update trajectory reference
                ref_state = self.traj_generator.get_reference_state()
                self.update_mpc_reference(ref_state)

                # Update data collection
                current_state = self.dynamics_node.get_state()
                current_control = self.dynamics_node.control
                self.data_collector.update(current_state, current_control)

                # Update trajectory generator
                self.traj_generator.update(self.dt)

                # Compute performance metrics
                self.compute_metrics(current_state, ref_state, current_control)

                # Print status
                if int(time.time() - start_time) % 5 == 0:  # Every 5 seconds
                    self.print_status()

                time.sleep(self.dt)

        except KeyboardInterrupt:
            print("\nShutdown requested by user")

        finally:
            self.stop()
            executor.shutdown()

    def update_mpc_reference(self, ref_state: np.ndarray):
        """Update MPC with new reference state."""
        # Create and publish reference message
        ref_msg = Float64MultiArray()
        ref_msg.data = ref_state.tolist()

        # Update MPC reference directly
        for k in range(self.mpc_node.N + 1):
            self.mpc_node.reference_trajectory[k] = ref_state

    def compute_metrics(self, state: np.ndarray, ref_state: np.ndarray, control: np.ndarray):
        """Compute and store performance metrics."""
        # Tracking error (position only)
        position_error = np.linalg.norm(state[0:3] - ref_state[0:3])
        self.tracking_error_history.append(position_error)

        # Control effort
        control_effort = np.linalg.norm(control)
        self.control_effort_history.append(control_effort)

        # Keep only last 1000 points
        if len(self.tracking_error_history) > 1000:
            self.tracking_error_history = self.tracking_error_history[-1000:]
            self.control_effort_history = self.control_effort_history[-1000:]

    def print_status(self):
        """Print system status."""
        state = self.dynamics_node.get_state()
        ref_state = self.traj_generator.get_reference_state()

        pos_error = np.linalg.norm(state[0:3] - ref_state[0:3])
        avg_error = np.mean(self.tracking_error_history[-50:]) if self.tracking_error_history else 0

        print(f"Position: [{state[0]:.2f}, {state[1]:.2f}, {state[2]:.2f}] | "
              f"Error: {pos_error:.3f}m | Avg Error: {avg_error:.3f}m | "
              f"GP Data: {len(self.gp_node.X_train)} points")

    def change_trajectory(self, traj_type: str):
        """Change trajectory type during simulation."""
        self.traj_generator.set_trajectory_type(traj_type)

    def save_gp_model(self, filename: str = "quadrotor_gp_model.npz"):
        """Save trained GP model."""
        self.gp_node.save_model(filename)

    def load_gp_model(self, filename: str = "quadrotor_gp_model.npz"):
        """Load pre-trained GP model."""
        self.gp_node.load_model(filename)


def main():
    """Main function for launching the complete system."""

    # Create system
    system = QuadrotorGPMPCSystem()

    # Start system
    system.start()

    print("\n=== Quadrotor GP-MPC Simulation ===")
    print("Starting with hover trajectory...")

    # Run initial hover phase
    print("Phase 1: Hover (20s) - Collecting initial data")
    system.run_simulation(duration=20.0)

    if not system.running:
        return

    # Switch to circular trajectory
    print("Phase 2: Circular trajectory (30s)")
    system.change_trajectory("circle")
    system.run_simulation(duration=30.0)

    if not system.running:
        return

    # Switch to figure-8 trajectory
    print("Phase 3: Figure-8 trajectory (30s)")
    system.change_trajectory("figure8")
    system.run_simulation(duration=30.0)

    if not system.running:
        return

    # Save trained model
    print("Saving trained GP model...")
    system.save_gp_model()

    print("\nSimulation completed successfully!")

    # Print final statistics
    if system.tracking_error_history:
        avg_error = np.mean(system.tracking_error_history)
        max_error = np.max(system.tracking_error_history)
        print(f"Average tracking error: {avg_error:.3f}m")
        print(f"Maximum tracking error: {max_error:.3f}m")
        print(f"GP training data points: {len(system.gp_node.X_train)}")

    # Cleanup
    rclpy.shutdown()


if __name__ == '__main__':
    main()
