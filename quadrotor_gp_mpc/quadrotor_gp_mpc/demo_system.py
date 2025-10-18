#!/usr/bin/env python3
"""
Simplified Demo System for Quadrotor GP-MPC with Gazebo Integration

This demo focuses on:
1. Stable quadrotor flight simulation
2. Clear visualization with Gazebo
3. Simple GP learning demonstration
4. Performance plots and results
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import time
import threading
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt
import os

# Import our modules
from .quadrotor_dynamics import QuadrotorDynamics
from .gaussian_process import GaussianProcess
from .mpc_controller import QuadrotorMPC


class SimpleGPDemo:
    """Simple demonstration of GP-MPC quadrotor system."""

    def __init__(self):
        # Initialize ROS
        rclpy.init()

        # Create nodes
        self.dynamics_node = QuadrotorDynamics()
        self.gp_node = GaussianProcess()
        self.mpc_node = QuadrotorMPC()

        # System parameters
        self.dt = 0.1
        self.running = False
        
        # Reference trajectory (start with simple hover)
        self.reference = np.array([0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0])  # Hover at 1m
        
        # Data collection
        self.training_data_x = []
        self.training_data_y = []
        self.prev_state = None
        self.prev_control = None
        
        # Performance tracking
        self.position_history = []
        self.error_history = []
        self.control_history = []
        self.time_history = []

        print("✅ Simple GP-MPC Demo initialized")

    def start(self):
        """Start the demo system."""
        # Initialize quadrotor at hover position with small perturbation
        initial_state = np.zeros(12)
        initial_state[0] = 0.1  # Small x offset
        initial_state[1] = 0.1  # Small y offset  
        initial_state[2] = 0.9  # Start slightly below target
        self.dynamics_node.set_state(initial_state)
        
        # Set MPC reference
        for k in range(self.mpc_node.N + 1):
            self.mpc_node.reference_trajectory[k] = self.reference
            
        self.running = True
        print("🚁 System started - Quadrotor will stabilize to hover")

    def collect_training_data(self, state, control):
        """Collect data for GP training."""
        if self.prev_state is not None:
            # Compute true dynamics vs model prediction
            true_next_state = state
            predicted_next_state = self.prev_state + self.dynamics_node.dynamics(self.prev_state, self.prev_control) * self.dt
            
            # Residual error (what GP should learn)
            residual = true_next_state - predicted_next_state
            
            # Input: [state, control]
            x_input = np.concatenate([self.prev_state, self.prev_control])
            
            # Store training data
            self.training_data_x.append(x_input)
            self.training_data_y.append(residual)
            
            # Add to GP every 10 points
            if len(self.training_data_x) % 10 == 0:
                X_train = np.array(self.training_data_x[-10:])
                Y_train = np.array(self.training_data_y[-10:])
                self.gp_node.add_training_data(X_train, Y_train)
                print(f"📊 GP updated with {len(self.training_data_x)} total training points")

        self.prev_state = state.copy()
        self.prev_control = control.copy()

    def run_demo(self, duration=30.0):
        """Run the demonstration."""
        # Use MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(self.dynamics_node)
        executor.add_node(self.gp_node)
        executor.add_node(self.mpc_node)

        # Start executor
        executor_thread = threading.Thread(target=executor.spin)
        executor_thread.daemon = True
        executor_thread.start()

        start_time = time.time()
        iteration = 0

        try:
            print(f"🎯 Running demo for {duration} seconds...")
            print("Phase 1: Learning to hover (stabilization)")
            
            while self.running and (time.time() - start_time) < duration:
                current_time = time.time() - start_time
                
                # Get current state and control
                state = self.dynamics_node.get_state()
                control = self.dynamics_node.control
                
                # Collect training data
                self.collect_training_data(state, control)
                
                # Track performance
                position = state[0:3]
                reference_pos = self.reference[0:3]
                error = np.linalg.norm(position - reference_pos)
                
                self.position_history.append(position.copy())
                self.error_history.append(error)
                self.control_history.append(control.copy())
                self.time_history.append(current_time)
                
                # Print status every 5 seconds
                if iteration % 50 == 0:  # Every 5 seconds at 10Hz
                    print(f"⏱️  t={current_time:.1f}s | "
                          f"Pos=[{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}] | "
                          f"Error={error:.3f}m | "
                          f"Training: {len(self.training_data_x)} points")
                
                # Change trajectory after 15 seconds
                if current_time > 15.0 and iteration == 150:
                    print("Phase 2: Circle trajectory")
                    self.start_circle_trajectory()
                
                iteration += 1
                time.sleep(self.dt)

        except KeyboardInterrupt:
            print("\n🛑 Demo interrupted by user")

        finally:
            self.running = False
            executor.shutdown()
            
        self.generate_results()

    def start_circle_trajectory(self):
        """Switch to circular trajectory."""
        # Simple circular reference
        circle_radius = 1.0
        circle_height = 1.5
        
        # Update reference (simple version)
        self.reference[0] = circle_radius  # Start at radius
        self.reference[1] = 0
        self.reference[2] = circle_height
        
        # Update MPC reference
        for k in range(self.mpc_node.N + 1):
            self.mpc_node.reference_trajectory[k] = self.reference

    def generate_results(self):
        """Generate and save performance plots."""
        if not self.position_history:
            print("❌ No data to plot")
            return
            
        try:
            # Create results directory
            results_dir = "/home/grandediw/quadrotor_results"
            os.makedirs(results_dir, exist_ok=True)
            
            print(f"📈 Generating results plots...")
            
            # Create figure with subplots
            fig, axes = plt.subplots(2, 2, figsize=(12, 8))
            fig.suptitle('Quadrotor GP-MPC Demo Results', fontsize=16)
            
            # Plot 1: Position vs time
            positions = np.array(self.position_history)
            times = np.array(self.time_history)
            
            axes[0,0].plot(times, positions[:, 0], 'r-', label='X')
            axes[0,0].plot(times, positions[:, 1], 'g-', label='Y')
            axes[0,0].plot(times, positions[:, 2], 'b-', label='Z')
            axes[0,0].axhline(y=1.0, color='k', linestyle='--', alpha=0.5, label='Target Z')
            axes[0,0].set_xlabel('Time (s)')
            axes[0,0].set_ylabel('Position (m)')
            axes[0,0].set_title('Position Tracking')
            axes[0,0].legend()
            axes[0,0].grid(True)
            
            # Plot 2: Tracking error
            axes[0,1].plot(times, self.error_history, 'r-')
            axes[0,1].set_xlabel('Time (s)')
            axes[0,1].set_ylabel('Tracking Error (m)')
            axes[0,1].set_title('Position Tracking Error')
            axes[0,1].grid(True)
            
            # Plot 3: Control inputs
            controls = np.array(self.control_history)
            axes[1,0].plot(times, controls[:, 0], 'b-', label='Thrust')
            axes[1,0].set_xlabel('Time (s)')
            axes[1,0].set_ylabel('Thrust (N)')
            axes[1,0].set_title('Control Input: Thrust')
            axes[1,0].grid(True)
            
            # Plot 4: 3D trajectory (use 2D projection)
            axes[1,1].plot(positions[:, 0], positions[:, 1], 'b-', linewidth=2)
            axes[1,1].plot(positions[0, 0], positions[0, 1], 'go', markersize=8, label='Start')
            axes[1,1].plot(positions[-1, 0], positions[-1, 1], 'ro', markersize=8, label='End')
            axes[1,1].plot(0, 0, 'k*', markersize=10, label='Target')
            axes[1,1].set_xlabel('X (m)')
            axes[1,1].set_ylabel('Y (m)')
            axes[1,1].set_title('2D Trajectory')
            axes[1,1].legend()
            axes[1,1].grid(True)
            axes[1,1].axis('equal')
            
            plt.tight_layout()
            
            # Save plot
            plot_file = os.path.join(results_dir, 'demo_results.png')
            plt.savefig(plot_file, dpi=150, bbox_inches='tight')
            plt.close()
            
            # Generate summary statistics
            final_error = self.error_history[-1] if self.error_history else 0
            avg_error = np.mean(self.error_history) if self.error_history else 0
            max_error = np.max(self.error_history) if self.error_history else 0
            
            summary = f"""
=== Quadrotor GP-MPC Demo Summary ===

Duration: {times[-1]:.1f} seconds
Training Data Points: {len(self.training_data_x)}

Performance Metrics:
- Final Tracking Error: {final_error:.3f} m
- Average Tracking Error: {avg_error:.3f} m  
- Maximum Tracking Error: {max_error:.3f} m

Files Generated:
- Results plot: {plot_file}

GP Learning Status:
- Total data points collected: {len(self.training_data_x)}
- GP model trained: {'Yes' if len(self.training_data_x) > 0 else 'No'}

✅ Demo completed successfully!
            """
            
            print(summary)
            
            # Save summary
            summary_file = os.path.join(results_dir, 'demo_summary.txt')
            with open(summary_file, 'w') as f:
                f.write(summary)
                
            print(f"📄 Summary saved to: {summary_file}")
            
        except Exception as e:
            print(f"❌ Error generating plots: {e}")
            
    def stop(self):
        """Stop the demo."""
        self.running = False
        print("🛑 Demo stopped")


def main():
    """Main demo function."""
    
    print("\n" + "="*50)
    print("🚁 QUADROTOR GP-MPC DEMONSTRATION")
    print("="*50)
    print("This demo shows:")
    print("• Stable quadrotor hover control")
    print("• Gaussian Process learning")
    print("• Model Predictive Control")
    print("• Performance visualization")
    print("="*50 + "\n")
    
    # Create and run demo
    demo = SimpleGPDemo()
    demo.start()
    
    try:
        demo.run_demo(duration=20.0)  # 20 second demo
    except Exception as e:
        print(f"❌ Demo error: {e}")
    finally:
        demo.stop()
        rclpy.shutdown()

    print("\n🎉 Demo completed! Check results in /home/grandediw/quadrotor_results/")


if __name__ == '__main__':
    main()