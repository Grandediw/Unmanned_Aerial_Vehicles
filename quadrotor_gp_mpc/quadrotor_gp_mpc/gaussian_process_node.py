#!/usr/bin/env python3
"""
Gaussian Process Learning Node for Gazebo x500 quadrotor.

Subscribes to:
- /gz_x500/state: Current state
- /gz_x500/control_input: Control commands

Publishes to:
- /gz_x500/gp_predictions: GP model predictions
- /gp/metrics: GP learning metrics
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64MultiArray

from quadrotor_gp_mpc.gaussian_process import GaussianProcess
from quadrotor_gp_mpc.performance_metrics import GPMetrics


class GaussianProcessNode(Node):
    """Gaussian Process Learning Node for quadrotor dynamics."""
    
    def __init__(self):
        super().__init__('gaussian_process_node')
        
        # Declare parameters (for documentation, though GP uses fixed parameters)
        self.declare_parameter('enabled', True)
        self.declare_parameter('training_interval', 10)
        
        # Get parameters
        self.enabled = self.get_parameter('enabled').value
        self.training_interval = self.get_parameter('training_interval').value
        
        # Initialize Gaussian Process (uses default parameters from class)
        self.gp = GaussianProcess()
        
        # Training data buffers
        self.X_train = []  # State-control pairs
        self.y_train = []  # State derivatives
        
        # State tracking
        self.current_state = np.zeros(12)
        self.current_control = np.zeros(4)
        self.previous_state = np.zeros(12)
        self.step_count = 0
        
        # Metrics
        self.gp_metrics = GPMetrics()
        
        # QoS profile
        qos_profile = QoSProfile(depth=10)
        
        # Publishers
        self.pred_pub = self.create_publisher(
            Float64MultiArray, '/gz_x500/gp_predictions', qos_profile
        )
        self.metrics_pub = self.create_publisher(
            Float64MultiArray, '/gp/metrics', qos_profile
        )
        
        # Subscribers
        self.state_sub = self.create_subscription(
            Float64MultiArray, '/gz_x500/state', self.state_callback, qos_profile
        )
        self.control_sub = self.create_subscription(
            Float64MultiArray, '/gz_x500/control_input', self.control_callback, qos_profile
        )
        
        # Timers
        self.training_timer = self.create_timer(
            self.training_interval * 0.01,  # Convert to seconds
            self.train_callback
        )
        self.metrics_timer = self.create_timer(1.0, self.publish_metrics_callback)
        
        self.get_logger().info(
            f"Gaussian Process Node initialized\n"
            f"  Training interval: {self.training_interval} steps\n"
            f"  Enabled: {self.enabled}"
        )
    
    def state_callback(self, msg):
        """Update current state."""
        if len(msg.data) >= 12:
            self.current_state = np.array(msg.data[:12])
        else:
            self.get_logger().warn(f"Invalid state length: {len(msg.data)}")
    
    def control_callback(self, msg):
        """Update current control input."""
        if len(msg.data) >= 4:
            self.current_control = np.array(msg.data[:4])
    
    def train_callback(self):
        """Periodically train GP on collected data."""
        if not self.enabled or len(self.X_train) < 2:
            return
        
        try:
            # Compute state derivative (actual from simulation)
            dt = 0.01
            state_derivative = (self.current_state - self.previous_state) / dt
            
            # Add to training data
            # Features: [state (12) + control (4)] = 16 input features
            x = np.concatenate([self.current_state, self.current_control])
            self.X_train.append(x)
            self.y_train.append(state_derivative)
            
            # Update GP if enough data
            if len(self.X_train) >= 5:
                X = np.array(self.X_train)
                y = np.array(self.y_train)
                
                # Train on latest batch
                try:
                    self.gp.fit(X[-50:], y[-50:, :3])  # Train on position dynamics
                    
                    # Make predictions on current state-control
                    x_test = np.concatenate([
                        self.current_state,
                        self.current_control
                    ]).reshape(1, -1)
                    
                    predictions = self.gp.predict(x_test)
                    
                    # Publish predictions
                    msg = Float64MultiArray()
                    msg.data = predictions[0].tolist() if len(predictions) > 0 else [0, 0, 0]
                    self.pred_pub.publish(msg)
                    
                except Exception as e:
                    self.get_logger().warn(f"GP training warning: {str(e)}")
            
            # Store for derivative computation
            self.previous_state = self.current_state.copy()
            self.step_count += 1
            
        except Exception as e:
            self.get_logger().error(f"Training error: {str(e)}")
    
    def publish_metrics_callback(self):
        """Publish GP metrics periodically."""
        try:
            msg = Float64MultiArray()
            msg.data = [
                float(len(self.X_train)),          # Number of training points
                float(len(self.gp.X_train)) if hasattr(self.gp, 'X_train') else 0,
                float(self.step_count)             # Total steps
            ]
            self.metrics_pub.publish(msg)
            
            if len(self.X_train) % 50 == 0:
                self.get_logger().info(
                    f"GP Status: {len(self.X_train)} training points collected"
                )
            
        except Exception as e:
            self.get_logger().error(f"Metrics publication error: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = GaussianProcessNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
