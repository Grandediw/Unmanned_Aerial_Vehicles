#!/usr/bin/env python3
"""
Performance Metrics Node for real-time monitoring.

Subscribes to:
- /gz_x500/state: Current state
- /reference_trajectory: Target state
- /mpc/metrics: MPC performance
- /gp/metrics: GP learning metrics

Publishes:
- /metrics/summary: Overall performance summary
"""

import numpy as np
import json
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64MultiArray


class PerformanceMetricsNode(Node):
    """Real-time performance metrics monitoring."""
    
    def __init__(self):
        super().__init__('performance_metrics_node')
        
        # Declare parameters
        self.declare_parameter('log_interval', 1.0)
        self.declare_parameter('save_results', True)
        self.declare_parameter('save_path', '/tmp/mpc_gz_metrics.json')
        
        # Get parameters
        self.log_interval = self.get_parameter('log_interval').value
        self.save_results = self.get_parameter('save_results').value
        self.save_path = self.get_parameter('save_path').value
        
        # State buffers
        self.current_state = np.zeros(12)
        self.reference_state = np.zeros(12)
        self.mpc_metrics = np.zeros(4)
        self.gp_metrics = np.zeros(3)
        
        # Metrics tracking
        self.position_errors = []
        self.velocity_errors = []
        self.control_inputs = []
        self.tracking_times = []
        self.gp_training_points = []
        
        self.start_time = datetime.now()
        self.step_count = 0
        
        # QoS profile
        qos_profile = QoSProfile(depth=10)
        
        # Publisher
        self.metrics_pub = self.create_publisher(
            Float64MultiArray, '/metrics/summary', qos_profile
        )
        
        # Subscribers
        self.state_sub = self.create_subscription(
            Float64MultiArray, '/gz_x500/state', self.state_callback, qos_profile
        )
        self.ref_sub = self.create_subscription(
            Float64MultiArray, '/reference_trajectory', self.reference_callback, qos_profile
        )
        self.mpc_metrics_sub = self.create_subscription(
            Float64MultiArray, '/mpc/metrics', self.mpc_metrics_callback, qos_profile
        )
        self.gp_metrics_sub = self.create_subscription(
            Float64MultiArray, '/gp/metrics', self.gp_metrics_callback, qos_profile
        )
        
        # Timer for logging
        self.log_timer = self.create_timer(self.log_interval, self.log_callback)
        
        self.get_logger().info(
            f"Performance Metrics Node initialized\n"
            f"  Log interval: {self.log_interval}s\n"
            f"  Save results: {self.save_results}\n"
            f"  Save path: {self.save_path}"
        )
    
    def state_callback(self, msg):
        """Update current state."""
        if len(msg.data) >= 12:
            self.current_state = np.array(msg.data[:12])
    
    def reference_callback(self, msg):
        """Update reference state."""
        if len(msg.data) >= 12:
            self.reference_state = np.array(msg.data[:12])
    
    def mpc_metrics_callback(self, msg):
        """Update MPC metrics."""
        if len(msg.data) >= 4:
            self.mpc_metrics = np.array(msg.data[:4])
    
    def gp_metrics_callback(self, msg):
        """Update GP metrics."""
        if len(msg.data) >= 3:
            self.gp_metrics = np.array(msg.data[:3])
    
    def log_callback(self):
        """Log and publish performance metrics."""
        try:
            # Compute errors
            pos_error = np.linalg.norm(self.current_state[:3] - self.reference_state[:3])
            vel_error = np.linalg.norm(self.current_state[3:6] - self.reference_state[3:6])
            att_error = np.linalg.norm(self.current_state[6:9] - self.reference_state[6:9])
            
            # Track metrics
            self.position_errors.append(pos_error)
            self.velocity_errors.append(vel_error)
            self.gp_training_points.append(self.gp_metrics[0])
            
            # Compute statistics
            elapsed_time = (datetime.now() - self.start_time).total_seconds()
            
            # Create summary message
            summary = Float64MultiArray()
            summary.data = [
                pos_error,                          # Current position error
                np.mean(self.position_errors[-10:]),  # Mean error (last 10 samples)
                np.max(self.position_errors),       # Max position error
                vel_error,                          # Current velocity error
                self.mpc_metrics[0],                # MPC error
                self.mpc_metrics[1],                # MPC velocity
                self.mpc_metrics[2],                # Thrust command
                self.gp_metrics[0],                 # GP training points
                elapsed_time,                       # Elapsed time
                float(self.step_count)              # Step count
            ]
            self.metrics_pub.publish(summary)
            
            # Log periodically
            if self.step_count % 10 == 0:
                self.get_logger().info(
                    f"\n=== Performance Metrics (Step {self.step_count}) ===\n"
                    f"Time: {elapsed_time:.2f}s\n"
                    f"Position Error: {pos_error:.4f}m (mean: {np.mean(self.position_errors[-10:]):.4f}m)\n"
                    f"Velocity Error: {vel_error:.4f}m/s\n"
                    f"Attitude Error: {att_error:.4f}rad\n"
                    f"GP Training Points: {int(self.gp_metrics[0])}\n"
                    f"Current Thrust: {self.mpc_metrics[2]:.4f}N"
                )
            
            self.step_count += 1
            
            # Save results periodically
            if self.save_results and self.step_count % 100 == 0:
                self.save_metrics()
            
        except Exception as e:
            self.get_logger().error(f"Metrics logging error: {str(e)}")
    
    def save_metrics(self):
        """Save metrics to JSON file."""
        try:
            metrics_dict = {
                'timestamp': datetime.now().isoformat(),
                'elapsed_time': (datetime.now() - self.start_time).total_seconds(),
                'step_count': self.step_count,
                'position_error': {
                    'mean': float(np.mean(self.position_errors)) if self.position_errors else 0.0,
                    'max': float(np.max(self.position_errors)) if self.position_errors else 0.0,
                    'min': float(np.min(self.position_errors)) if self.position_errors else 0.0,
                },
                'velocity_error': {
                    'mean': float(np.mean(self.velocity_errors)) if self.velocity_errors else 0.0,
                    'max': float(np.max(self.velocity_errors)) if self.velocity_errors else 0.0,
                },
                'gp_training_points': float(self.gp_metrics[0]),
                'current_state': self.current_state.tolist(),
                'reference_state': self.reference_state.tolist(),
            }
            
            with open(self.save_path, 'w') as f:
                json.dump(metrics_dict, f, indent=2)
            
            self.get_logger().info(f"Metrics saved to {self.save_path}")
            
        except Exception as e:
            self.get_logger().error(f"Metrics save error: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = PerformanceMetricsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
