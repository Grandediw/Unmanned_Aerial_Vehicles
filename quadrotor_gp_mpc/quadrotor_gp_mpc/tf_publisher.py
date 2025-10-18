#!/usr/bin/env python3
"""
TF Publisher for Quadrotor Visualization

This node subscribes to the quadrotor state and publishes TF transforms
for visualization in RViz.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from transforms3d.euler import euler2quat


class QuadrotorTFPublisher(Node):
    """Publishes TF transforms for quadrotor visualization."""

    def __init__(self):
        super().__init__('quadrotor_tf_publisher')

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to quadrotor state
        self.state_sub = self.create_subscription(
            Float64MultiArray,
            '/quadrotor/state',
            self.state_callback,
            10
        )

        # Timer for TF publishing
        self.timer = self.create_timer(0.02, self.publish_tf)  # 50 Hz
        
        # Current state
        self.current_state = np.zeros(12)
        
        self.get_logger().info("Quadrotor TF Publisher initialized")

    def state_callback(self, msg):
        """Update current state from dynamics."""
        self.current_state = np.array(msg.data)

    def publish_tf(self):
        """Publish TF transforms for visualization."""
        # Extract state
        pos = self.current_state[0:3]
        attitude = self.current_state[6:9]  # [roll, pitch, yaw]
        
        # Create transform from world to base_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        
        # Set translation
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(pos[2])
        
        # Set rotation (convert from Euler angles to quaternion)
        quaternion = euler2quat(attitude[0], attitude[1], attitude[2])
        t.transform.rotation.w = float(quaternion[0])  # w component first in transforms3d
        t.transform.rotation.x = float(quaternion[1])
        t.transform.rotation.y = float(quaternion[2])
        t.transform.rotation.z = float(quaternion[3])
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    tf_publisher = QuadrotorTFPublisher()
    
    try:
        rclpy.spin(tf_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        tf_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()