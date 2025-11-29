#!/usr/bin/env python3
"""
Debug script to check position and velocity feedback from PX4
"""

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition
import numpy as np

class DebugPosition(Node):
    def __init__(self):
        super().__init__('debug_position')
        
        # Subscribe to position
        self.position_sub = self.create_subscription(
            VehicleLocalPosition, 
            '/fmu/out/vehicle_local_position', 
            self.position_callback, 
            10
        )
        
        self.msg_count = 0
        
        self.get_logger().info("🔍 Debug Position - Monitoring position feedback...")
        
    def position_callback(self, msg):
        """Debug position callback"""
        self.msg_count += 1
        
        # Log detailed position data every 10 messages (about 1 second)
        if self.msg_count % 10 == 0:
            self.get_logger().info(f"📍 Position Debug #{self.msg_count}:")
            self.get_logger().info(f"   Raw Position: x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}")
            self.get_logger().info(f"   Raw Velocity: vx={msg.vx:.3f}, vy={msg.vy:.3f}, vz={msg.vz:.3f}")
            self.get_logger().info(f"   Position valid: {msg.xy_valid}, {msg.z_valid}")
            self.get_logger().info(f"   Velocity valid: {msg.v_xy_valid}, {msg.v_z_valid}")
            self.get_logger().info(f"   Timestamp: {msg.timestamp}")
            
            # Check for zero values (potential issue)
            if msg.x == 0.0 and msg.y == 0.0 and msg.z == 0.0:
                self.get_logger().warn("⚠️  All position values are zero!")
                
            if msg.vx == 0.0 and msg.vy == 0.0 and msg.vz == 0.0:
                self.get_logger().warn("⚠️  All velocity values are zero!")
                
            # Show current state vector format
            current_state = np.array([msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz])
            self.get_logger().info(f"   State vector: {current_state}")

def main(args=None):
    print('🔍 Starting Position Debug...')
    
    rclpy.init(args=args)
    debug_node = DebugPosition()
    
    try:
        rclpy.spin(debug_node)
    except KeyboardInterrupt:
        debug_node.get_logger().info('🛑 Debug stopped by user')
    finally:
        debug_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()