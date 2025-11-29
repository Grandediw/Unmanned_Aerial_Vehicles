#!/usr/bin/env python3
"""
Quick QoS Test
==============
Test if we can receive PX4 messages with correct QoS settings
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition
import time

class QoSTest(Node):
    def __init__(self):
        super().__init__('qos_test')
        
        # Configure QoS to match PX4 exactly
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.get_logger().info(f"🔧 Using QoS: {qos_profile.reliability.name}, {qos_profile.durability.name}")
        
        # Test subscribers
        self.status_sub = self.create_subscription(
            VehicleStatus, 
            '/fmu/out/vehicle_status', 
            self.status_callback, 
            qos_profile
        )
        
        self.position_sub = self.create_subscription(
            VehicleLocalPosition, 
            '/fmu/out/vehicle_local_position', 
            self.position_callback, 
            qos_profile
        )
        
        self.status_count = 0
        self.position_count = 0
        self.start_time = time.time()
        
        # Timer to report status
        self.timer = self.create_timer(2.0, self.report_status)
        
    def status_callback(self, msg):
        self.status_count += 1
        if self.status_count == 1:
            self.get_logger().info("✅ First vehicle status received!")
            
    def position_callback(self, msg):
        self.position_count += 1
        if self.position_count == 1:
            self.get_logger().info("✅ First position received!")
            self.get_logger().info(f"📍 Position: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}")
            self.get_logger().info(f"📍 Valid flags: xy_valid={getattr(msg, 'xy_valid', 'N/A')}, z_valid={getattr(msg, 'z_valid', 'N/A')}")
            
    def report_status(self):
        elapsed = time.time() - self.start_time
        self.get_logger().info(f"📊 After {elapsed:.1f}s: Status messages: {self.status_count}, Position messages: {self.position_count}")
        
        if self.status_count == 0 and self.position_count == 0:
            self.get_logger().warn("⚠️  No messages received - check if PX4 and MicroXRCE agent are running")
        elif self.status_count > 0 and self.position_count == 0:
            self.get_logger().warn("⚠️  Receiving status but no position - possible EKF2 issue")

def main():
    print('🔍 Starting QoS Test...')
    
    rclpy.init()
    node = QoSTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 QoS test stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()