#!/usr/bin/env python3
"""
Simple takeoff demo to test Gazebo visualization
"""

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
import time

class TakeoffDemo(Node):
    def __init__(self):
        super().__init__('takeoff_demo')
        
        # Publishers
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        
        # Subscriber
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, 10)
        
        self.armed = False
        self.takeoff_height = 2.0
        
        # Timer for main control loop
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz
        self.start_time = time.time()
        
        self.get_logger().info("🚁 Takeoff Demo Started - Watch the drone in Gazebo!")
        
    def vehicle_status_callback(self, msg):
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        
    def arm_vehicle(self):
        """Send arm command"""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0  # Arm
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)
        
    def switch_to_offboard(self):
        """Switch to offboard mode"""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0  # Main mode
        msg.param2 = 6.0  # Offboard mode
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)
        
    def publish_offboard_control_mode(self):
        """Publish offboard control mode"""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_pub.publish(msg)
        
    def publish_trajectory_setpoint(self, x=0.0, y=0.0, z=0.0):
        """Publish trajectory setpoint"""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [x, y, -z]  # NED coordinates (z is negative up)
        msg.yaw = 0.0
        self.trajectory_setpoint_pub.publish(msg)
        
    def control_loop(self):
        """Main control loop"""
        current_time = time.time() - self.start_time
        
        # Always publish offboard control mode
        self.publish_offboard_control_mode()
        
        if current_time < 2.0:
            # Phase 1: Startup - publish setpoints
            self.publish_trajectory_setpoint(0.0, 0.0, 0.0)
            self.get_logger().info("🔄 Preparing for takeoff...")
            
        elif current_time < 4.0:
            # Phase 2: Arm the vehicle
            self.publish_trajectory_setpoint(0.0, 0.0, 0.0)
            if not self.armed:
                self.arm_vehicle()
                self.get_logger().info("🔫 Arming vehicle...")
            
        elif current_time < 6.0:
            # Phase 3: Switch to offboard and start takeoff
            self.switch_to_offboard()
            self.publish_trajectory_setpoint(0.0, 0.0, self.takeoff_height)
            self.get_logger().info(f"🚁 Taking off to {self.takeoff_height}m - Watch Gazebo!")
            
        else:
            # Phase 4: Hold position
            self.publish_trajectory_setpoint(0.0, 0.0, self.takeoff_height)
            if int(current_time) % 5 == 0:  # Log every 5 seconds
                self.get_logger().info(f"✈️  Hovering at {self.takeoff_height}m")

def main(args=None):
    rclpy.init(args=args)
    node = TakeoffDemo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Takeoff demo stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()