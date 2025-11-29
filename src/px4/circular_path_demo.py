#!/usr/bin/env python3
"""
Circular Path Following Demo for PX4 with ROS2
==================================================
Based on the working takeoff_demo.py, this demo shows autonomous circular flight
"""

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
import time
import math

class CircularPathDemo(Node):
    def __init__(self):
        super().__init__('circular_path_demo')
        
        # Publishers - same as working takeoff demo
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        
        # Subscriber - same as working takeoff demo
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, 10)
        
        # Flight state variables
        self.armed = False
        self.takeoff_height = 2.0
        
        # Circular flight parameters
        self.circle_radius = 3.0      # meters
        self.circle_speed = 2.0       # m/s
        self.center_x = 0.0
        self.center_y = 0.0
        self.angular_velocity = self.circle_speed / self.circle_radius  # rad/s
        
        # Timer for main control loop - same frequency as working demo
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz
        self.start_time = time.time()
        
        self.get_logger().info("🔄 Circular Path Demo Started - Watch the drone trace circles in Gazebo!")
        
    def vehicle_status_callback(self, msg):
        """Same as working takeoff demo"""
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        
    def arm_vehicle(self):
        """Send arm command - same as working takeoff demo"""
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
        """Switch to offboard mode - same as working takeoff demo"""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0  # Custom mode
        msg.param2 = 6.0  # Offboard mode
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def publish_offboard_control_mode(self):
        """Publish offboard control mode - same as working takeoff demo"""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        """Publish trajectory setpoint - same format as working takeoff demo"""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [x, y, -z]  # NED coordinates (z is negative up)
        msg.yaw = yaw
        self.trajectory_setpoint_pub.publish(msg)

    def calculate_circular_position(self, elapsed_time):
        """Calculate position on circular trajectory"""
        angle = self.angular_velocity * elapsed_time
        x = self.center_x + self.circle_radius * math.cos(angle)
        y = self.center_y + self.circle_radius * math.sin(angle)
        z = self.takeoff_height  # Positive z (will be negated in publish method)
        yaw = angle + math.pi/2   # Face direction of motion
        return x, y, z, yaw

    def control_loop(self):
        """Main control loop - exactly like working takeoff demo"""
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
            
        elif current_time < 8.0:
            # Phase 3: Switch to offboard and takeoff
            self.switch_to_offboard()
            self.publish_trajectory_setpoint(0.0, 0.0, self.takeoff_height)
            self.get_logger().info(f"🚁 Taking off to {self.takeoff_height}m - Watch Gazebo!")
            
        else:
            # Phase 4: Circular flight
            flight_time = current_time - 8.0
            x, y, z, yaw = self.calculate_circular_position(flight_time)
            self.publish_trajectory_setpoint(x, y, z, yaw)
            
            # Log startup message once
            if flight_time < 0.1:
                self.get_logger().info("🎯 Starting circular path flight!")
                self.get_logger().info(f"   📊 Circle radius: {self.circle_radius}m")
                self.get_logger().info(f"   🏃 Circle speed: {self.circle_speed}m/s")
                self.get_logger().info(f"   ⏱️  Period: {2*math.pi/self.angular_velocity:.1f}s per circle")
            
            # Log progress every 5 seconds
            if int(flight_time) % 5 == 0 and flight_time > 0:
                circles_completed = flight_time / (2 * math.pi / self.angular_velocity)
                self.get_logger().info(f"🔄 Flying circles: {circles_completed:.1f} completed, pos: ({x:.1f}, {y:.1f}, {z:.1f})")


def main(args=None):
    print('🚁 Starting Circular Path Demo...')
    rclpy.init(args=args)
    circular_path_demo = CircularPathDemo()
    
    try:
        rclpy.spin(circular_path_demo)
    except KeyboardInterrupt:
        circular_path_demo.get_logger().info('🛑 Circular path demo stopped by user')
    finally:
        circular_path_demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()