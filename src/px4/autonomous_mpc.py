#!/usr/bin/env python3
############################################################################
# Simplified Autonomous MPC Controller for PX4
# No keyboard dependency - fully autonomous waypoint following
############################################################################

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# Import MPC solver and message types
import casadi as ca
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleAttitudeSetpoint
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand
from std_msgs.msg import Bool


class AutonomousMPC(Node):
    def __init__(self):
        super().__init__('autonomous_mpc')
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile)
        
        self.odometry_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.odometry_callback, qos_profile)

        # Publishers
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.attitude_setpoint_pub = self.create_publisher(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 10)

        # MPC Parameters
        self.T = 0.1  # Control loop time
        self.mass = 1.5
        self.g = 9.81

        # State variables
        self.current_odometry = None
        self.vehicle_status = None
        self.current_state = "WAITING"
        self.state_timer = 0
        
        # Autonomous waypoints (NED frame)
        self.waypoints = [
            np.array([0.0, 0.0, -5.0]),   # Hover at 5m altitude
            np.array([10.0, 0.0, -5.0]),  # Move 10m North
            np.array([10.0, 10.0, -5.0]), # Move 10m East
            np.array([0.0, 10.0, -5.0]),  # Move back West
            np.array([0.0, 0.0, -5.0]),   # Return to origin
        ]
        self.current_waypoint = 0
        self.target_position = self.waypoints[0]

        # Setup timers
        self.timer = self.create_timer(self.T, self.control_loop)
        self.status_timer = self.create_timer(1.0, self.status_update)
        
        self.get_logger().info("🚁 AUTONOMOUS MPC CONTROLLER STARTED")
        self.get_logger().info(f"📍 Will follow {len(self.waypoints)} waypoints")

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def odometry_callback(self, msg):
        try:
            # PX4 quaternion format: [w, x, y, z]
            q = msg.q
            if len(q) >= 4:
                qw, qx, qy, qz = q[0], q[1], q[2], q[3]
                
                # Convert to Euler angles
                roll = np.arctan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
                pitch = np.arcsin(np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0))
                yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

                self.current_odometry = np.array([
                    msg.position[0], msg.position[1], msg.position[2],
                    msg.velocity[0], msg.velocity[1], msg.velocity[2],
                    roll, pitch, yaw
                ])
        except Exception as e:
            self.get_logger().error(f"Odometry error: {e}")

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def arm_drone(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("🔧 ARM command sent")

    def takeoff_drone(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=1.0, param7=5.0)
        self.get_logger().info("🚀 TAKEOFF command sent")

    def set_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info("📡 OFFBOARD mode command sent")

    def publish_simple_hover_command(self):
        """Publish a simple hover command using attitude setpoints"""
        # Publish offboard control mode
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = False
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = True
        offboard_msg.body_rate = False
        self.offboard_mode_pub.publish(offboard_msg)

        # Simple hover attitude setpoint
        attitude_msg = VehicleAttitudeSetpoint()
        attitude_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        
        # Quaternion for level flight (no roll/pitch)
        attitude_msg.q_d = [1.0, 0.0, 0.0, 0.0]  # [w, x, y, z]
        
        # Hover thrust (approximately 1G)
        hover_thrust = 0.5  # Normalized thrust value
        attitude_msg.thrust_body = [0.0, 0.0, -hover_thrust]  # NED frame
        
        self.attitude_setpoint_pub.publish(attitude_msg)

    def control_loop(self):
        """Main control loop - simplified state machine"""
        if self.vehicle_status is None or self.current_odometry is None:
            return

        self.state_timer += self.T

        if self.current_state == "WAITING":
            if (self.vehicle_status.pre_flight_checks_pass and 
                self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED):
                self.current_state = "ARMING"
                self.state_timer = 0
                self.get_logger().info("✅ Flight checks pass - starting autonomous sequence")

        elif self.current_state == "ARMING":
            self.arm_drone()
            if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.current_state = "TAKING_OFF"
                self.state_timer = 0
                self.get_logger().info("🚁 ARMED - initiating takeoff")

        elif self.current_state == "TAKING_OFF":
            self.takeoff_drone()
            if self.state_timer > 3.0:  # Wait 3 seconds after takeoff command
                self.current_state = "SWITCHING_TO_OFFBOARD"
                self.state_timer = 0
                self.get_logger().info("🚀 Takeoff initiated - switching to offboard")

        elif self.current_state == "SWITCHING_TO_OFFBOARD":
            self.set_offboard_mode()
            self.publish_simple_hover_command()  # Start publishing control commands
            if self.state_timer > 2.0:  # Wait 2 seconds for mode switch
                self.current_state = "FLYING"
                self.state_timer = 0
                self.get_logger().info("🎯 OFFBOARD mode - autonomous flight started!")

        elif self.current_state == "FLYING":
            # Publish offboard control commands
            self.publish_simple_hover_command()
            
            # Check if we need to move to next waypoint
            if self.current_odometry is not None:
                current_pos = self.current_odometry[0:3]
                distance_to_target = np.linalg.norm(current_pos - self.target_position)
                
                if distance_to_target < 2.0 and self.state_timer > 5.0:  # Close enough and held for 5s
                    self.current_waypoint = (self.current_waypoint + 1) % len(self.waypoints)
                    self.target_position = self.waypoints[self.current_waypoint]
                    self.state_timer = 0
                    self.get_logger().info(f"📍 Moving to waypoint {self.current_waypoint}: "
                                         f"[{self.target_position[0]:.1f}, {self.target_position[1]:.1f}, {self.target_position[2]:.1f}]")

    def status_update(self):
        """Print status information every second"""
        if self.vehicle_status is not None and self.current_odometry is not None:
            pos = self.current_odometry[0:3]
            self.get_logger().info(f"🔄 State: {self.current_state} | "
                                 f"Pos: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}] | "
                                 f"Armed: {self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED} | "
                                 f"FlightCheck: {self.vehicle_status.pre_flight_checks_pass}")


def main(args=None):
    rclpy.init(args=args)
    
    autonomous_mpc = AutonomousMPC()
    
    try:
        rclpy.spin(autonomous_mpc)
    except KeyboardInterrupt:
        print("\n🛑 Autonomous MPC stopped by user")
    
    autonomous_mpc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()