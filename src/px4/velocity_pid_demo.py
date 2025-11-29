#!/usr/bin/env python3
"""
Velocity PID Controller Demo for PX4 with ROS2
==============================================
This demo implements a PID controller for velocity control, allowing precise
movement control with feedback from the vehicle's current velocity.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition
import time
import math

class PIDController:
    """Simple PID controller implementation"""
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, max_output=5.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        
        # Internal state
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
    def update(self, setpoint, current_value):
        """Update PID controller and return control output"""
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0.0:
            return 0.0
            
        # Calculate error
        error = setpoint - current_value
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.previous_error) / dt
        d_term = self.kd * derivative
        
        # Calculate output
        output = p_term + i_term + d_term
        
        # Apply output limits
        output = max(-self.max_output, min(self.max_output, output))
        
        # Update state
        self.previous_error = error
        self.last_time = current_time
        
        return output
        
    def reset(self):
        """Reset PID controller internal state"""
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

class VelocityPIDDemo(Node):
    def __init__(self):
        super().__init__('velocity_pid_demo')
        
        # Configure QoS profile to match PX4 requirements
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers - same as working demos
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        
        # Subscribers - get both status and position feedback with correct QoS
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, 
            '/fmu/out/vehicle_status', 
            self.vehicle_status_callback, 
            qos_profile
        )
        self.vehicle_position_sub = self.create_subscription(
            VehicleLocalPosition, 
            '/fmu/out/vehicle_local_position', 
            self.position_callback, 
            qos_profile
        )
        
        # Flight state variables
        self.armed = False
        self.takeoff_height = 2.0
        
        # Current state
        self.current_position = [0.0, 0.0, 0.0]
        self.current_velocity = [0.0, 0.0, 0.0]
        
        # PID Controllers for each axis (reduced gains for stability)
        self.pid_x = PIDController(kp=1.0, ki=0.05, kd=0.2, max_output=2.0)
        self.pid_y = PIDController(kp=1.0, ki=0.05, kd=0.2, max_output=2.0)
        self.pid_z = PIDController(kp=1.0, ki=0.05, kd=0.2, max_output=1.0)
        
        # Velocity setpoints (target velocities)
        self.velocity_setpoint = [0.0, 0.0, 0.0]  # [vx, vy, vz] in m/s
        
        # Demo pattern parameters (conservative values)
        self.pattern_amplitude = 0.5  # m/s (reduced for safety)
        self.pattern_frequency = 0.1  # Hz (slower pattern)
        
        # Timer for main control loop
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz
        self.start_time = time.time()
        
        self.get_logger().info("🎯 Velocity PID Demo Started - Advanced velocity control with feedback!")
        
    def vehicle_status_callback(self, msg):
        """Get vehicle arming status"""
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        
    def position_callback(self, msg):
        """Get current position and velocity feedback"""
        self.current_position = [msg.x, msg.y, msg.z]
        self.current_velocity = [msg.vx, msg.vy, msg.vz]
        
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
        
    def publish_position_setpoint(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        """Publish position setpoint (for initial phases)"""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [x, y, -z]  # NED coordinates (z is negative up)
        msg.yaw = yaw
        self.trajectory_setpoint_pub.publish(msg)

    def publish_offboard_control_mode_position(self):
        """Publish offboard control mode for position control"""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_pub.publish(msg)
        
    def publish_offboard_control_mode(self):
        """Publish offboard control mode for velocity control"""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = False
        msg.velocity = True  # Enable velocity control
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_pub.publish(msg)
        
    def publish_velocity_setpoint(self, vx=0.0, vy=0.0, vz=0.0, yaw=0.0):
        """Publish velocity setpoint - NED frame"""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.velocity = [vx, vy, vz]  # Velocity in NED frame (m/s)
        msg.yaw = yaw
        # Also set position to NaN to indicate velocity control
        msg.position = [float('nan'), float('nan'), float('nan')]
        self.trajectory_setpoint_pub.publish(msg)
        
    def calculate_desired_velocity(self, flight_time):
        """Calculate desired velocity pattern - figure-8 pattern"""
        # Figure-8 velocity pattern
        vx = self.pattern_amplitude * math.sin(2 * math.pi * self.pattern_frequency * flight_time)
        vy = self.pattern_amplitude * math.sin(4 * math.pi * self.pattern_frequency * flight_time)
        vz = 0.0  # Keep constant altitude
        return vx, vy, vz
        
    def update_pid_controllers(self):
        """Update PID controllers and calculate velocity commands"""
        # Get PID outputs for each axis
        vx_cmd = self.pid_x.update(self.velocity_setpoint[0], self.current_velocity[0])
        vy_cmd = self.pid_y.update(self.velocity_setpoint[1], self.current_velocity[1])
        vz_cmd = self.pid_z.update(self.velocity_setpoint[2], self.current_velocity[2])
        
        return vx_cmd, vy_cmd, vz_cmd

    def control_loop(self):
        """Main control loop - similar to working demos"""
        current_time = time.time() - self.start_time
        
        if current_time < 2.0:
            # Phase 1: Startup - position control
            self.publish_offboard_control_mode_position()
            self.publish_position_setpoint(0.0, 0.0, 0.0)
            self.get_logger().info("🔄 Preparing PID velocity controller...")
            
        elif current_time < 4.0:
            # Phase 2: Arm the vehicle - position control
            self.publish_offboard_control_mode_position()
            self.publish_position_setpoint(0.0, 0.0, 0.0)
            if not self.armed:
                self.arm_vehicle()
                self.get_logger().info("🔫 Arming vehicle...")
            
        elif current_time < 8.0:
            # Phase 3: Switch to offboard and takeoff - position control
            self.switch_to_offboard()
            self.publish_offboard_control_mode_position()
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            self.get_logger().info(f"🚁 Taking off to {self.takeoff_height}m - Watch Gazebo!")
            
        elif current_time < 12.0:
            # Phase 4: Hover and stabilize - position control
            self.publish_offboard_control_mode_position()
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            self.get_logger().info("🎯 Hovering - Preparing for velocity control")
            
        else:
            # Phase 5: Switch to velocity control and PID demo
            flight_time = current_time - 12.0
            
            # Switch to velocity control mode
            self.publish_offboard_control_mode()
            
            # Calculate desired velocity pattern
            vx_desired, vy_desired, vz_desired = self.calculate_desired_velocity(flight_time)
            
            # Update velocity setpoints
            self.velocity_setpoint = [vx_desired, vy_desired, vz_desired]
            
            # Get PID controller outputs
            vx_cmd, vy_cmd, vz_cmd = self.update_pid_controllers()
            
            # Publish velocity commands
            self.publish_velocity_setpoint(vx_cmd, vy_cmd, vz_cmd)
            
            # Log PID performance every 2 seconds
            if int(flight_time) % 2 == 0 and flight_time > 0:
                self.get_logger().info(f"🎯 PID Control Active:")
                self.get_logger().info(f"   📊 Desired vel: ({vx_desired:.2f}, {vy_desired:.2f}, {vz_desired:.2f}) m/s")
                self.get_logger().info(f"   📈 Current vel: ({self.current_velocity[0]:.2f}, {self.current_velocity[1]:.2f}, {self.current_velocity[2]:.2f}) m/s")
                self.get_logger().info(f"   🎮 PID output: ({vx_cmd:.2f}, {vy_cmd:.2f}, {vz_cmd:.2f}) m/s")
                self.get_logger().info(f"   📍 Position: ({self.current_position[0]:.1f}, {self.current_position[1]:.1f}, {self.current_position[2]:.1f}) m")


def main(args=None):
    print('🎯 Starting Velocity PID Demo...')
    rclpy.init(args=args)
    velocity_pid_demo = VelocityPIDDemo()
    
    try:
        rclpy.spin(velocity_pid_demo)
    except KeyboardInterrupt:
        velocity_pid_demo.get_logger().info('🛑 Velocity PID demo stopped by user')
    finally:
        velocity_pid_demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()