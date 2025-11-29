#!/usr/bin/env python3
"""
6-Cascade PID Controller Demo for PX4 with ROS2
===============================================
This demo implements a comprehensive 6-cascade PID control system:
- Position PIDs (X, Y, Z) -> Velocity setpoints
- Velocity PIDs (Vx, Vy, Vz) -> Acceleration/Thrust setpoints
- Attitude PIDs (Roll, Pitch, Yaw) -> Angular rate setpoints

This provides full 6-DOF control with cascaded feedback loops for precise
position and attitude control.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleAttitudeSetpoint
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, VehicleAttitude, VehicleRatesSetpoint
from std_msgs.msg import Float64MultiArray, Float64, Header
from geometry_msgs.msg import PointStamped, Vector3Stamped, QuaternionStamped
import time
import math
import numpy as np

class PIDController:
    """Enhanced PID controller with anti-windup and output limiting"""
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, max_output=5.0, max_integral=10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.max_integral = max_integral
        
        # Internal state
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        self.initialized = False
        
    def update(self, setpoint, current_value, dt=None):
        """Update PID controller and return control output"""
        current_time = time.time()
        if dt is None:
            dt = current_time - self.last_time
        
        if dt <= 0.0 or not self.initialized:
            self.last_time = current_time
            self.initialized = True
            return 0.0
            
        # Calculate error
        error = setpoint - current_value
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        # Clamp integral to prevent windup
        self.integral = max(-self.max_integral, min(self.max_integral, self.integral))
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
        self.initialized = False

def quaternion_to_euler(q):
    """Convert quaternion to Euler angles (roll, pitch, yaw)"""
    # q = [w, x, y, z]
    w, x, y, z = q[0], q[1], q[2], q[3]
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw

def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles to quaternion"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return [w, x, y, z]

class CascadePIDDemo(Node):
    def __init__(self):
        super().__init__('cascade_pid_demo')
        
        # Configure QoS profile to match PX4 requirements
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers for PX4 control
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        # CHANGED: Use VehicleRatesSetpoint for true 9-PID cascade
        self.vehicle_rates_setpoint_pub = self.create_publisher(VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', 10)

        # Monitoring publishers
        
        # Publishers for monitoring (PlotJuggler compatible)
        self.position_setpoint_pub = self.create_publisher(PointStamped, '/cascade_pid/position_setpoint', 10)
        self.position_current_pub = self.create_publisher(PointStamped, '/cascade_pid/position_current', 10)
        self.velocity_setpoint_pub = self.create_publisher(Vector3Stamped, '/cascade_pid/velocity_setpoint', 10)
        self.velocity_current_pub = self.create_publisher(Vector3Stamped, '/cascade_pid/velocity_current', 10)
        self.attitude_setpoint_pub = self.create_publisher(Vector3Stamped, '/cascade_pid/attitude_setpoint', 10)
        self.attitude_current_pub = self.create_publisher(Vector3Stamped, '/cascade_pid/attitude_current', 10)
        self.position_error_pub = self.create_publisher(Vector3Stamped, '/cascade_pid/position_error', 10)
        self.velocity_error_pub = self.create_publisher(Vector3Stamped, '/cascade_pid/velocity_error', 10)
        self.attitude_error_pub = self.create_publisher(Vector3Stamped, '/cascade_pid/attitude_error', 10)
        self.control_outputs_pub = self.create_publisher(Float64MultiArray, '/cascade_pid/control_outputs', 10)
        
        # Subscribers for feedback
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
        self.vehicle_attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile
        )
        
        # Flight state variables
        self.armed = False
        self.takeoff_height = 3.0
        
        # Current state
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_velocity = np.array([0.0, 0.0, 0.0])
        self.current_attitude = np.array([0.0, 0.0, 0.0])  # [roll, pitch, yaw]
        self.current_angular_velocity = np.array([0.0, 0.0, 0.0])
        
        # Setpoints
        self.position_setpoint = np.array([0.0, 0.0, self.takeoff_height])
        self.velocity_setpoint = np.array([0.0, 0.0, 0.0])
        self.attitude_setpoint = np.array([0.0, 0.0, 0.0])  # [roll, pitch, yaw]
        
        # === POSITION PID CONTROLLERS (Outer Loop) - EMERGENCY SAFE TUNING ===
        # Position -> Velocity - VERY CONSERVATIVE to prevent instability
        self.pid_pos_x = PIDController(kp=0.3, ki=0.01, kd=0.05, max_output=1.0, max_integral=0.3)
        self.pid_pos_y = PIDController(kp=0.3, ki=0.01, kd=0.05, max_output=1.0, max_integral=0.3)
        self.pid_pos_z = PIDController(kp=0.8, ki=0.04, kd=0.12, max_output=2.0, max_integral=0.8)
        
        # === VELOCITY PID CONTROLLERS (Middle Loop) - EMERGENCY SAFE TUNING ===
        # Velocity -> Attitude/Thrust - VERY CONSERVATIVE to prevent aggressive attitudes
        self.pid_vel_x = PIDController(kp=0.35, ki=0.01, kd=0.03, max_output=0.36, max_integral=0.3)  # -> pitch (max 5.7°)
        self.pid_vel_y = PIDController(kp=0.35, ki=0.01, kd=0.03, max_output=0.36, max_integral=0.3)  # -> roll (max 5.7°)
        self.pid_vel_z = PIDController(kp=0.3, ki=0.01, kd=0.02, max_output=0.8, max_integral=1.0)   # -> thrust
        
        # === ATTITUDE PID CONTROLLERS (Inner Loop) - EMERGENCY SAFE TUNING ===
        # Attitude -> Angular rates - VERY CONSERVATIVE to prevent >100°/s rate commands
        self.pid_att_roll = PIDController(kp=0.5, ki=0.02, kd=0.05, max_output=0.5, max_integral=0.1)  # max 28.6°/s
        self.pid_att_pitch = PIDController(kp=0.5, ki=0.02, kd=0.05, max_output=0.5, max_integral=0.1)  # max 28.6°/s
        self.pid_att_yaw = PIDController(kp=0.3, ki=0.01, kd=0.02, max_output=0.3, max_integral=0.05)  # max 17.2°/s
        
        # Demo pattern parameters - CONSERVATIVE for initial testing
        self.pattern_amplitude = 6.0  # meters (smaller circle for safety)
        self.pattern_frequency = 0.02  # Hz (very slow pattern for testing)
        self.pattern_height_var = 0.5  # meters height variation
        
        # Control mode
        self.use_cascade_control = True
        self.control_phase = "position"  # "position", "velocity", "attitude", "cascade"
        
        # Timer for main control loop
        self.timer = self.create_timer(0.02, self.control_loop)  # 50Hz for precise control
        self.start_time = time.time()
        
        self.get_logger().info("🎯 6-Cascade PID Demo Started!")
        self.get_logger().info("📊 Control Loops: Position->Velocity->Attitude->Rates")
        
    def vehicle_status_callback(self, msg):
        """Get vehicle arming status"""
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        
    def position_callback(self, msg):
        """Get current position and velocity feedback"""
        self.current_position = np.array([msg.x, msg.y, -msg.z])   ##### DOUBLE CHECK NED!!!!!!
        self.current_velocity = np.array([msg.vx, msg.vy, -msg.vz])
        
    def attitude_callback(self, msg):
        """Get current attitude feedback"""
        # Convert quaternion to Euler angles
        q = [msg.q[0], msg.q[1], msg.q[2], msg.q[3]]  # [w, x, y, z]
        roll, pitch, yaw = quaternion_to_euler(q)
        self.current_attitude = np.array([roll, pitch, yaw])
        
        # Angular velocity is also provided in the message
        # Note: PX4 provides rollspeed, pitchspeed, yawspeed
        # For this demo, we'll use the attitude feedback primarily
        
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
        
    def publish_offboard_control_mode_rates(self):
        """Publish offboard control mode for body rate control - TRUE 9-PID CASCADE"""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False  # DISABLED - we control attitude with our own PIDs
        msg.body_rate = True  # ENABLED - we send rate commands directly
        self.offboard_control_mode_pub.publish(msg)
        
    def publish_position_setpoint(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        """Publish position setpoint"""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [x, y, -z]  # NED coordinates (z is negative up)
        msg.yaw = yaw
        self.trajectory_setpoint_pub.publish(msg)
        
    def publish_rates_setpoint(self, rollrate=0.0, pitchrate=0.0, yawrate=0.0, thrust=0.5):
        """Publish body rate and thrust setpoint - TRUE 9-PID CASCADE"""
        msg = VehicleRatesSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # Set the angular rates (rad/s) - direct output from attitude PIDs
        msg.roll = float(rollrate)
        msg.pitch = float(pitchrate)
        msg.yaw = float(yawrate)

        # Set thrust (normalized 0-1, in NED frame)
        normalized_thrust = np.clip(thrust, 0.0, 1.0)
        msg.thrust_body[0] = 0.0
        msg.thrust_body[1] = 0.0
        msg.thrust_body[2] = -normalized_thrust  # Negative for upward thrust in NED
        
        self.vehicle_rates_setpoint_pub.publish(msg)
    ###### CIRCULAR TRAJECTORY   
     
    # def calculate_reference_trajectory(self, flight_time):
    #     """Calculate reference position trajectory - CIRCULAR TRAJECTORY"""
    #     # Circular trajectory parameters
    #     radius = self.pattern_amplitude  # 2.0 meters radius
    #     angular_frequency = 2 * math.pi * self.pattern_frequency  # rad/s

    #     # Calculate circular trajectory in XY plane
    #     angle = angular_frequency * flight_time
    #     x = radius * math.cos(angle)
    #     y = radius * math.sin(angle)
    #     # x = 0.0  # Hover at origin X
    #     # y = 0.0  # Hover at origin Y
    #     z = self.takeoff_height  # Maintain constant altitude

    #     # Optional: Yaw to face direction of motion (tangent to circle)
    #     # yaw = angle + math.pi/2  # Face forward along circle
    #     #x = 0.0  # Hover at origin X
    #     #y = 0.0
    #     yaw = 0.0  # Keep constant yaw for simplicity

    #     return np.array([x, y, z]), yaw

    #### 8 TRAJECTORY
    def calculate_reference_trajectory(self, t):
        """
        Figure-8 (Gerono) in XY at constant Z.
        x(t) = A * sin(ω t)
        y(t) = (A/2) * sin(2 ω t)
        z    = constant (internal +up)
        """
        A = float(self.pattern_amplitude)          # meters
        w = 2.0 * math.pi * float(self.pattern_frequency)  # rad/s

        # Optional smooth start to avoid step accelerations
        ramp = math.tanh(max(t, 0.0) / 5.0)  # 5 s ramp-in

        # Position
        x = ramp * (A * math.sin(w * t))
        #y = ramp * ((A / 2.0) * math.sin(2.0 * w * t)) # Figure-8
        y = ramp * (A * math.cos(w * t))    # Circle
        z = self.takeoff_height

        # Velocities (for yaw tangent)
        vx = ramp * (A * w * math.cos(w * t))
        vy = ramp * (A * w * math.cos(2.0 * w * t))

        # Yaw: set one of the two lines below
        yaw = 0.0                          # keep constant yaw (simpler)
        #yaw = math.atan2(vy, vx + 1e-6)      # face along the path (tangent)

        return np.array([x, y, z]), yaw
        
    def update_cascade_control(self, dt):
        """Update the full 9-CASCADE PID control system - TRUE CASCADE CONTROL"""
        
        # === OUTER LOOP: Position Control ===
        # Position PID -> Velocity setpoints
        vel_x_cmd = self.pid_pos_x.update(self.position_setpoint[0], self.current_position[0], dt)
        vel_y_cmd = self.pid_pos_y.update(self.position_setpoint[1], self.current_position[1], dt)
        vel_z_cmd = self.pid_pos_z.update(self.position_setpoint[2], self.current_position[2], dt)
        
        # Update velocity setpoints
        self.velocity_setpoint = np.array([vel_x_cmd, vel_y_cmd, vel_z_cmd])
        
        # === MIDDLE LOOP: Velocity Control ===
        # Velocity PID -> Attitude/Thrust setpoints
        # Note: In NED frame, forward velocity (vx) requires pitch down (negative pitch)
        # Rightward velocity (vy) requires roll right (positive roll)
        
        pitch_cmd = -self.pid_vel_x.update(self.velocity_setpoint[0], self.current_velocity[0], dt)  # Forward vel -> pitch down
        roll_cmd = self.pid_vel_y.update(self.velocity_setpoint[1], self.current_velocity[1], dt)    # Right vel -> roll right
        thrust_cmd = self.pid_vel_z.update(self.velocity_setpoint[2], self.current_velocity[2], dt)  # Up vel -> more thrust
        
        # Update attitude setpoints (roll, pitch from velocity control + desired yaw)
        self.attitude_setpoint[0] = roll_cmd   # Roll
        self.attitude_setpoint[1] = pitch_cmd  # Pitch
        # Yaw is set separately from trajectory
        
        # Convert thrust command to normalized thrust - INCREASED FOR ALTITUDE RECOVERY
        # Higher base thrust to overcome gravity + more authority for large altitude errors
        HOVER_THRUST = 0.7 #0.36
        thrust_normalized = np.clip(HOVER_THRUST + thrust_cmd , 0.1, 1.15)
        
        # === INNER LOOP: Attitude Control ===
        # Attitude PID -> Angular rate setpoints (not directly used in this demo)
        # For demonstration, we'll calculate what the rate commands would be
        rollrate_cmd = self.pid_att_roll.update(self.attitude_setpoint[0], self.current_attitude[0], dt)
        pitchrate_cmd = self.pid_att_pitch.update(self.attitude_setpoint[1], self.current_attitude[1], dt)
        yawrate_cmd = self.pid_att_yaw.update(self.attitude_setpoint[2], self.current_attitude[2], dt)
        
        # EMERGENCY SAFETY: Additional rate limiting to prevent instability
        max_safe_rate = 0.7  # 28.6 degrees/second maximum  # Correct value 0.5
        rollrate_cmd = np.clip(rollrate_cmd, -max_safe_rate, max_safe_rate)
        pitchrate_cmd = np.clip(pitchrate_cmd, -max_safe_rate, max_safe_rate)
        yawrate_cmd = np.clip(yawrate_cmd, -max_safe_rate, max_safe_rate)
        
        return {
            'velocity_setpoint': self.velocity_setpoint,
            'attitude_setpoint': self.attitude_setpoint,
            'thrust': thrust_normalized,
            'rate_setpoint': np.array([rollrate_cmd, pitchrate_cmd, yawrate_cmd])
        }
        
    def publish_monitoring_data(self, control_outputs, flight_time):
        """Publish monitoring data for PlotJuggler"""
        current_time = self.get_clock().now()
        
        # Position data
        pos_setpoint_msg = PointStamped()
        pos_setpoint_msg.header.stamp = current_time.to_msg()
        pos_setpoint_msg.header.frame_id = "map"
        pos_setpoint_msg.point.x = float(self.position_setpoint[0])
        pos_setpoint_msg.point.y = float(self.position_setpoint[1])
        pos_setpoint_msg.point.z = float(self.position_setpoint[2])
        self.position_setpoint_pub.publish(pos_setpoint_msg)
        
        pos_current_msg = PointStamped()
        pos_current_msg.header.stamp = current_time.to_msg()
        pos_current_msg.header.frame_id = "map"
        pos_current_msg.point.x = float(self.current_position[0])
        pos_current_msg.point.y = float(self.current_position[1])
        pos_current_msg.point.z = float(self.current_position[2])
        self.position_current_pub.publish(pos_current_msg)
        
        # Velocity data
        vel_setpoint_msg = Vector3Stamped()
        vel_setpoint_msg.header.stamp = current_time.to_msg()
        vel_setpoint_msg.header.frame_id = "map"
        vel_setpoint_msg.vector.x = float(control_outputs['velocity_setpoint'][0])
        vel_setpoint_msg.vector.y = float(control_outputs['velocity_setpoint'][1])
        vel_setpoint_msg.vector.z = float(control_outputs['velocity_setpoint'][2])
        self.velocity_setpoint_pub.publish(vel_setpoint_msg)
        
        vel_current_msg = Vector3Stamped()
        vel_current_msg.header.stamp = current_time.to_msg()
        vel_current_msg.header.frame_id = "map"
        vel_current_msg.vector.x = float(self.current_velocity[0])
        vel_current_msg.vector.y = float(self.current_velocity[1])
        vel_current_msg.vector.z = float(self.current_velocity[2])
        self.velocity_current_pub.publish(vel_current_msg)
        
        # Attitude data
        att_setpoint_msg = Vector3Stamped()
        att_setpoint_msg.header.stamp = current_time.to_msg()
        att_setpoint_msg.header.frame_id = "map"
        att_setpoint_msg.vector.x = float(control_outputs['attitude_setpoint'][0])  # roll
        att_setpoint_msg.vector.y = float(control_outputs['attitude_setpoint'][1])  # pitch
        att_setpoint_msg.vector.z = float(control_outputs['attitude_setpoint'][2])  # yaw
        self.attitude_setpoint_pub.publish(att_setpoint_msg)
        
        att_current_msg = Vector3Stamped()
        att_current_msg.header.stamp = current_time.to_msg()
        att_current_msg.header.frame_id = "map"
        att_current_msg.vector.x = float(self.current_attitude[0])  # roll
        att_current_msg.vector.y = float(self.current_attitude[1])  # pitch
        att_current_msg.vector.z = float(self.current_attitude[2])  # yaw
        self.attitude_current_pub.publish(att_current_msg)
        
        # Error signals
        pos_error = self.position_setpoint - self.current_position
        pos_error_msg = Vector3Stamped()
        pos_error_msg.header.stamp = current_time.to_msg()
        pos_error_msg.header.frame_id = "map"
        pos_error_msg.vector.x = float(pos_error[0])
        pos_error_msg.vector.y = float(pos_error[1])
        pos_error_msg.vector.z = float(pos_error[2])
        self.position_error_pub.publish(pos_error_msg)
        
        vel_error = control_outputs['velocity_setpoint'] - self.current_velocity
        vel_error_msg = Vector3Stamped()
        vel_error_msg.header.stamp = current_time.to_msg()
        vel_error_msg.header.frame_id = "map"
        vel_error_msg.vector.x = float(vel_error[0])
        vel_error_msg.vector.y = float(vel_error[1])
        vel_error_msg.vector.z = float(vel_error[2])
        self.velocity_error_pub.publish(vel_error_msg)
        
        att_error = control_outputs['attitude_setpoint'] - self.current_attitude
        att_error_msg = Vector3Stamped()
        att_error_msg.header.stamp = current_time.to_msg()
        att_error_msg.header.frame_id = "map"
        att_error_msg.vector.x = float(att_error[0])
        att_error_msg.vector.y = float(att_error[1])
        att_error_msg.vector.z = float(att_error[2])
        self.attitude_error_pub.publish(att_error_msg)
        
        # Comprehensive control data
        control_msg = Float64MultiArray()
        control_msg.data = [
            float(self.position_setpoint[0]), float(self.position_setpoint[1]), float(self.position_setpoint[2]),  # pos setpoint
            float(self.current_position[0]), float(self.current_position[1]), float(self.current_position[2]),    # pos current
            float(control_outputs['velocity_setpoint'][0]), float(control_outputs['velocity_setpoint'][1]), float(control_outputs['velocity_setpoint'][2]),  # vel setpoint
            float(self.current_velocity[0]), float(self.current_velocity[1]), float(self.current_velocity[2]),    # vel current
            float(control_outputs['attitude_setpoint'][0]), float(control_outputs['attitude_setpoint'][1]), float(control_outputs['attitude_setpoint'][2]),  # att setpoint
            float(self.current_attitude[0]), float(self.current_attitude[1]), float(self.current_attitude[2]),    # att current
            float(control_outputs['thrust']),  # thrust
            float(flight_time)  # time
        ]
        self.control_outputs_pub.publish(control_msg)

    def control_loop(self):
        """Main control loop with cascade PID control"""
        current_time = time.time() - self.start_time
        
        if current_time < 2.0:
            # Phase 1: Startup - position control
            self.publish_offboard_control_mode_position()
            self.publish_position_setpoint(0.0, 0.0, 0.0)
            self.get_logger().info("🔄 Preparing 6-Cascade PID controller...")
            
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
            
        elif current_time < 20.0:
            # Phase 4: Extended hover and stabilize - position control
            self.publish_offboard_control_mode_position()
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            self.get_logger().info("🎯 Hovering - Stabilizing before cascade control")
            
        else:
            # Phase 5: Cascade PID hover-only control
            flight_time = current_time - 20.0
            
            # Calculate reference trajectory
            target_position, target_yaw = self.calculate_reference_trajectory(flight_time)
            self.position_setpoint = target_position
            self.attitude_setpoint[2] = target_yaw  # Set desired yaw
            
            # Update cascade control system
            dt = 0.02  # 50Hz control loop
            control_outputs = self.update_cascade_control(dt)
            
            # Switch to body rate control mode (TRUE 9-PID CASCADE!)
            self.publish_offboard_control_mode_rates()
            
            # Send body rate and thrust commands to PX4
            self.publish_rates_setpoint(
                rollrate=control_outputs['rate_setpoint'][0],
                pitchrate=control_outputs['rate_setpoint'][1], 
                yawrate=control_outputs['rate_setpoint'][2],
                thrust=control_outputs['thrust']
            )
            
            # Publish monitoring data
            self.publish_monitoring_data(control_outputs, flight_time)

            
            
            # Log performance every 2 seconds in circular trajectory mode
            if int(flight_time) % 2 == 0 and flight_time > 0:
                pos_error = np.linalg.norm(self.position_setpoint - self.current_position)
                vel_error = np.linalg.norm(control_outputs['velocity_setpoint'] - self.current_velocity)
                att_error = np.linalg.norm(control_outputs['attitude_setpoint'] - self.current_attitude)
                
                self.get_logger().info(f"� CIRCULAR TRAJECTORY - 9-CASCADE PID Control (TRUE CASCADE!) (t={flight_time:.1f}s):")
                self.get_logger().info(f"   📍 Pos: setpoint=({self.position_setpoint[0]:.1f}, {self.position_setpoint[1]:.1f}, {self.position_setpoint[2]:.1f}), current=({self.current_position[0]:.1f}, {self.current_position[1]:.1f}, {self.current_position[2]:.1f}), error={pos_error:.3f}m")
                self.get_logger().info(f"   🏃 Vel: setpoint=({control_outputs['velocity_setpoint'][0]:.2f}, {control_outputs['velocity_setpoint'][1]:.2f}, {control_outputs['velocity_setpoint'][2]:.2f}), current=({self.current_velocity[0]:.2f}, {self.current_velocity[1]:.2f}, {self.current_velocity[2]:.2f}), error={vel_error:.3f}m/s")
                self.get_logger().info(f"   🎪 Att: setpoint=({math.degrees(control_outputs['attitude_setpoint'][0]):.1f}°, {math.degrees(control_outputs['attitude_setpoint'][1]):.1f}°, {math.degrees(control_outputs['attitude_setpoint'][2]):.1f}°)")
                self.get_logger().info(f"        current=({math.degrees(self.current_attitude[0]):.1f}°, {math.degrees(self.current_attitude[1]):.1f}°, {math.degrees(self.current_attitude[2]):.1f}°), error={math.degrees(att_error):.1f}°")
                self.get_logger().info(f"   � Rate: commands=({math.degrees(control_outputs['rate_setpoint'][0]):.1f}°/s, {math.degrees(control_outputs['rate_setpoint'][1]):.1f}°/s, {math.degrees(control_outputs['rate_setpoint'][2]):.1f}°/s)")
                self.get_logger().info(f"   �🚀 Thrust: {control_outputs['thrust']:.3f}")


def main(args=None):
    print('🎯 Starting TRUE 9-CASCADE PID Demo...')
    print('📊 Control Architecture:')
    print('   Position PIDs -> Velocity setpoints')
    print('   Velocity PIDs -> Attitude setpoints')
    print('   Attitude PIDs -> Body Rate commands to PX4')
    print('⚠️  CRITICAL: PX4 attitude stabilization DISABLED!') 
    print('   Attitude PIDs -> Rate setpoints')
    print('')
    
    rclpy.init(args=args)
    cascade_pid_demo = CascadePIDDemo()
    
    try:
        rclpy.spin(cascade_pid_demo)
    except KeyboardInterrupt:
        cascade_pid_demo.get_logger().info('🛑 9-CASCADE PID demo stopped by user')
    finally:
        cascade_pid_demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()