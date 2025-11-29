#!/usr/bin/env python3
"""
Simple MPC Demo without position feedback dependency
================================================================
This version uses open-loop control to demonstrate MPC trajectory
generation without relying on PX4 position feedback.
"""

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
import time
import math
import numpy as np

try:
    import casadi as ca
    CASADI_AVAILABLE = True
except ImportError:
    CASADI_AVAILABLE = False

class SimpleMPCDemo(Node):
    def __init__(self):
        super().__init__('simple_mpc_demo')
        
        if not CASADI_AVAILABLE:
            self.get_logger().error("❌ CasADi not available! Install with: pip install casadi")
            return
        
        # Publishers
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        
        # Subscribers
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, 10)
        
        # Flight state
        self.armed = False
        self.takeoff_height = 2.0
        
        # Simulated state (open-loop estimation)
        self.simulated_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # [x, y, z, vx, vy, vz]
        self.last_accel_cmd = np.array([0.0, 0.0, 0.0])
        
        # Trajectory parameters
        self.ref_amplitude_x = 3.0
        self.ref_amplitude_y = 1.5  
        self.ref_frequency = 0.08
        
        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)
        self.start_time = time.time()
        
        self.get_logger().info("🚁 Simple MPC Demo Started (Open-Loop)")
        
    def vehicle_status_callback(self, msg):
        """Get vehicle arming status"""
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        
    def arm_vehicle(self):
        """Send arm command"""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0
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
        msg.param1 = 1.0
        msg.param2 = 6.0
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
        
    def publish_position_setpoint(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        """Publish position setpoint"""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [x, y, -z]  # NED coordinates
        msg.yaw = yaw
        self.trajectory_setpoint_pub.publish(msg)
        
    def update_simulated_state(self, accel_cmd, dt=0.1):
        """Update simulated state using simple integration"""
        # Simple Euler integration
        # Update velocity
        self.simulated_state[3:6] += accel_cmd * dt
        # Update position 
        self.simulated_state[0:3] += self.simulated_state[3:6] * dt
        # Store acceleration command
        self.last_accel_cmd = accel_cmd.copy()
        
    def generate_reference_trajectory(self, t):
        """Generate figure-8 reference trajectory"""
        omega = 2 * np.pi * self.ref_frequency
        
        # Reference position and velocity
        x_ref = self.ref_amplitude_x * np.sin(omega * t)
        y_ref = self.ref_amplitude_y * np.sin(2 * omega * t)  
        z_ref = -self.takeoff_height
        
        vx_ref = self.ref_amplitude_x * omega * np.cos(omega * t)
        vy_ref = self.ref_amplitude_y * 2 * omega * np.cos(2 * omega * t)
        vz_ref = 0.0
        
        return np.array([x_ref, y_ref, z_ref, vx_ref, vy_ref, vz_ref])
        
    def simple_mpc_control(self, current_state, reference_state):
        """Simple MPC-style control law"""
        # Position error
        pos_error = reference_state[0:3] - current_state[0:3]
        vel_error = reference_state[3:6] - current_state[3:6]
        
        # Simple PD controller with feedforward (MPC-like)
        Kp = np.array([2.0, 2.0, 5.0])  # Position gains
        Kd = np.array([1.0, 1.0, 2.0])  # Velocity gains
        
        # Feedforward acceleration (from reference trajectory)
        omega = 2 * np.pi * self.ref_frequency
        t = time.time() - self.start_time - 12.0  # Flight time
        
        ax_ff = -self.ref_amplitude_x * omega**2 * np.sin(omega * t)
        ay_ff = -self.ref_amplitude_y * 4 * omega**2 * np.sin(2 * omega * t)
        az_ff = 0.0
        
        feedforward = np.array([ax_ff, ay_ff, az_ff])
        
        # Control law
        accel_cmd = Kp * pos_error + Kd * vel_error + feedforward
        
        # Limit acceleration
        max_accel = 2.0
        accel_cmd = np.clip(accel_cmd, -max_accel, max_accel)
        
        return accel_cmd

    def control_loop(self):
        """Main control loop"""
        current_time = time.time() - self.start_time
        
        if current_time < 2.0:
            # Phase 1: Startup
            self.publish_offboard_control_mode_position()
            self.publish_position_setpoint(0.0, 0.0, 0.0)
            self.get_logger().info("🔄 Simple MPC Demo - Initializing...")
            
        elif current_time < 4.0:
            # Phase 2: Arm
            self.publish_offboard_control_mode_position()
            self.publish_position_setpoint(0.0, 0.0, 0.0)
            if not self.armed:
                self.arm_vehicle()
                self.get_logger().info("🔫 Arming vehicle...")
            
        elif current_time < 8.0:
            # Phase 3: Takeoff
            self.switch_to_offboard()
            self.publish_offboard_control_mode_position()
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            self.get_logger().info(f"🚁 Taking off to {self.takeoff_height}m")
            
        elif current_time < 12.0:
            # Phase 4: Stabilize
            self.publish_offboard_control_mode_position()
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            self.get_logger().info("🎯 Stabilizing at hover altitude")
            
            # Initialize simulated state to hover position
            if current_time > 10.0:
                self.simulated_state = np.array([0.0, 0.0, -self.takeoff_height, 0.0, 0.0, 0.0])
            
        else:
            # Phase 5: Open-loop MPC trajectory following
            flight_time = current_time - 12.0
            
            # Generate reference
            reference_state = self.generate_reference_trajectory(flight_time)
            
            # Simple MPC control
            accel_cmd = self.simple_mpc_control(self.simulated_state, reference_state)
            
            # Update simulated state
            self.update_simulated_state(accel_cmd)
            
            # For now, continue using position control with reference trajectory
            # (since acceleration control might not work without proper position feedback)
            self.publish_offboard_control_mode_position()
            self.publish_position_setpoint(reference_state[0], reference_state[1], -reference_state[2])
            
            # Log every 2 seconds
            if int(flight_time) % 2 == 0 and flight_time > 0:
                pos_error = np.linalg.norm(self.simulated_state[0:3] - reference_state[0:3])
                vel_error = np.linalg.norm(self.simulated_state[3:6] - reference_state[3:6])
                
                self.get_logger().info(f"🎯 Open-Loop MPC Demo (t={flight_time:.1f}s):")
                self.get_logger().info(f"   📊 Reference: ({reference_state[0]:.2f}, {reference_state[1]:.2f}, {reference_state[2]:.2f})")
                self.get_logger().info(f"   📈 Simulated: ({self.simulated_state[0]:.2f}, {self.simulated_state[1]:.2f}, {self.simulated_state[2]:.2f})")
                self.get_logger().info(f"   🎮 MPC accel: ({accel_cmd[0]:.2f}, {accel_cmd[1]:.2f}, {accel_cmd[2]:.2f}) m/s²")
                self.get_logger().info(f"   ⚡ Position error: {pos_error:.3f} m")
                self.get_logger().info(f"   💨 Velocity error: {vel_error:.3f} m/s")

def main(args=None):
    print('🎯 Starting Simple MPC Demo (Open-Loop)...')
    
    if not CASADI_AVAILABLE:
        print("❌ CasADi not available! Install with: pip install casadi")
        return
    
    rclpy.init(args=args)
    demo = SimpleMPCDemo()
    
    try:
        rclpy.spin(demo)
    except KeyboardInterrupt:
        demo.get_logger().info('🛑 Demo stopped')
    finally:
        demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()