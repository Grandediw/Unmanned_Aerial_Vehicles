#!/usr/bin/env python3
"""
PX4 Dynamics Analyzer - Ground Truth System Identification
=========================================================
This tool analyzes PX4's actual dynamics by performing system identification
experiments to understand the real relationship between commands and response.
"""

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, VehicleAttitude
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

class DynamicsAnalyzer(Node):
    def __init__(self):
        super().__init__('dynamics_analyzer')
        
        # Publishers
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        
        # Subscribers
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, 10)
        self.vehicle_position_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.position_callback, 10)
        self.vehicle_attitude_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, 10)
        
        # State variables
        self.armed = False
        self.takeoff_height = 2.0
        
        # Data collection for system identification
        self.experiment_data = {
            'time': deque(maxlen=2000),
            'command_ax': deque(maxlen=2000),
            'command_ay': deque(maxlen=2000), 
            'command_az': deque(maxlen=2000),
            'actual_ax': deque(maxlen=2000),
            'actual_ay': deque(maxlen=2000),
            'actual_az': deque(maxlen=2000),
            'position_x': deque(maxlen=2000),
            'position_y': deque(maxlen=2000),
            'position_z': deque(maxlen=2000),
            'velocity_x': deque(maxlen=2000),
            'velocity_y': deque(maxlen=2000),
            'velocity_z': deque(maxlen=2000),
            'roll': deque(maxlen=2000),
            'pitch': deque(maxlen=2000),
            'yaw': deque(maxlen=2000)
        }
        
        # Previous state for numerical differentiation
        self.prev_velocity = np.zeros(3)
        self.prev_time = 0
        
        # Experiment control
        self.experiment_phase = 'startup'  # startup, arm, takeoff, hover, test_x, test_y, test_z, analyze
        self.experiment_start_time = None
        self.test_start_time = None
        self.current_test_axis = 0  # 0=x, 1=y, 2=z
        
        # Timer for main control loop
        self.timer = self.create_timer(0.02, self.control_loop)  # 50Hz for high-resolution data
        self.start_time = time.time()
        
        self.get_logger().info("🔬 PX4 Dynamics Analyzer Started!")
        self.get_logger().info("📊 Will perform step response tests on each axis")
        
    def vehicle_status_callback(self, msg):
        """Get vehicle arming status"""
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        
    def position_callback(self, msg):
        """Collect position and velocity data"""
        current_time = time.time()
        
        # Always record the time and command data first
        self.experiment_data['time'].append(current_time)
        
        # Check if position data is valid
        if msg.xy_valid and msg.z_valid and msg.v_xy_valid and msg.v_z_valid:
            # Record position and velocity data
            self.experiment_data['position_x'].append(msg.x)
            self.experiment_data['position_y'].append(msg.y)
            self.experiment_data['position_z'].append(msg.z)
            self.experiment_data['velocity_x'].append(msg.vx)
            self.experiment_data['velocity_y'].append(msg.vy)
            self.experiment_data['velocity_z'].append(msg.vz)
            
            # Calculate actual acceleration (numerical differentiation)
            if self.prev_time > 0:
                dt = current_time - self.prev_time
                if dt > 0 and dt < 0.1:  # Reasonable dt
                    current_velocity = np.array([msg.vx, msg.vy, msg.vz])
                    actual_accel = (current_velocity - self.prev_velocity) / dt
                    
                    self.experiment_data['actual_ax'].append(actual_accel[0])
                    self.experiment_data['actual_ay'].append(actual_accel[1])
                    self.experiment_data['actual_az'].append(actual_accel[2])
                else:
                    # Invalid dt, use zero acceleration
                    self.experiment_data['actual_ax'].append(0.0)
                    self.experiment_data['actual_ay'].append(0.0)
                    self.experiment_data['actual_az'].append(0.0)
            else:
                # First measurement
                self.experiment_data['actual_ax'].append(0.0)
                self.experiment_data['actual_ay'].append(0.0)
                self.experiment_data['actual_az'].append(0.0)
            
            self.prev_velocity = np.array([msg.vx, msg.vy, msg.vz])
            self.prev_time = current_time
            
            # Log data reception every 100 callbacks
            if hasattr(self, '_data_callback_count'):
                self._data_callback_count += 1
            else:
                self._data_callback_count = 1
                
            if self._data_callback_count % 100 == 0:
                self.get_logger().info(f"📊 Collecting data... {len(self.experiment_data['time'])} points, pos=({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f})")
                
        else:
            # Record invalid data as NaN for time alignment
            self.experiment_data['position_x'].append(float('nan'))
            self.experiment_data['position_y'].append(float('nan'))
            self.experiment_data['position_z'].append(float('nan'))
            self.experiment_data['velocity_x'].append(float('nan'))
            self.experiment_data['velocity_y'].append(float('nan'))
            self.experiment_data['velocity_z'].append(float('nan'))
            self.experiment_data['actual_ax'].append(float('nan'))
            self.experiment_data['actual_ay'].append(float('nan'))
            self.experiment_data['actual_az'].append(float('nan'))
            
            # Log invalid data occasionally
            if not hasattr(self, '_invalid_data_logged') or time.time() - self._invalid_data_logged > 5.0:
                self._invalid_data_logged = time.time()
                self.get_logger().warn(f"⚠️  Invalid position data: xy_valid={msg.xy_valid}, z_valid={msg.z_valid}, v_xy_valid={msg.v_xy_valid}, v_z_valid={msg.v_z_valid}")
                
        # Ensure command data arrays stay synchronized (pad with zeros if needed)
        while len(self.experiment_data['command_ax']) < len(self.experiment_data['time']):
            self.experiment_data['command_ax'].append(0.0)
            self.experiment_data['command_ay'].append(0.0)
            self.experiment_data['command_az'].append(0.0)
            
    def attitude_callback(self, msg):
        """Collect attitude data"""
        # Convert quaternion to Euler angles
        q = [msg.q[0], msg.q[1], msg.q[2], msg.q[3]]  # [w, x, y, z]
        
        # Convert to roll, pitch, yaw
        roll, pitch, yaw = self.quaternion_to_euler(q)
        
        if len(self.experiment_data['time']) > 0:  # Only record if we have time data
            self.experiment_data['roll'].append(roll)
            self.experiment_data['pitch'].append(pitch)
            self.experiment_data['yaw'].append(yaw)
            
    def quaternion_to_euler(self, q):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        w, x, y, z = q
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if np.abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)
            
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
        
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
        
    def publish_offboard_control_mode_acceleration(self):
        """Publish offboard control mode for acceleration control"""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = False
        msg.velocity = False
        msg.acceleration = True
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_pub.publish(msg)
        
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
        
    def publish_acceleration_setpoint(self, ax=0.0, ay=0.0, az=0.0, yaw=0.0):
        """Publish acceleration setpoint and record command"""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.acceleration = [ax, ay, az]
        msg.yaw = yaw
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        self.trajectory_setpoint_pub.publish(msg)
        
        # Record commanded acceleration
        if len(self.experiment_data['time']) > 0:  # Only record if we have time data
            self.experiment_data['command_ax'].append(ax)
            self.experiment_data['command_ay'].append(ay)
            self.experiment_data['command_az'].append(az)
            
    def generate_step_input(self, axis, test_time):
        """Generate step input for system identification"""
        step_amplitude = 1.0  # m/s² step
        step_duration = 2.0   # seconds
        
        if test_time < step_duration:
            # Step input
            if axis == 0:  # X axis
                return step_amplitude, 0.0, 0.0
            elif axis == 1:  # Y axis  
                return 0.0, step_amplitude, 0.0
            elif axis == 2:  # Z axis
                return 0.0, 0.0, step_amplitude
        else:
            # Return to zero
            return 0.0, 0.0, 0.0
            
    def save_experiment_data(self):
        """Save collected data for analysis"""
        if len(self.experiment_data['time']) == 0:
            self.get_logger().warn("⚠️  No data collected!")
            return None
            
        # Find the maximum length among all arrays
        max_length = max(len(values) for values in self.experiment_data.values())
        
        # Ensure all arrays have the same length by padding with NaN
        data_dict = {}
        for key, values in self.experiment_data.items():
            array = np.array(list(values))
            if len(array) < max_length:
                # Pad with NaN
                padded = np.full(max_length, np.nan)
                padded[:len(array)] = array
                data_dict[key] = padded
            else:
                data_dict[key] = array[:max_length]  # Truncate if too long
                
        # Report data quality
        time_points = len(data_dict['time'])
        valid_position_points = np.sum(~np.isnan(data_dict['position_x']))
        valid_command_points = np.sum(~np.isnan(data_dict['command_ax']))
        
        self.get_logger().info(f"� Data Summary:")
        self.get_logger().info(f"   Total time points: {time_points}")
        self.get_logger().info(f"   Valid position points: {valid_position_points} ({valid_position_points/time_points*100:.1f}%)")
        self.get_logger().info(f"   Valid command points: {valid_command_points} ({valid_command_points/time_points*100:.1f}%)")
        
        if valid_position_points < 10:
            self.get_logger().error("❌ Insufficient position data for analysis!")
            return None
            
        # Save as .npz file
        filename = f"/tmp/px4_dynamics_data_{int(time.time())}.npz"
        np.savez(filename, **data_dict)
        
        self.get_logger().info(f"💾 Data saved to: {filename}")
        
        return filename
            
    def control_loop(self):
        """Main control loop for dynamics identification experiments"""
        current_time = time.time() - self.start_time
        
        if self.experiment_phase == 'startup' and current_time < 2.0:
            self.publish_offboard_control_mode_position()
            self.publish_position_setpoint(0.0, 0.0, 0.0)
            
        elif self.experiment_phase == 'startup' and current_time >= 2.0:
            self.experiment_phase = 'arm'
            self.get_logger().info("🔫 Phase: Arming vehicle...")
            
        elif self.experiment_phase == 'arm' and current_time < 4.0:
            self.publish_offboard_control_mode_position()
            self.publish_position_setpoint(0.0, 0.0, 0.0)
            if not self.armed:
                self.arm_vehicle()
                
        elif self.experiment_phase == 'arm' and current_time >= 4.0:
            self.experiment_phase = 'takeoff'
            self.get_logger().info("🚁 Phase: Taking off...")
            
        elif self.experiment_phase == 'takeoff' and current_time < 8.0:
            self.switch_to_offboard()
            self.publish_offboard_control_mode_position()
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            
        elif self.experiment_phase == 'takeoff' and current_time >= 8.0:
            self.experiment_phase = 'hover'
            self.get_logger().info("🎯 Phase: Stabilizing hover...")
            
        elif self.experiment_phase == 'hover' and current_time < 12.0:
            self.publish_offboard_control_mode_position()
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            
        elif self.experiment_phase == 'hover' and current_time >= 12.0:
            self.experiment_phase = 'test_x'
            self.experiment_start_time = current_time
            self.test_start_time = current_time
            self.current_test_axis = 0
            self.get_logger().info("🔬 Phase: Starting X-axis step response test...")
            
        elif self.experiment_phase == 'test_x':
            test_time = current_time - self.test_start_time
            self.publish_offboard_control_mode_acceleration()
            
            ax, ay, az = self.generate_step_input(0, test_time)  # X-axis test
            self.publish_acceleration_setpoint(ax, ay, az)
            
            if test_time >= 4.0:  # 4 seconds for step + recovery
                self.experiment_phase = 'test_y'
                self.test_start_time = current_time
                self.current_test_axis = 1
                self.get_logger().info("🔬 Phase: Starting Y-axis step response test...")
                
        elif self.experiment_phase == 'test_y':
            test_time = current_time - self.test_start_time
            self.publish_offboard_control_mode_acceleration()
            
            ax, ay, az = self.generate_step_input(1, test_time)  # Y-axis test
            self.publish_acceleration_setpoint(ax, ay, az)
            
            if test_time >= 4.0:
                self.experiment_phase = 'test_z'
                self.test_start_time = current_time
                self.current_test_axis = 2
                self.get_logger().info("🔬 Phase: Starting Z-axis step response test...")
                
        elif self.experiment_phase == 'test_z':
            test_time = current_time - self.test_start_time
            self.publish_offboard_control_mode_acceleration()
            
            ax, ay, az = self.generate_step_input(2, test_time)  # Z-axis test
            self.publish_acceleration_setpoint(ax, ay, az)
            
            if test_time >= 4.0:
                self.experiment_phase = 'analyze'
                self.get_logger().info("📊 Phase: Experiment complete, analyzing data...")
                
        elif self.experiment_phase == 'analyze':
            # Save data and stop
            filename = self.save_experiment_data()
            self.get_logger().info("✅ Dynamics analysis complete!")
            self.get_logger().info(f"📈 Run data analysis with: python3 analyze_dynamics.py {filename}")
            
            # Return to hover for safety
            self.publish_offboard_control_mode_position()
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            
            # Stop experiment
            self.experiment_phase = 'done'


def main(args=None):
    print('🔬 Starting PX4 Dynamics Analyzer...')
    
    rclpy.init(args=args)
    analyzer = DynamicsAnalyzer()
    
    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        analyzer.get_logger().info('🛑 Dynamics analysis stopped by user')
        # Try to save data before exit
        try:
            analyzer.save_experiment_data()
        except:
            pass
    finally:
        analyzer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()