#!/usr/bin/env python3
"""
Simple Dynamics Data Collector
===============================
Uses the working MPC demo to collect acceleration command vs position response data
for system identification. This bypasses the sensor issues in Gazebo.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition
import time
import math
import numpy as np
from collections import deque

class SimpleDynamicsCollector(Node):
    def __init__(self):
        super().__init__('simple_dynamics_collector')
        
        # Configure QoS to match PX4 exactly
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.get_logger().info(f"🔧 QoS Profile: {qos_profile.reliability.name}, {qos_profile.durability.name}")
        
        # Publishers (use default QoS for commands)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        
        # Subscribers (use PX4-compatible QoS)
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
        
        # State variables
        self.armed = False
        self.takeoff_height = 2.0
        
        # Data collection
        self.data = {
            'time': [],
            'cmd_ax': [],
            'cmd_ay': [], 
            'cmd_az': [],
            'pos_x': [],
            'pos_y': [],
            'pos_z': [],
            'vel_x': [],
            'vel_y': [],
            'vel_z': []
        }
        
        # Previous state for acceleration calculation
        self.prev_vel = np.zeros(3)
        self.prev_time = 0
        
        # Test phase
        self.phase = 'startup'
        self.test_start = None
        self.data_collection_active = False
        
        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)
        self.start_time = time.time()
        
        self.get_logger().info("📊 Simple Dynamics Data Collector Started!")
        self.get_logger().info("🎯 Will use simulated state for robust data collection")
        self.get_logger().info("🔗 QoS configured for PX4 compatibility (BEST_EFFORT)")
        
    def vehicle_status_callback(self, msg):
        prev_armed = self.armed
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        
        # Log arming state changes
        if prev_armed != self.armed:
            if self.armed:
                self.get_logger().info("🔫 Vehicle ARMED")
            else:
                self.get_logger().info("🔓 Vehicle DISARMED")
        
        # Log navigation state occasionally
        if hasattr(self, '_last_nav_state_log') and time.time() - self._last_nav_state_log < 5.0:
            return
        self._last_nav_state_log = time.time()
        
        nav_state_names = {
            0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION",
            4: "AUTO_LOITER", 5: "AUTO_RTL", 6: "ACRO", 7: "UNUSED",
            8: "DESCEND", 9: "TERMINATION", 10: "OFFBOARD", 11: "STAB",
            12: "UNUSED2", 13: "UNUSED3", 14: "AUTO_TAKEOFF", 15: "AUTO_LAND",
            16: "AUTO_FOLLOW_TARGET", 17: "AUTO_PRECLAND", 18: "ORBIT"
        }
        nav_state = nav_state_names.get(msg.nav_state, f"UNKNOWN({msg.nav_state})")
        self.get_logger().info(f"🧭 Navigation state: {nav_state}, Armed: {self.armed}")
        
    def position_callback(self, msg):
        """Collect position data when available"""
        if not self.data_collection_active:
            return
            
        current_time = time.time()
        
        # Always record time for synchronization
        self.data['time'].append(current_time)
        
        # Check data validity and use real position data if available
        pos_valid = hasattr(msg, 'xy_valid') and msg.xy_valid and hasattr(msg, 'z_valid') and msg.z_valid
        vel_valid = hasattr(msg, 'v_xy_valid') and msg.v_xy_valid and hasattr(msg, 'v_z_valid') and msg.v_z_valid
        
        if pos_valid:
            self.data['pos_x'].append(msg.x)
            self.data['pos_y'].append(msg.y) 
            self.data['pos_z'].append(msg.z)
            
            if vel_valid:
                self.data['vel_x'].append(msg.vx)
                self.data['vel_y'].append(msg.vy)
                self.data['vel_z'].append(msg.vz)
            else:
                self.data['vel_x'].append(float('nan'))
                self.data['vel_y'].append(float('nan'))
                self.data['vel_z'].append(float('nan'))
                
            # Log successful data reception occasionally
            if len(self.data['time']) % 100 == 1:  # First successful and every 100th
                self.get_logger().info(f"📍 Position data: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}")
        else:
            # No valid position data - use NaN placeholders but log the attempt
            self.data['pos_x'].append(float('nan'))
            self.data['pos_y'].append(float('nan'))
            self.data['pos_z'].append(float('nan'))
            self.data['vel_x'].append(float('nan'))
            self.data['vel_y'].append(float('nan'))
            self.data['vel_z'].append(float('nan'))
            
            # Log data validity status occasionally
            if len(self.data['time']) % 100 == 1:
                self.get_logger().warn(f"⚠️  Invalid position data: xy_valid={getattr(msg, 'xy_valid', 'missing')}, z_valid={getattr(msg, 'z_valid', 'missing')}")
            
        # Log data collection progress
        if len(self.data['time']) % 50 == 0:
            valid_pos = sum(1 for x in self.data['pos_x'] if not np.isnan(x))
            total_points = len(self.data['time'])
            self.get_logger().info(f"📊 Collecting data: {total_points} points, {valid_pos} valid positions ({valid_pos/total_points*100:.1f}%)")
        
    def arm_vehicle(self):
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
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_pub.publish(msg)
        
    def publish_offboard_control_mode_acceleration(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = False
        msg.velocity = False
        msg.acceleration = True
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_pub.publish(msg)
        
    def publish_position_setpoint(self, x=0.0, y=0.0, z=0.0):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [x, y, -z]
        self.trajectory_setpoint_pub.publish(msg)
        
    def publish_acceleration_setpoint(self, ax=0.0, ay=0.0, az=0.0):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.acceleration = [ax, ay, az]
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        self.trajectory_setpoint_pub.publish(msg)
        
        # Record commanded acceleration
        if self.data_collection_active:
            self.data['cmd_ax'].append(ax)
            self.data['cmd_ay'].append(ay)
            self.data['cmd_az'].append(az)
        
    def generate_test_sequence(self, test_time):
        """Generate systematic test inputs for dynamics identification"""
        
        # Phase 1: X-axis step (0-3s)
        if test_time < 3.0:
            return 1.0, 0.0, 0.0
        # Phase 2: Rest (3-4s)
        elif test_time < 4.0:
            return 0.0, 0.0, 0.0
        # Phase 3: Y-axis step (4-7s)  
        elif test_time < 7.0:
            return 0.0, 1.0, 0.0
        # Phase 4: Rest (7-8s)
        elif test_time < 8.0:
            return 0.0, 0.0, 0.0
        # Phase 5: Z-axis step (8-11s)
        elif test_time < 11.0:
            return 0.0, 0.0, 0.5
        # Phase 6: Rest (11-12s)
        elif test_time < 12.0:
            return 0.0, 0.0, 0.0
        # Phase 7: Combined test (12-15s)
        elif test_time < 15.0:
            return 0.5, 0.5, 0.0
        else:
            return 0.0, 0.0, 0.0
        
    def save_data(self):
        """Save collected data"""
        if len(self.data['time']) == 0:
            self.get_logger().warn("⚠️  No data collected!")
            return None
            
        # Ensure all arrays have same length
        max_len = max(len(v) for v in self.data.values())
        
        for key in self.data:
            while len(self.data[key]) < max_len:
                self.data[key].append(float('nan'))
                
        # Convert to numpy and save
        data_dict = {k: np.array(v) for k, v in self.data.items()}
        
        filename = f"/tmp/simple_dynamics_data_{int(time.time())}.npz"
        np.savez(filename, **data_dict)
        
        valid_points = sum(1 for x in data_dict['pos_x'] if not np.isnan(x))
        total_points = len(data_dict['time'])
        
        self.get_logger().info(f"💾 Data saved: {filename}")
        self.get_logger().info(f"📊 Total points: {total_points}, Valid: {valid_points} ({valid_points/total_points*100:.1f}%)")
        
        return filename
        
    def control_loop(self):
        current_time = time.time() - self.start_time
        
        if self.phase == 'startup' and current_time < 2.0:
            self.publish_offboard_control_mode_position()
            self.publish_position_setpoint(0.0, 0.0, 0.0)
            
        elif self.phase == 'startup' and current_time >= 2.0:
            self.phase = 'arm'
            self.get_logger().info("🔫 Phase: Arming...")
            
        elif self.phase == 'arm' and current_time < 4.0:
            self.publish_offboard_control_mode_position()
            self.publish_position_setpoint(0.0, 0.0, 0.0)
            if not self.armed:
                self.arm_vehicle()
                
        elif self.phase == 'arm' and current_time >= 4.0:
            self.phase = 'takeoff'
            self.get_logger().info("🚁 Phase: Taking off...")
            
        elif self.phase == 'takeoff' and current_time < 8.0:
            self.switch_to_offboard()
            self.publish_offboard_control_mode_position()
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            
        elif self.phase == 'takeoff' and current_time >= 8.0:
            self.phase = 'hover'
            self.get_logger().info("🎯 Phase: Stabilizing...")
            
        elif self.phase == 'hover' and current_time < 12.0:
            self.publish_offboard_control_mode_position()
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            
        elif self.phase == 'hover' and current_time >= 12.0:
            self.phase = 'test'
            self.test_start = current_time
            self.data_collection_active = True
            self.get_logger().info("🔬 Phase: Starting dynamics test...")
            
        elif self.phase == 'test':
            test_time = current_time - self.test_start
            
            if test_time < 20.0:  # 20 second test
                self.publish_offboard_control_mode_acceleration()
                ax, ay, az = self.generate_test_sequence(test_time)
                self.publish_acceleration_setpoint(ax, ay, az)
            else:
                self.phase = 'complete'
                self.data_collection_active = False
                self.get_logger().info("📊 Phase: Test complete!")
                
        elif self.phase == 'complete':
            # Return to hover and save data
            self.publish_offboard_control_mode_position()
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            
            if not hasattr(self, 'data_saved'):
                self.save_data()
                self.data_saved = True
                

def main(args=None):
    print('📊 Starting Simple Dynamics Data Collector...')
    
    rclpy.init(args=args)
    collector = SimpleDynamicsCollector()
    
    try:
        rclpy.spin(collector)
    except KeyboardInterrupt:
        collector.get_logger().info('🛑 Data collection stopped')
        try:
            collector.save_data()
        except:
            pass
    finally:
        collector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()