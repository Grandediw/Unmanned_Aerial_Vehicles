#!/usr/bin/env python3
"""
PX4 EKF2 Position Diagnostic
===========================
Diagnose why position stays at (0,0,0) and provide fixes
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, SensorCombined
import time

class EKF2Diagnostic(Node):
    def __init__(self):
        super().__init__('ekf2_diagnostic')
        
        # Configure QoS to match PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers for all relevant data
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos_profile
        )
        self.vehicle_position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.position_callback, qos_profile
        )
        self.sensor_combined_sub = self.create_subscription(
            SensorCombined, '/fmu/out/sensor_combined', self.sensor_callback, qos_profile
        )
        # GPS subscription removed due to import issues
        
        # Data tracking
        self.status_count = 0
        self.position_count = 0
        self.sensor_count = 0
        # self.gps_count = 0  # Removed GPS tracking
        
        self.last_position = [0.0, 0.0, 0.0]
        self.position_valid = False
        self.sensor_data_valid = False
        # self.gps_valid = False  # Removed GPS tracking
        
        # Timer for periodic reports
        self.timer = self.create_timer(3.0, self.report_status)
        self.start_time = time.time()
        
        self.get_logger().info("🔍 EKF2 Position Diagnostic Started")
        self.get_logger().info("📊 Checking why position stays at (0,0,0)...")
        
    def status_callback(self, msg):
        self.status_count += 1
        if self.status_count == 1:
            self.get_logger().info("✅ Vehicle status: Connected")
            
    def position_callback(self, msg):
        self.position_count += 1
        self.last_position = [msg.x, msg.y, msg.z]
        
        # Check validity flags
        xy_valid = hasattr(msg, 'xy_valid') and msg.xy_valid
        z_valid = hasattr(msg, 'z_valid') and msg.z_valid
        self.position_valid = xy_valid and z_valid
        
        if self.position_count <= 3:
            self.get_logger().info(f"📍 Position #{self.position_count}: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})")
            self.get_logger().info(f"   Valid: xy={xy_valid}, z={z_valid}")
            self.get_logger().info(f"   Timestamp: {msg.timestamp}")
            
    def sensor_callback(self, msg):
        self.sensor_count += 1
        
        # Check if we have IMU data
        accel_valid = hasattr(msg, 'accelerometer_m_s2') and len(msg.accelerometer_m_s2) >= 3
        gyro_valid = hasattr(msg, 'gyro_rad') and len(msg.gyro_rad) >= 3
        
        self.sensor_data_valid = accel_valid and gyro_valid
        
        if self.sensor_count <= 2:
            self.get_logger().info(f"📊 Sensor #{self.sensor_count}:")
            if accel_valid:
                accel = msg.accelerometer_m_s2
                self.get_logger().info(f"   Accel: ({accel[0]:.2f}, {accel[1]:.2f}, {accel[2]:.2f}) m/s²")
            if gyro_valid:
                gyro = msg.gyro_rad
                self.get_logger().info(f"   Gyro: ({gyro[0]:.2f}, {gyro[1]:.2f}, {gyro[2]:.2f}) rad/s")
                
    def report_status(self):
        elapsed = time.time() - self.start_time
        
        self.get_logger().info(f"\n🔍 EKF2 Diagnostic Report ({elapsed:.1f}s)")
        self.get_logger().info(f"   Status msgs: {self.status_count}")
        self.get_logger().info(f"   Position msgs: {self.position_count}")
        self.get_logger().info(f"   Sensor msgs: {self.sensor_count}")
        # self.get_logger().info(f"   GPS msgs: {self.gps_count}")  # Removed GPS
        
        # Position analysis
        if self.position_count > 0:
            pos_x, pos_y, pos_z = self.last_position
            if abs(pos_x) < 0.001 and abs(pos_y) < 0.001 and abs(pos_z) < 0.001:
                self.get_logger().error("❌ PROBLEM: Position stuck at (0,0,0)")
                
                # Diagnose the cause
                if not self.position_valid:
                    self.get_logger().error("   Cause: Position validity flags are FALSE")
                    self.get_logger().info("   💡 This means EKF2 hasn't initialized properly")
                    
                if self.sensor_count == 0:
                    self.get_logger().error("   Cause: No sensor data (IMU)")
                    self.get_logger().info("   💡 PX4 SITL sensor simulation not working")
                    
                # Removed GPS checks since module unavailable
                    
            else:
                self.get_logger().info(f"✅ Position changing: ({pos_x:.3f}, {pos_y:.3f}, {pos_z:.3f})")
                
        else:
            self.get_logger().error("❌ No position messages received")
            
        # Provide specific fixes
        self.provide_fixes()
        
    def provide_fixes(self):
        """Provide specific fixes based on the diagnosis"""
        self.get_logger().info("\n🔧 FIXES:")
        
        if self.position_count == 0:
            self.get_logger().info("1. Check PX4 SITL is running: 'px4-commander status'")
            self.get_logger().info("2. Check MicroXRCE agent: 'netstat -an | grep 8888'")
            
        elif not self.position_valid:
            self.get_logger().info("1. Force EKF2 reset: Send 'commander ekf reset' to PX4")
            self.get_logger().info("2. Check EKF2 status: 'listener ekf2_timestamps' in PX4 console")
            self.get_logger().info("3. Restart with proper sensor simulation")
            
        if self.sensor_count == 0:
            self.get_logger().info("4. Restart PX4 SITL with: 'make px4_sitl gz_x500'")
            self.get_logger().info("5. Ensure Gazebo is properly loading the drone model")
            
        # Removed GPS fix suggestions
            
        self.get_logger().info("\n🚀 Quick fix command sequence:")
        self.get_logger().info("   1. pkill -f px4")
        self.get_logger().info("   2. pkill -f gz")
        self.get_logger().info("   3. cd ~/PX4-Autopilot")
        self.get_logger().info("   4. make px4_sitl gz_x500")
        self.get_logger().info("   5. Wait for 'EKF2 reset' message in console")
        

def main():
    print('🔍 Starting EKF2 Position Diagnostic...')
    print('📊 This will identify why position stays at (0,0,0)')
    
    rclpy.init()
    diagnostic = EKF2Diagnostic()
    
    try:
        # Run for 15 seconds
        start_time = time.time()
        while time.time() - start_time < 15:
            rclpy.spin_once(diagnostic, timeout_sec=0.1)
            
        # Final summary
        print('\n' + '='*60)
        print('🎯 FINAL EKF2 DIAGNOSIS')
        print('='*60)
        
        pos_stuck = all(abs(p) < 0.001 for p in diagnostic.last_position)
        
        if diagnostic.position_count == 0:
            print('❌ CRITICAL: No position data at all')
            print('   Fix: Check PX4 SITL and MicroXRCE agent')
            
        elif pos_stuck and not diagnostic.position_valid:
            print('❌ CRITICAL: EKF2 not initialized (validity flags false)')
            print('   Fix: EKF2 needs proper sensor data to initialize')
            print('   Solution: Restart PX4 SITL with proper Gazebo simulation')
            
        elif pos_stuck and diagnostic.sensor_count == 0:
            print('❌ CRITICAL: No sensor data (IMU missing)')
            print('   Fix: Gazebo sensor simulation not working')
            print('   Solution: Restart with make px4_sitl gz_x500')
            
        elif pos_stuck:
            print('⚠️  WARNING: Position stuck despite data available')
            print('   Fix: Try EKF2 reset command in PX4 console')
            
        else:
            print('✅ SUCCESS: Position data looks good!')
            print(f'   Position: ({diagnostic.last_position[0]:.3f}, {diagnostic.last_position[1]:.3f}, {diagnostic.last_position[2]:.3f})')
            
        print('\n💡 For your PID controller:')
        if not pos_stuck:
            print('   ✅ Your velocity_pid_demo should work correctly')
        else:
            print('   ❌ Your velocity_pid_demo will see zero feedback')
            print('   🔧 Fix the EKF2 issue first, then retry PID control')
            
    except KeyboardInterrupt:
        diagnostic.get_logger().info('🛑 Diagnostic stopped')
    finally:
        diagnostic.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()