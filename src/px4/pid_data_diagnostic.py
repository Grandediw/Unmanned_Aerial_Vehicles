#!/usr/bin/env python3
"""
PID Data Flow Diagnostic Tool
============================
This tool verifies that your PID controller is correctly reading output from PX4.
It monitors the same data streams your velocity_pid_demo uses and validates the data quality.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, TrajectorySetpoint, OffboardControlMode
import time
import math
import numpy as np

class PIDDataDiagnostic(Node):
    def __init__(self):
        super().__init__('pid_data_diagnostic')
        
        # Configure QoS to match PX4 (same as working demos)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers - exactly like your velocity_pid_demo
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
        
        # Data storage for analysis
        self.position_data = []
        self.velocity_data = []
        self.timestamps = []
        
        # Statistics
        self.status_count = 0
        self.position_count = 0
        self.start_time = time.time()
        self.last_position_time = 0
        self.last_velocity_time = 0
        
        # Validation flags
        self.armed = False
        self.position_valid = False
        self.velocity_valid = False
        
        # PID-relevant data
        self.current_position = [0.0, 0.0, 0.0]
        self.current_velocity = [0.0, 0.0, 0.0]
        self.last_current_position = [0.0, 0.0, 0.0]
        
        # Timer for periodic reports
        self.timer = self.create_timer(2.0, self.report_status)
        
        self.get_logger().info("🔍 PID Data Flow Diagnostic Started")
        self.get_logger().info("📊 Monitoring the same data streams your PID controller uses...")
        
    def vehicle_status_callback(self, msg):
        """Monitor vehicle status (same as your PID controller)"""
        self.status_count += 1
        prev_armed = self.armed
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        
        # Log arming state changes
        if prev_armed != self.armed:
            if self.armed:
                self.get_logger().info("✅ Vehicle ARMED - PID can now control")
            else:
                self.get_logger().info("❌ Vehicle DISARMED")
        
        # First status message
        if self.status_count == 1:
            self.get_logger().info("✅ First vehicle status received - Communication working!")
            
    def position_callback(self, msg):
        """Monitor position/velocity data (same as your PID controller)"""
        self.position_count += 1
        current_time = time.time()
        
        # Store previous position for rate calculation
        self.last_current_position = self.current_position.copy()
        
        # Update current state (exactly like your PID controller)
        self.current_position = [msg.x, msg.y, msg.z]
        self.current_velocity = [msg.vx, msg.vy, msg.vz]
        
        # Check data validity flags
        self.position_valid = hasattr(msg, 'xy_valid') and msg.xy_valid and hasattr(msg, 'z_valid') and msg.z_valid
        self.velocity_valid = hasattr(msg, 'v_xy_valid') and msg.v_xy_valid and hasattr(msg, 'v_z_valid') and msg.v_z_valid
        
        # Store data for analysis
        if len(self.timestamps) < 1000:  # Keep last 1000 samples
            self.timestamps.append(current_time)
            self.position_data.append(self.current_position.copy())
            self.velocity_data.append(self.current_velocity.copy())
        else:
            # Shift arrays
            self.timestamps.pop(0)
            self.position_data.pop(0)
            self.velocity_data.pop(0)
            self.timestamps.append(current_time)
            self.position_data.append(self.current_position.copy())
            self.velocity_data.append(self.current_velocity.copy())
        
        # Log first few messages with details
        if self.position_count <= 5:
            self.get_logger().info(f"📍 Position msg #{self.position_count}:")
            self.get_logger().info(f"   Position: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}) m")
            self.get_logger().info(f"   Velocity: ({msg.vx:.3f}, {msg.vy:.3f}, {msg.vz:.3f}) m/s")
            self.get_logger().info(f"   Valid flags: xy={getattr(msg, 'xy_valid', 'N/A')}, z={getattr(msg, 'z_valid', 'N/A')}, v_xy={getattr(msg, 'v_xy_valid', 'N/A')}, v_z={getattr(msg, 'v_z_valid', 'N/A')}")
            
        # Update timing
        self.last_position_time = current_time
        self.last_velocity_time = current_time
        
    def calculate_statistics(self):
        """Calculate data quality statistics"""
        if len(self.timestamps) < 10:
            return None
            
        # Convert to numpy arrays
        timestamps = np.array(self.timestamps)
        positions = np.array(self.position_data)
        velocities = np.array(self.velocity_data)
        
        # Calculate rates
        dt = np.diff(timestamps)
        avg_rate = 1.0 / np.mean(dt) if len(dt) > 0 else 0
        
        # Position statistics
        pos_std = np.std(positions, axis=0)
        pos_range = np.ptp(positions, axis=0)
        
        # Velocity statistics  
        vel_std = np.std(velocities, axis=0)
        vel_range = np.ptp(velocities, axis=0)
        
        # Check for reasonable data
        position_reasonable = all(abs(p) < 100 for p in self.current_position)  # Within 100m
        velocity_reasonable = all(abs(v) < 50 for v in self.current_velocity)   # Within 50 m/s
        
        # Calculate numerical derivatives for cross-check
        if len(positions) >= 3:
            pos_diff = np.diff(positions, axis=0)
            dt_diff = np.diff(timestamps)
            numerical_vel = pos_diff / dt_diff.reshape(-1, 1)
            
            # Compare with reported velocity (last few samples)
            if len(numerical_vel) > 0:
                recent_numerical_vel = numerical_vel[-1]
                recent_reported_vel = velocities[-1]
                vel_error = np.abs(recent_numerical_vel - recent_reported_vel)
                
                return {
                    'rate': avg_rate,
                    'pos_std': pos_std,
                    'pos_range': pos_range,
                    'vel_std': vel_std,
                    'vel_range': vel_range,
                    'position_reasonable': position_reasonable,
                    'velocity_reasonable': velocity_reasonable,
                    'vel_consistency': vel_error,
                    'samples': len(timestamps)
                }
        
        return {
            'rate': avg_rate,
            'pos_std': pos_std,
            'pos_range': pos_range,
            'vel_std': vel_std,
            'vel_range': vel_range,
            'position_reasonable': position_reasonable,
            'velocity_reasonable': velocity_reasonable,
            'samples': len(timestamps)
        }
        
    def report_status(self):
        """Report data flow status"""
        elapsed = time.time() - self.start_time
        
        self.get_logger().info(f"\n📊 PID Data Flow Report ({elapsed:.1f}s)")
        self.get_logger().info(f"   Status messages: {self.status_count}")
        self.get_logger().info(f"   Position messages: {self.position_count}")
        
        if self.position_count == 0:
            self.get_logger().warn("⚠️  NO POSITION DATA - Your PID controller won't work!")
            self.get_logger().warn("   Check: PX4 running? MicroXRCE agent running? EKF2 working?")
            return
            
        # Current state
        self.get_logger().info(f"   Armed: {self.armed}")
        self.get_logger().info(f"   Position valid: {self.position_valid}")
        self.get_logger().info(f"   Velocity valid: {self.velocity_valid}")
        
        # Current values (what your PID sees)
        self.get_logger().info(f"   Current position: ({self.current_position[0]:.3f}, {self.current_position[1]:.3f}, {self.current_position[2]:.3f}) m")
        self.get_logger().info(f"   Current velocity: ({self.current_velocity[0]:.3f}, {self.current_velocity[1]:.3f}, {self.current_velocity[2]:.3f}) m/s")
        
        # Data quality analysis
        stats = self.calculate_statistics()
        if stats:
            self.get_logger().info(f"   Data rate: {stats['rate']:.1f} Hz")
            self.get_logger().info(f"   Position noise (std): ({stats['pos_std'][0]:.4f}, {stats['pos_std'][1]:.4f}, {stats['pos_std'][2]:.4f}) m")
            self.get_logger().info(f"   Velocity noise (std): ({stats['vel_std'][0]:.4f}, {stats['vel_std'][1]:.4f}, {stats['vel_std'][2]:.4f}) m/s")
            
            # Data quality assessment
            if stats['rate'] > 30:
                self.get_logger().info("✅ Good data rate for PID control")
            elif stats['rate'] > 10:
                self.get_logger().warn("⚠️  Moderate data rate - PID might be sluggish")
            else:
                self.get_logger().error("❌ Low data rate - PID control will be poor")
                
            if stats['position_reasonable'] and stats['velocity_reasonable']:
                self.get_logger().info("✅ Data values look reasonable")
            else:
                self.get_logger().warn("⚠️  Data values look suspicious")
                
            # Velocity consistency check
            if 'vel_consistency' in stats:
                vel_error = stats['vel_consistency']
                if all(e < 0.5 for e in vel_error):
                    self.get_logger().info("✅ Velocity data is consistent with position derivatives")
                else:
                    self.get_logger().warn(f"⚠️  Velocity inconsistency: {vel_error}")
        
        # PID-specific recommendations
        if self.position_count > 0:
            if self.position_valid and self.velocity_valid:
                self.get_logger().info("✅ Your PID controller should work well with this data!")
            elif self.position_valid:
                self.get_logger().warn("⚠️  Position valid but velocity may be unreliable")
            else:
                self.get_logger().error("❌ Invalid position/velocity - PID will fail!")
                
        self.get_logger().info("")
        
def main():
    print('🔍 Starting PID Data Flow Diagnostic...')
    print('📊 This tool monitors the exact data your PID controller reads from PX4')
    print('⏰ Will run for 30 seconds, then provide summary...')
    
    rclpy.init()
    diagnostic = PIDDataDiagnostic()
    
    try:
        # Run for 30 seconds
        start_time = time.time()
        while time.time() - start_time < 30:
            rclpy.spin_once(diagnostic, timeout_sec=0.1)
            
        # Final summary
        print('\n' + '='*60)
        print('🎯 FINAL PID DATA FLOW ASSESSMENT')
        print('='*60)
        
        if diagnostic.position_count == 0:
            print('❌ CRITICAL: No position data received!')
            print('   Your PID controller cannot work without position/velocity feedback')
            print('   Check PX4 SITL, MicroXRCE agent, and sensor simulation')
        elif diagnostic.position_count < 100:
            print('⚠️  WARNING: Low data rate')
            print(f'   Only {diagnostic.position_count} messages in 30s ({diagnostic.position_count/30:.1f} Hz)')
            print('   PID control may be sluggish')
        else:
            print(f'✅ SUCCESS: Good data flow ({diagnostic.position_count} messages, {diagnostic.position_count/30:.1f} Hz)')
            
        if diagnostic.position_valid and diagnostic.velocity_valid:
            print('✅ Data quality: Valid position and velocity data')
        elif diagnostic.position_valid:
            print('⚠️  Data quality: Position valid, velocity may be unreliable')
        else:
            print('❌ Data quality: Invalid position/velocity flags')
            
        # Data ranges
        if len(diagnostic.position_data) > 0:
            pos_range = np.ptp(diagnostic.position_data, axis=0)
            vel_range = np.ptp(diagnostic.velocity_data, axis=0)
            print(f'📊 Position range: ({pos_range[0]:.3f}, {pos_range[1]:.3f}, {pos_range[2]:.3f}) m')
            print(f'📊 Velocity range: ({vel_range[0]:.3f}, {vel_range[1]:.3f}, {vel_range[2]:.3f}) m/s')
            
        print('\n💡 Recommendations:')
        if diagnostic.position_count > 100 and diagnostic.position_valid:
            print('   ✅ Your PID controller should receive good feedback from PX4')
            print('   ✅ Data flow is working correctly')
            print('   🎯 Your velocity_pid_demo should work as expected')
        else:
            print('   ❌ Fix PX4 data communication before expecting PID to work')
            print('   🔧 Check: PX4 SITL running? MicroXRCE agent connected? EKF2 healthy?')
            
    except KeyboardInterrupt:
        diagnostic.get_logger().info('🛑 Diagnostic stopped by user')
    finally:
        diagnostic.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()