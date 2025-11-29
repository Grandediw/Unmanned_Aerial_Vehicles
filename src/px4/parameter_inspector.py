#!/usr/bin/env python3
"""
PX4 Parameter Inspector
======================
Examines key PX4 parameters that affect vehicle dynamics and control response.
"""

import rclpy
from rclpy.node import Node
from px4_msgs.msg import ParameterUpdate
import time

class ParameterInspector(Node):
    def __init__(self):
        super().__init__('parameter_inspector')
        
        # Key parameters that affect dynamics
        self.key_params = {
            # Vehicle mass and inertia
            'MPC_T_MAX_GPS': 'Maximum thrust in GPS mode',
            'MPC_THR_HOVER': 'Hover throttle',
            'MPC_THR_MAX': 'Maximum throttle',
            'MPC_THR_MIN': 'Minimum throttle',
            
            # Position control
            'MPC_POS_MODE': 'Position control mode',
            'MPC_XY_P': 'Position P gain (X/Y)',
            'MPC_Z_P': 'Position P gain (Z)',
            'MPC_XY_VEL_P': 'Velocity P gain (X/Y)',
            'MPC_Z_VEL_P': 'Velocity P gain (Z)',
            'MPC_XY_VEL_I': 'Velocity I gain (X/Y)',
            'MPC_Z_VEL_I': 'Velocity I gain (Z)',
            'MPC_XY_VEL_D': 'Velocity D gain (X/Y)',
            'MPC_Z_VEL_D': 'Velocity D gain (Z)',
            
            # Acceleration limits
            'MPC_ACC_HOR_MAX': 'Max horizontal acceleration',
            'MPC_ACC_UP_MAX': 'Max upward acceleration',
            'MPC_ACC_DOWN_MAX': 'Max downward acceleration',
            
            # Velocity limits
            'MPC_XY_VEL_MAX': 'Max horizontal velocity',
            'MPC_Z_VEL_MAX_UP': 'Max upward velocity',
            'MPC_Z_VEL_MAX_DN': 'Max downward velocity',
            
            # Rate control (attitude)
            'MC_ROLLRATE_P': 'Roll rate P gain',
            'MC_PITCHRATE_P': 'Pitch rate P gain',
            'MC_YAWRATE_P': 'Yaw rate P gain',
            'MC_ROLLRATE_I': 'Roll rate I gain',
            'MC_PITCHRATE_I': 'Pitch rate I gain',
            'MC_YAWRATE_I': 'Yaw rate I gain',
            'MC_ROLLRATE_D': 'Roll rate D gain',
            'MC_PITCHRATE_D': 'Pitch rate D gain',
            'MC_YAWRATE_D': 'Yaw rate D gain',
            
            # Vehicle physical properties
            'SYS_VEHICLE_TYPE': 'Vehicle type',
            'SYS_AUTOSTART': 'Autostart configuration',
            
            # Estimator parameters
            'EKF2_GND_EFF_DZ': 'Ground effect dead zone',
            'EKF2_GND_MAX_HGT': 'Max height for ground effect',
        }
        
        self.get_logger().info("🔍 PX4 Parameter Inspector Started")
        self.get_logger().info("📋 Key parameters affecting dynamics:")
        
        for param, description in self.key_params.items():
            self.get_logger().info(f"   • {param}: {description}")
            
        self.get_logger().info("\n💡 To check actual parameter values, use:")
        self.get_logger().info("   param show <PARAMETER_NAME>")
        self.get_logger().info("   Example: param show MPC_XY_P")
        
        self.get_logger().info("\n🔧 Common dynamics tuning parameters:")
        self.get_logger().info("   • MPC_XY_P: Higher = more aggressive position tracking")
        self.get_logger().info("   • MPC_XY_VEL_P: Higher = more aggressive velocity tracking")
        self.get_logger().info("   • MPC_ACC_HOR_MAX: Limits maximum horizontal acceleration")
        self.get_logger().info("   • MPC_THR_HOVER: Affects thrust to acceleration mapping")
        
        # Start a timer to provide periodic reminders
        self.timer = self.create_timer(10.0, self.reminder_callback)
        
    def reminder_callback(self):
        """Provide periodic reminders"""
        self.get_logger().info("💭 Run dynamics_analyzer for ground truth system identification!")
        self.get_logger().info("🚁 Or connect to PX4 console: mavlink start -x -u 14556 -r 4000")

def main(args=None):
    rclpy.init(args=args)
    inspector = ParameterInspector()
    
    try:
        rclpy.spin(inspector)
    except KeyboardInterrupt:
        inspector.get_logger().info('🛑 Parameter inspector stopped')
    finally:
        inspector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()