#!/usr/bin/env python3
"""
Efficient Data Collection MPC Controller
========================================
Phase 1: Pure data collection flight controller without GP overhead
- Runs standard MPC with excellent tracking
- Logs all state transitions for offline GP training
- No GP predictions or learning during flight
- Maximum flight performance and reliability

Usage:
    ros2 run px4_offboard data_collector_mpc
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleAttitudeSetpoint
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, VehicleAttitude, VehicleRatesSetpoint, VehicleAngularVelocity
from std_msgs.msg import Float64MultiArray, Float64, Header
from geometry_msgs.msg import PointStamped, Vector3Stamped, QuaternionStamped
import time
import math
import numpy as np
import casadi as ca
import os
import pickle
from datetime import datetime

from .trajectory_definitions import TrajectoryType, get_figure8_setpoint, get_circular_setpoint

class DataCollectorMPC(Node):
    """Pure MPC controller with data collection for offline GP training"""
    
    def __init__(self):
        super().__init__('data_collector_mpc')
        
        # Initialize data collection
        self.data_collection_active = True
        self.max_samples = 10000
        self.collected_data = []
        
        # Create data directory
        self.data_dir = "/home/grandediw/ros2_px4_offboard_example_ws/flight_data"
        os.makedirs(self.data_dir, exist_ok=True)
        
        # Flight parameters
        self.control_timer_period = 0.02  # 50Hz control
        self.trajectory_type = TrajectoryType.FIGURE8
        self.trajectory_scale = 3.0
        self.trajectory_speed = 0.5
        
        # Initialize state
        self.current_state = np.zeros(12)
        self.vehicle_status = None
        self.offboard_setpoint_counter = 0
        self.takeoff_height = 3.0
        
        # Flight phases
        self.flight_phase = "INIT"
        self.phase_start_time = None
        self.hover_start_time = None
        
        # MPC setup
        self.setup_mpc()
        
        # ROS2 setup
        self.setup_ros_interfaces()
        
        self.get_logger().info("✅ Data Collection MPC initialized")
        self.get_logger().info("📊 Ready to collect flight data for offline GP training")

    def setup_mpc(self):
        """Initialize MPC controller"""
        # MPC parameters
        self.N = 25  # prediction horizon steps
        self.dt = 0.02  # discretization time
        
        # State: [x, y, z, vx, vy, vz] - 6D double integrator
        # Control: [ax, ay, az, yaw_rate]
        self.nx = 6  # state dimension
        self.nu = 4  # control dimension
        
        # Weights
        self.Q_pos = np.diag([100, 100, 100])  # position weights
        self.Q_vel = np.diag([10, 10, 10])     # velocity weights
        self.R = np.diag([1, 1, 1, 10])       # control weights
        
        # Constraints
        self.pos_min = np.array([-10, -10, 0.5])
        self.pos_max = np.array([10, 10, 8])
        self.vel_max = np.array([5, 5, 3])
        self.acc_max = np.array([8, 8, 15])
        self.yaw_rate_max = 2.0
        
        self.build_mpc()
        
    def build_mpc(self):
        """Build the MPC optimization problem"""
        opti = ca.Opti()
        
        # Decision variables
        X = opti.variable(self.nx, self.N + 1)  # states
        U = opti.variable(self.nu, self.N)      # controls
        
        # Parameters
        X0 = opti.parameter(self.nx, 1)         # initial state
        X_ref = opti.parameter(self.nx, self.N + 1)  # reference trajectory
        
        # Objective
        J = 0
        for k in range(self.N):
            # State cost
            pos_error = X[:3, k] - X_ref[:3, k]
            vel_error = X[3:6, k] - X_ref[3:6, k]
            
            J += ca.mtimes([pos_error.T, self.Q_pos, pos_error])
            J += ca.mtimes([vel_error.T, self.Q_vel, vel_error])
            
            # Control cost
            J += ca.mtimes([U[:, k].T, self.R, U[:, k]])
            
        # Terminal cost
        pos_error_f = X[:3, -1] - X_ref[:3, -1]
        vel_error_f = X[3:6, -1] - X_ref[3:6, -1]
        J += 2 * ca.mtimes([pos_error_f.T, self.Q_pos, pos_error_f])
        J += 2 * ca.mtimes([vel_error_f.T, self.Q_vel, vel_error_f])
        
        opti.minimize(J)
        
        # Dynamics constraints (double integrator)
        for k in range(self.N):
            x_next = X[:3, k] + self.dt * X[3:6, k]
            v_next = X[3:6, k] + self.dt * U[:3, k]
            opti.subject_to(X[:3, k + 1] == x_next)
            opti.subject_to(X[3:6, k + 1] == v_next)
        
        # Initial condition
        opti.subject_to(X[:, 0] == X0)
        
        # State constraints
        for k in range(self.N + 1):
            opti.subject_to(opti.bounded(self.pos_min, X[:3, k], self.pos_max))
            opti.subject_to(opti.bounded(-self.vel_max, X[3:6, k], self.vel_max))
        
        # Control constraints
        for k in range(self.N):
            opti.subject_to(opti.bounded(-self.acc_max, U[:3, k], self.acc_max))
            opti.subject_to(opti.bounded(-self.yaw_rate_max, U[3, k], self.yaw_rate_max))
        
        # Solver settings
        opts = {
            'ipopt.print_level': 0,
            'ipopt.sb': 'yes',
            'print_time': 0
        }
        opti.solver('ipopt', opts)
        
        self.opti = opti
        self.X = X
        self.U = U
        self.X0 = X0
        self.X_ref = X_ref

    def collect_data_sample(self, state_prev, control, state_current, dt):
        """Collect a data sample for offline GP training"""
        if not self.data_collection_active or len(self.collected_data) >= self.max_samples:
            return
            
        # Store transition data
        sample = {
            'timestamp': time.time(),
            'state_prev': state_prev.copy(),
            'control': control.copy(), 
            'state_current': state_current.copy(),
            'dt': dt,
            'flight_phase': self.flight_phase
        }
        
        self.collected_data.append(sample)
        
        # Log progress
        if len(self.collected_data) % 100 == 0:
            fill_pct = (len(self.collected_data) / self.max_samples) * 100
            self.get_logger().info(f"📊 Data collection: {len(self.collected_data)} samples ({fill_pct:.1f}% full)")

    def save_collected_data(self):
        """Save collected flight data to file"""
        if not self.collected_data:
            self.get_logger().warn("⚠️  No data to save")
            return
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.data_dir}/flight_data_{timestamp}.pkl"
        
        # Create comprehensive dataset
        dataset = {
            'samples': self.collected_data,
            'metadata': {
                'total_samples': len(self.collected_data),
                'collection_time': timestamp,
                'trajectory_type': self.trajectory_type.name,
                'trajectory_scale': self.trajectory_scale,
                'trajectory_speed': self.trajectory_speed,
                'control_frequency': 1.0 / self.control_timer_period,
                'flight_duration': time.time() - self.collected_data[0]['timestamp'] if self.collected_data else 0
            }
        }
        
        with open(filename, 'wb') as f:
            pickle.dump(dataset, f)
            
        file_size = os.path.getsize(filename) / (1024 * 1024)  # MB
        self.get_logger().info(f"💾 Saved {len(self.collected_data)} samples to {filename}")
        self.get_logger().info(f"📊 Dataset size: {file_size:.1f} MB")
        self.get_logger().info(f"🧠 Ready for offline GP training!")
        
        return filename

    def solve_mpc(self, current_state, target_pos, target_yaw=0.0):
        """Solve MPC optimization problem"""
        # Extract position and velocity
        mpc_state = np.zeros(6)
        mpc_state[:3] = current_state[:3]  # position
        mpc_state[3:6] = current_state[6:9]  # velocity
        
        # Generate reference trajectory
        x_ref = np.zeros((6, self.N + 1))
        for k in range(self.N + 1):
            t_pred = k * self.dt
            if self.flight_phase == "MPC_TRAJECTORY":
                if self.trajectory_type == TrajectoryType.FIGURE8:
                    pos_ref, vel_ref = get_figure8_setpoint(
                        time.time() - self.phase_start_time + t_pred,
                        self.trajectory_scale, self.trajectory_speed, self.takeoff_height
                    )
                else:
                    pos_ref, vel_ref = get_circular_setpoint(
                        time.time() - self.phase_start_time + t_pred,
                        self.trajectory_scale, self.trajectory_speed, self.takeoff_height
                    )
                x_ref[:3, k] = pos_ref
                x_ref[3:6, k] = vel_ref
            else:
                x_ref[:3, k] = target_pos
                x_ref[3:6, k] = [0, 0, 0]
        
        # Set parameters
        self.opti.set_value(self.X0, mpc_state.reshape(-1, 1))
        self.opti.set_value(self.X_ref, x_ref)
        
        try:
            sol = self.opti.solve()
            u_opt = sol.value(self.U)
            return u_opt[:, 0]  # First control input
        except Exception as e:
            self.get_logger().warn(f"MPC solve failed: {e}")
            return np.array([0, 0, 9.81, 0])  # Hover command

    def setup_ros_interfaces(self):
        """Setup ROS2 publishers and subscribers"""
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_rates_setpoint_pub = self.create_publisher(VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos_profile)

        # Subscribers
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_local_position_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_attitude_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, qos_profile)

        # Control timer
        self.control_timer = self.create_timer(self.control_timer_period, self.control_loop)
        
        self.get_logger().info("🔗 ROS2 interfaces initialized")

    def vehicle_status_callback(self, msg):
        """Vehicle status callback"""
        self.vehicle_status = msg

    def vehicle_local_position_callback(self, msg):
        """Vehicle position callback"""
        # Update state: [x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz]
        self.current_state[:3] = [msg.x, msg.y, msg.z]
        self.current_state[6:9] = [msg.vx, msg.vy, msg.vz]

    def vehicle_attitude_callback(self, msg):
        """Vehicle attitude callback"""  
        # Convert quaternion to Euler angles
        q = [msg.q[0], msg.q[1], msg.q[2], msg.q[3]]  # w, x, y, z
        roll, pitch, yaw = self.quaternion_to_euler(q)
        self.current_state[3:6] = [roll, pitch, yaw]

    def quaternion_to_euler(self, q):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        w, x, y, z = q
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(max(-1, min(1, sinp)))
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw

    def control_loop(self):
        """Main control loop"""
        if self.vehicle_status is None:
            return

        current_time = time.time()
        
        # Phase management
        if self.flight_phase == "INIT":
            self.phase_start_time = current_time
            self.flight_phase = "TAKEOFF"
            self.get_logger().info("🚁 Phase 1: TAKEOFF")
            
        elif self.flight_phase == "TAKEOFF":
            if current_time - self.phase_start_time > 2.0:
                self.arm()
                self.flight_phase = "ARM_WAIT"
                self.phase_start_time = current_time
                self.get_logger().info("🔫 Phase 2: ARM_WAIT")
                
        elif self.flight_phase == "ARM_WAIT":
            if current_time - self.phase_start_time > 2.0:
                self.flight_phase = "CLIMB"
                self.phase_start_time = current_time
                self.get_logger().info(f"📈 Phase 3: CLIMB to {self.takeoff_height}m")
                
        elif self.flight_phase == "CLIMB":
            if current_time - self.phase_start_time > 8.0 or abs(self.current_state[2] - self.takeoff_height) < 0.5:
                self.flight_phase = "HOVER"
                self.hover_start_time = current_time
                self.get_logger().info("🎯 Phase 4: HOVER")
                
        elif self.flight_phase == "HOVER":
            if current_time - self.hover_start_time > 12.0:
                self.flight_phase = "MPC_TRAJECTORY" 
                self.phase_start_time = current_time
                self.get_logger().info("🧠 Phase 5: MPC TRAJECTORY with data collection")
                
        # Control logic
        if self.flight_phase in ["TAKEOFF", "ARM_WAIT", "CLIMB"]:
            self.publish_offboard_control_mode()
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height, 0.0)
            
        elif self.flight_phase == "HOVER":
            self.publish_offboard_control_mode()
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height, 0.0)
            
        elif self.flight_phase == "MPC_TRAJECTORY":
            # Store previous state for data collection
            state_prev = self.current_state.copy()
            
            # Get trajectory setpoint
            if self.trajectory_type == TrajectoryType.FIGURE8:
                target_pos, target_vel = get_figure8_setpoint(
                    current_time - self.phase_start_time,
                    self.trajectory_scale, self.trajectory_speed, self.takeoff_height
                )
            else:
                target_pos, target_vel = get_circular_setpoint(
                    current_time - self.phase_start_time,
                    self.trajectory_scale, self.trajectory_speed, self.takeoff_height
                )
                
            # Solve MPC
            control_output = self.solve_mpc(self.current_state, target_pos, 0.0)
            
            # Convert acceleration to body rates and publish
            self.publish_offboard_control_mode_rates()
            roll_rate, pitch_rate, yaw_rate, thrust = self.acceleration_to_body_rates(
                control_output[:3], control_output[3], self.current_state
            )
            self.publish_body_rates(roll_rate, pitch_rate, yaw_rate, thrust)
            
            # Data collection
            self.collect_data_sample(
                state_prev, control_output, self.current_state, self.control_timer_period
            )
            
            # Status logging
            if len(self.collected_data) % 50 == 0 and len(self.collected_data) > 0:
                pos_error = np.linalg.norm(np.array(target_pos) - self.current_state[:3])
                self.get_logger().info(
                    f"🧠 MPC Control | Pos error: {pos_error:.3f}m | "
                    f"Data: {len(self.collected_data)} samples"
                )

        self.offboard_setpoint_counter += 1

    def acceleration_to_body_rates(self, acc_cmd, yaw_rate_cmd, state):
        """Convert acceleration command to body rates using geometric control"""
        # Current attitude
        roll, pitch, yaw = state[3:6]
        
        # Desired acceleration (world frame)
        acc_des = np.array(acc_cmd)
        acc_des[2] += 9.81  # Add gravity compensation
        
        # Thrust magnitude
        thrust = np.linalg.norm(acc_des)
        thrust = max(0.1, min(thrust / 15.0, 1.0))  # Normalize to [0.1, 1.0]
        
        # Desired thrust direction
        if np.linalg.norm(acc_des) > 0:
            thrust_dir = acc_des / np.linalg.norm(acc_des)
        else:
            thrust_dir = np.array([0, 0, 1])
        
        # Desired roll and pitch
        roll_des = math.asin(max(-0.5, min(0.5, thrust_dir[1])))
        pitch_des = math.atan2(-thrust_dir[0], thrust_dir[2])
        
        # Rate commands (simple P control)
        kp_att = 3.0
        roll_rate = kp_att * (roll_des - roll)
        pitch_rate = kp_att * (pitch_des - pitch)
        yaw_rate = yaw_rate_cmd
        
        # Limit rates
        max_rate = 2.0  # rad/s
        roll_rate = max(-max_rate, min(max_rate, roll_rate))
        pitch_rate = max(-max_rate, min(max_rate, pitch_rate))
        yaw_rate = max(-max_rate, min(max_rate, yaw_rate))
        
        return roll_rate, pitch_rate, yaw_rate, thrust

    def arm(self):
        """Send arm command"""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = 1.0
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def publish_offboard_control_mode(self):
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
        """Publish offboard control mode for body rate control"""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = True
        self.offboard_control_mode_pub.publish(msg)

    def publish_position_setpoint(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        """Publish position setpoint"""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [x, y, -z]  # NED coordinates
        msg.yaw = yaw
        self.trajectory_setpoint_pub.publish(msg)

    def publish_body_rates(self, roll_rate, pitch_rate, yaw_rate, thrust):
        """Publish body rate commands"""
        msg = VehicleRatesSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.roll = roll_rate
        msg.pitch = pitch_rate
        msg.yaw = yaw_rate
        msg.thrust_body = [0.0, 0.0, -thrust]
        self.vehicle_rates_setpoint_pub.publish(msg)

    def cleanup(self):
        """Cleanup and save data"""
        self.get_logger().info("🧹 Shutting down data collection...")
        if self.collected_data:
            self.save_collected_data()
        self.data_collection_active = False

def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        controller = DataCollectorMPC()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("🛑 Interrupted by user")
    finally:
        if 'controller' in locals():
            controller.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()