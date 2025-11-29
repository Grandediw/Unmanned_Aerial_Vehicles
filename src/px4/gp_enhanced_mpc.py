#!/usr/bin/env python3
"""
GP-Enhanced MPC Controller - Phase 3
====================================
Phase 3: Optional GP-enhanced MPC with learned dynamics
- Uses baseline MPC as fallback (always works)
- Optionally loads trained GP models from Phase 2
- User can enable/disable GP enhancement via parameters
- Graceful fallback if GP models unavailable or fail

Usage:
    # Run with GP enhancement
    ros2 run px4_offboard gp_enhanced_mpc --ros-args -p use_gp:=true -p gp_model_file:=/path/to/models.pkl
    
    # Run baseline MPC (no GP)
    ros2 run px4_offboard gp_enhanced_mpc --ros-args -p use_gp:=false
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleRatesSetpoint
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, VehicleAttitude
import time
import math
import numpy as np
import casadi as ca
import os
import pickle
import warnings
warnings.filterwarnings('ignore')

from .trajectory_definitions import TrajectoryType, get_figure8_setpoint, get_circular_setpoint

class GPEnhancedMPC(Node):
    """MPC controller with optional GP enhancement"""
    
    def __init__(self):
        super().__init__('gp_enhanced_mpc')
        
        # Parameters
        self.declare_parameter('use_gp', False)
        self.declare_parameter('gp_model_file', '')
        self.declare_parameter('trajectory_type', 'figure8')
        self.declare_parameter('trajectory_scale', 3.0)
        self.declare_parameter('trajectory_speed', 0.5)
        
        # Get parameters
        self.use_gp = self.get_parameter('use_gp').value
        self.gp_model_file = self.get_parameter('gp_model_file').value
        self.trajectory_scale = self.get_parameter('trajectory_scale').value  
        self.trajectory_speed = self.get_parameter('trajectory_speed').value
        
        traj_type_str = self.get_parameter('trajectory_type').value
        self.trajectory_type = TrajectoryType.FIGURE8 if traj_type_str == 'figure8' else TrajectoryType.CIRCULAR
        
        # Flight parameters
        self.control_timer_period = 0.02  # 50Hz control
        self.takeoff_height = 3.0
        
        # Initialize state
        self.current_state = np.zeros(12)
        self.vehicle_status = None
        self.offboard_setpoint_counter = 0
        
        # Flight phases
        self.flight_phase = "INIT"
        self.phase_start_time = None
        self.hover_start_time = None
        
        # GP initialization
        self.gp_models = None
        self.gp_scalers_input = None
        self.gp_scalers_output = None
        self.gp_available = False
        self.gp_predictions_enabled = False
        
        # Performance tracking
        self.mpc_solve_times = []
        self.gp_prediction_times = []
        self.tracking_errors = []
        
        # Setup
        self.setup_mpc()
        self.setup_gp()
        self.setup_ros_interfaces()
        
        # Status logging
        if self.gp_available and self.use_gp:
            self.get_logger().info("🧠 GP-Enhanced MPC initialized with learned dynamics")
        else:
            self.get_logger().info("🎯 Baseline MPC initialized (no GP enhancement)")

    def setup_gp(self):
        """Initialize GP models if requested and available"""
        if not self.use_gp:
            self.get_logger().info("🎯 GP enhancement disabled by parameter")
            return
            
        if not self.gp_model_file:
            # Try to find latest model file
            model_dir = "/home/grandediw/ros2_px4_offboard_example_ws/gp_models"
            if os.path.exists(model_dir):
                model_files = [f for f in os.listdir(model_dir) if f.startswith('gp_models_') and f.endswith('.pkl')]
                if model_files:
                    self.gp_model_file = os.path.join(model_dir, sorted(model_files)[-1])
                    self.get_logger().info(f"🔍 Auto-detected GP model: {os.path.basename(self.gp_model_file)}")
        
        if not self.gp_model_file or not os.path.exists(self.gp_model_file):
            self.get_logger().warn("⚠️  No GP model file found - running baseline MPC")
            return
        
        try:
            self.get_logger().info(f"🧠 Loading GP models from: {os.path.basename(self.gp_model_file)}")
            
            with open(self.gp_model_file, 'rb') as f:
                model_package = pickle.load(f)
            
            self.gp_models = model_package['models']
            self.gp_scalers_input = model_package['scalers_input']
            self.gp_scalers_output = model_package['scalers_output']
            
            # Validate models
            required_models = ['pos_x', 'pos_y', 'pos_z', 'vel_vx', 'vel_vy', 'vel_vz']
            if all(model in self.gp_models for model in required_models):
                self.gp_available = True
                self.gp_predictions_enabled = True
                self.get_logger().info("✅ GP models loaded and validated")
                
                # Log training stats
                stats = model_package.get('training_stats', {})
                if stats:
                    avg_r2 = np.mean([s['r2'] for s in stats.values()])
                    self.get_logger().info(f"📊 Model quality: R²={avg_r2:.3f}")
            else:
                raise ValueError("Incomplete GP model set")
                
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load GP models: {e}")
            self.get_logger().warn("⚠️  Falling back to baseline MPC")

    def predict_gp_residuals(self, state, control):
        """Predict dynamics residuals using GP models"""
        if not self.gp_available or not self.gp_predictions_enabled:
            return np.zeros(3), np.zeros(3)  # No GP correction
        
        try:
            # Prepare input: [pos(3), vel(3), control(4)]
            mpc_state = np.concatenate([state[:3], state[6:9]])  # pos + vel
            input_vec = np.concatenate([mpc_state, control])
            
            # Scale input (use first model's scaler since they're shared)
            input_scaled = self.gp_scalers_input['pos_x'].transform(input_vec.reshape(1, -1))
            
            # Predict residuals
            pos_residuals = np.zeros(3)
            vel_residuals = np.zeros(3)
            
            # Position residuals
            for i, axis in enumerate(['x', 'y', 'z']):
                model_key = f'pos_{axis}'
                gp_model = self.gp_models[model_key]
                scaler_output = self.gp_scalers_output[model_key]
                
                pred_scaled = gp_model.predict(input_scaled, return_std=False)
                pred = scaler_output.inverse_transform(pred_scaled.reshape(-1, 1)).ravel()[0]
                pos_residuals[i] = pred
            
            # Velocity residuals
            for i, axis in enumerate(['vx', 'vy', 'vz']):
                model_key = f'vel_{axis}'
                gp_model = self.gp_models[model_key]
                scaler_output = self.gp_scalers_output[model_key]
                
                pred_scaled = gp_model.predict(input_scaled, return_std=False)
                pred = scaler_output.inverse_transform(pred_scaled.reshape(-1, 1)).ravel()[0]
                vel_residuals[i] = pred
            
            return pos_residuals, vel_residuals
            
        except Exception as e:
            self.get_logger().warn(f"GP prediction failed: {e}")
            return np.zeros(3), np.zeros(3)

    def setup_mpc(self):
        """Initialize MPC controller"""
        # MPC parameters
        self.N = 25  # prediction horizon steps
        self.dt = 0.02  # discretization time
        
        # State: [x, y, z, vx, vy, vz] - 6D
        # Control: [ax, ay, az, yaw_rate]
        self.nx = 6  # state dimension
        self.nu = 4  # control dimension
        
        # Weights (slightly higher for GP-enhanced version)
        pos_weight = 120 if self.use_gp else 100
        vel_weight = 12 if self.use_gp else 10
        
        self.Q_pos = np.diag([pos_weight, pos_weight, pos_weight])
        self.Q_vel = np.diag([vel_weight, vel_weight, vel_weight])
        self.R = np.diag([1, 1, 1, 10])
        
        # Constraints
        self.pos_min = np.array([-10, -10, 0.5])
        self.pos_max = np.array([10, 10, 8])
        self.vel_max = np.array([5, 5, 3])
        self.acc_max = np.array([8, 8, 15])
        self.yaw_rate_max = 2.0
        
        self.build_mpc()
        
    def build_mpc(self):
        """Build MPC optimization problem with optional GP dynamics"""
        opti = ca.Opti()
        
        # Decision variables
        X = opti.variable(self.nx, self.N + 1)  # states
        U = opti.variable(self.nu, self.N)      # controls
        
        # Parameters
        X0 = opti.parameter(self.nx, 1)         # initial state
        X_ref = opti.parameter(self.nx, self.N + 1)  # reference trajectory
        
        # GP residual parameters (for GP-enhanced version)
        if self.use_gp:
            GP_pos = opti.parameter(3, self.N)  # position residuals
            GP_vel = opti.parameter(3, self.N)  # velocity residuals
        
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
        
        # Dynamics constraints
        for k in range(self.N):
            # Nominal dynamics
            x_next_nominal = X[:3, k] + self.dt * X[3:6, k]
            v_next_nominal = X[3:6, k] + self.dt * U[:3, k]
            
            if self.use_gp:
                # GP-enhanced dynamics
                x_next = x_next_nominal + self.dt * GP_pos[:, k]
                v_next = v_next_nominal + self.dt * GP_vel[:, k] 
            else:
                # Baseline dynamics
                x_next = x_next_nominal
                v_next = v_next_nominal
            
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
        
        if self.use_gp:
            self.GP_pos = GP_pos
            self.GP_vel = GP_vel

    # def solve_mpc(self, current_state, target_pos, target_yaw=0.0):
    #     """Solve MPC optimization with optional GP enhancement"""
    #     solve_start = time.time()
        
    #     # Extract MPC state
    #     mpc_state = np.zeros(6)
    #     mpc_state[:3] = current_state[:3]  # position
    #     mpc_state[3:6] = current_state[6:9]  # velocity
        
    #     # Generate reference trajectory
    #     x_ref = np.zeros((6, self.N + 1))
    #     for k in range(self.N + 1):
    #         t_pred = k * self.dt
    #         if self.flight_phase == "MPC_TRAJECTORY":
    #             if self.trajectory_type == TrajectoryType.FIGURE8:
    #                 pos_ref, vel_ref = get_figure8_setpoint(
    #                     time.time() - self.phase_start_time + t_pred,
    #                     self.trajectory_scale, self.trajectory_speed, self.takeoff_height
    #                 )
    #             else:
    #                 pos_ref, vel_ref = get_circular_setpoint(
    #                     time.time() - self.phase_start_time + t_pred,
    #                     self.trajectory_scale, self.trajectory_speed, self.takeoff_height
    #                 )
    #             x_ref[:3, k] = pos_ref
    #             x_ref[3:6, k] = vel_ref
    #         else:
    #             x_ref[:3, k] = target_pos
    #             x_ref[3:6, k] = [0, 0, 0]
        
    #     # Set basic parameters
    #     self.opti.set_value(self.X0, mpc_state.reshape(-1, 1))
    #     self.opti.set_value(self.X_ref, x_ref)
        
    #     # GP predictions for enhanced dynamics
    #     if self.use_gp and self.gp_available:
    #         gp_start = time.time()
            
    #         # Predict GP residuals for each horizon step
    #         gp_pos_corrections = np.zeros((3, self.N))
    #         gp_vel_corrections = np.zeros((3, self.N))
            
    #         # Use current state and rough control estimate
    #         for k in range(self.N):
    #             # Simple control estimate (can be improved with iterative refinement)
    #             rough_control = np.array([0, 0, 9.81, 0])  # Hover as initial guess
                
    #             pos_res, vel_res = self.predict_gp_residuals(current_state, rough_control)
    #             gp_pos_corrections[:, k] = pos_res
    #             gp_vel_corrections[:, k] = vel_res
            
    #         self.opti.set_value(self.GP_pos, gp_pos_corrections)
    #         self.opti.set_value(self.GP_vel, gp_vel_corrections)
            
    #         gp_time = time.time() - gp_start
    #         self.gp_prediction_times.append(gp_time)
        
    #     # Solve MPC
    #     try:
    #         sol = self.opti.solve()
    #         u_opt = sol.value(self.U)
    #         solve_time = time.time() - solve_start
    #         self.mpc_solve_times.append(solve_time)
    #         return u_opt[:, 0]  # First control input
            
    #     except Exception as e:
    #         self.get_logger().warn(f"MPC solve failed: {e}")
    #         return np.array([0, 0, 9.81, 0])  # Hover command
    def solve_mpc(self, current_state, target_pos, target_yaw=0.0):
        """Solve MPC optimization with optional GP enhancement"""
        solve_start = time.time()
        
        # Extract MPC state
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
        
        # Set basic parameters
        self.opti.set_value(self.X0, mpc_state.reshape(-1, 1))
        self.opti.set_value(self.X_ref, x_ref)
        
        # GP predictions for enhanced dynamics
        if self.use_gp and self.gp_available:
            gp_start = time.time()
            
            # Predict GP residuals for each horizon step
            gp_pos_corrections = np.zeros((3, self.N))
            gp_vel_corrections = np.zeros((3, self.N))
            
            # Use current state and rough control estimate
            for k in range(self.N):
                # Simple control estimate (can be improved with iterative refinement)
                rough_control = np.array([0, 0, 9.81, 0])  # Hover as initial guess
                
                pos_res, vel_res = self.predict_gp_residuals(current_state, rough_control)
                gp_pos_corrections[:, k] = pos_res
                gp_vel_corrections[:, k] = vel_res
            
            self.opti.set_value(self.GP_pos, gp_pos_corrections)
            self.opti.set_value(self.GP_vel, gp_vel_corrections)
            
            gp_time = time.time() - gp_start
            self.gp_prediction_times.append(gp_time)
        
        # Solve MPC
        try:
            sol = self.opti.solve()
            u_opt = sol.value(self.U)
            solve_time = time.time() - solve_start
            self.mpc_solve_times.append(solve_time)
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

    def vehicle_status_callback(self, msg):
        """Vehicle status callback"""
        self.vehicle_status = msg

    def vehicle_local_position_callback(self, msg):
        """Vehicle position callback"""
        self.current_state[:3] = [msg.x, msg.y, msg.z]
        self.current_state[6:9] = [msg.vx, msg.vy, msg.vz]

    def vehicle_attitude_callback(self, msg):
        """Vehicle attitude callback"""  
        q = [msg.q[0], msg.q[1], msg.q[2], msg.q[3]]  # w, x, y, z
        roll, pitch, yaw = self.quaternion_to_euler(q)
        self.current_state[3:6] = [roll, pitch, yaw]

    def quaternion_to_euler(self, q):
        """Convert quaternion to Euler angles"""
        w, x, y, z = q
        
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(max(-1, min(1, sinp)))
        
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw

    def control_loop(self):
        """Main control loop with flight phase management"""
        if self.vehicle_status is None:
            return

        current_time = time.time()
        
        # Flight phase management (same as baseline)
        if self.flight_phase == "INIT":
            self.phase_start_time = current_time
            self.flight_phase = "TAKEOFF"
            mode_str = "GP-Enhanced" if (self.gp_available and self.use_gp) else "Baseline"
            self.get_logger().info(f"🚁 Starting {mode_str} MPC Flight")
            
        elif self.flight_phase == "TAKEOFF":
            if current_time - self.phase_start_time > 2.0:
                self.arm()
                self.flight_phase = "ARM_WAIT"
                self.phase_start_time = current_time
                
        elif self.flight_phase == "ARM_WAIT":
            if current_time - self.phase_start_time > 2.0:
                self.flight_phase = "CLIMB"
                self.phase_start_time = current_time
                
        elif self.flight_phase == "CLIMB":
            if current_time - self.phase_start_time > 8.0 or abs(self.current_state[2] - self.takeoff_height) < 0.5:
                self.flight_phase = "HOVER"
                self.hover_start_time = current_time
                
        elif self.flight_phase == "HOVER":
            if current_time - self.hover_start_time > 5.0:
                self.flight_phase = "MPC_TRAJECTORY"
                self.phase_start_time = current_time
                mode_str = "GP-Enhanced" if (self.gp_available and self.use_gp) else "Baseline"
                self.get_logger().info(f"🧠 Starting {mode_str} Trajectory Following")
        
        # Control execution
        if self.flight_phase in ["TAKEOFF", "ARM_WAIT", "CLIMB"]:
            self.publish_offboard_control_mode()
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height, 0.0)
            
        elif self.flight_phase == "HOVER":
            self.publish_offboard_control_mode()
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height, 0.0)
            
        elif self.flight_phase == "MPC_TRAJECTORY":
            # Get trajectory target
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
            
            # Solve MPC (with or without GP)
            control_output = self.solve_mpc(self.current_state, target_pos, 0.0)
            
            # Execute control
            self.publish_offboard_control_mode_rates()
            roll_rate, pitch_rate, yaw_rate, thrust = self.acceleration_to_body_rates(
                control_output[:3], control_output[3], self.current_state
            )
            self.publish_body_rates(roll_rate, pitch_rate, yaw_rate, thrust)
            
            # Track performance
            pos_error = np.linalg.norm(np.array(target_pos) - self.current_state[:3])
            self.tracking_errors.append(pos_error)
            
            # Periodic status logging
            if self.offboard_setpoint_counter % 100 == 0:
                avg_solve_time = np.mean(self.mpc_solve_times[-10:]) if self.mpc_solve_times else 0
                avg_error = np.mean(self.tracking_errors[-50:]) if self.tracking_errors else 0
                
                mode_str = "GP-Enhanced" if (self.gp_available and self.use_gp) else "Baseline"
                self.get_logger().info(
                    f"🧠 {mode_str} MPC | Error: {pos_error:.3f}m | "
                    f"Avg Error: {avg_error:.3f}m | Solve: {avg_solve_time*1000:.1f}ms"
                )

        self.offboard_setpoint_counter += 1

    def acceleration_to_body_rates(self, acc_cmd, yaw_rate_cmd, state):
        """Convert acceleration to body rates"""
        roll, pitch, yaw = state[3:6]
        
        acc_des = np.array(acc_cmd)
        acc_des[2] += 9.81
        
        thrust = np.linalg.norm(acc_des)
        thrust = max(0.1, min(thrust / 15.0, 1.0))
        
        if np.linalg.norm(acc_des) > 0:
            thrust_dir = acc_des / np.linalg.norm(acc_des)
        else:
            thrust_dir = np.array([0, 0, 1])
        
        roll_des = math.asin(max(-0.5, min(0.5, thrust_dir[1])))
        pitch_des = math.atan2(-thrust_dir[0], thrust_dir[2])
        
        kp_att = 3.0
        roll_rate = kp_att * (roll_des - roll)
        pitch_rate = kp_att * (pitch_des - pitch)
        yaw_rate = yaw_rate_cmd
        
        max_rate = 2.0
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
        """Publish position control mode"""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_pub.publish(msg)

    def publish_offboard_control_mode_rates(self):
        """Publish body rate control mode"""
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
        msg.position = [x, y, -z]
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

    def print_performance_summary(self):
        """Print performance statistics"""
        if self.tracking_errors and self.mpc_solve_times:
            avg_error = np.mean(self.tracking_errors)
            std_error = np.std(self.tracking_errors)
            avg_solve = np.mean(self.mpc_solve_times) * 1000  # ms
            
            mode_str = "GP-Enhanced" if (self.gp_available and self.use_gp) else "Baseline"
            
            self.get_logger().info(f"\n📊 {mode_str} MPC Performance Summary:")
            self.get_logger().info(f"  Average tracking error: {avg_error:.3f} ± {std_error:.3f} m")
            self.get_logger().info(f"  Average solve time: {avg_solve:.1f} ms")
            
            if self.gp_prediction_times:
                avg_gp_time = np.mean(self.gp_prediction_times) * 1000
                self.get_logger().info(f"  Average GP prediction time: {avg_gp_time:.1f} ms")

def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        controller = GPEnhancedMPC()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("🛑 Interrupted by user")
    finally:
        if 'controller' in locals():
            controller.print_performance_summary()
        rclpy.shutdown()

if __name__ == '__main__':
    main()