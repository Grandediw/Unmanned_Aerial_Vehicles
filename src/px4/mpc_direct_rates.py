#!/usr/bin/env python3
"""
Direct Rate-Output MPC Controller with True GP Enhancement
=========================================================
This implements a proper GP-enhanced MPC that:
1. Uses GP-predicted dynamics INSIDE the MPC optimization
2. Directly outputs body rates and thrust (no geometric allocation)
3. Has cleaner, more direct control architecture
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, OffboardControlMode, VehicleRatesSetpoint
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, VehicleAttitude, VehicleAngularVelocity
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PointStamped, Vector3Stamped
import time
import math
import numpy as np
import os

# MPC dependencies
try:
    import casadi as ca
    CASADI_AVAILABLE = True
except ImportError:
    CASADI_AVAILABLE = False
    print("⚠️  CasADi not available. Install with: pip install casadi")

# Simple GP dynamics learning
try:
    from .simple_gp import SimpleQuadrotorGP, SimpleGPEnhancedMPC
    GP_AVAILABLE = True
    print("🧠 Simple GP dynamics learning available")
except ImportError:
    try:
        from simple_gp import SimpleQuadrotorGP, SimpleGPEnhancedMPC
        GP_AVAILABLE = True 
        print("🧠 Simple GP dynamics learning available")
    except ImportError:
        GP_AVAILABLE = False
        print("⚠️  Scikit-learn not available. Install with: pip install scikit-learn")

def quaternion_to_euler(q):
    """Convert quaternion to Euler angles (roll, pitch, yaw)"""
    w, x, y, z = q[0], q[1], q[2], q[3]
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw

def wrap_angle(angle):
    """Wrap angle to [-π, π] range"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

class DirectRateMPC:
    """
    True GP-Enhanced MPC that directly outputs body rates and thrust
    State: [x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r] (12D full state)
    Control: [p_cmd, q_cmd, r_cmd, thrust] (direct body rates + thrust)
    """
    
    def __init__(self, dt=0.02, N=20, use_gp=True, gp_model_path=None):
        if not CASADI_AVAILABLE:
            raise ImportError("CasADi is required for MPC. Install with: pip install casadi")
            
        self.dt = dt
        self.N = N  # Prediction horizon
        self.use_gp = use_gp and GP_AVAILABLE
        self.gp_model_path = gp_model_path
        
        # Full quadrotor state: [x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r]
        self.n_states = 12
        # Direct control: [p_cmd, q_cmd, r_cmd, thrust_cmd]
        self.n_controls = 4
        
        # Initialize GP
        self.gp_dynamics = None
        self.gp_enhanced = None
        if self.use_gp and GP_AVAILABLE:
            self.gp_dynamics = SimpleQuadrotorGP(max_data_points=1000)
            
            # Load pre-trained model if available
            if gp_model_path and os.path.exists(gp_model_path):
                if self.gp_dynamics.load_model(gp_model_path):
                    print(f"✅ GP model loaded: {gp_model_path}")
                    self.gp_enhanced = SimpleGPEnhancedMPC(
                        self.gp_dynamics, 
                        confidence_threshold=0.15
                    )
                else:
                    print(f"⚠️ Failed to load GP model: {gp_model_path}")
                    self.use_gp = False
            else:
                print("📊 GP initialized for data collection only")
                self.use_gp = False  # Collect data but don't use GP yet
        
        self.setup_mpc_problem()
        self.previous_solution = None
        
    def setup_mpc_problem(self):
        """Setup the MPC optimization problem with full quadrotor dynamics"""
        
        # === FULL QUADROTOR STATE ===
        # Position and velocity
        x = ca.SX.sym('x')      # position x
        y = ca.SX.sym('y')      # position y
        z = ca.SX.sym('z')      # position z
        vx = ca.SX.sym('vx')    # velocity x
        vy = ca.SX.sym('vy')    # velocity y
        vz = ca.SX.sym('vz')    # velocity z
        
        # Attitude
        roll = ca.SX.sym('roll')    # roll angle
        pitch = ca.SX.sym('pitch')  # pitch angle
        yaw = ca.SX.sym('yaw')      # yaw angle
        
        # Angular rates
        p = ca.SX.sym('p')      # roll rate
        q = ca.SX.sym('q')      # pitch rate
        r = ca.SX.sym('r')      # yaw rate
        
        states = ca.vertcat(x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r)
        
        # === DIRECT CONTROL COMMANDS ===
        p_cmd = ca.SX.sym('p_cmd')          # roll rate command
        q_cmd = ca.SX.sym('q_cmd')          # pitch rate command
        r_cmd = ca.SX.sym('r_cmd')          # yaw rate command
        thrust_cmd = ca.SX.sym('thrust_cmd') # thrust command
        
        controls = ca.vertcat(p_cmd, q_cmd, r_cmd, thrust_cmd)
        
        # === QUADROTOR DYNAMICS ===
        g = 9.81
        
        # Position dynamics
        x_dot = vx
        y_dot = vy
        z_dot = vz
        
        # Velocity dynamics (from thrust and attitude)
        vx_dot = thrust_cmd * (ca.sin(roll) * ca.sin(yaw) + ca.cos(roll) * ca.cos(yaw) * ca.sin(pitch))
        vy_dot = thrust_cmd * (-ca.sin(roll) * ca.cos(yaw) + ca.cos(roll) * ca.sin(yaw) * ca.sin(pitch))
        vz_dot = thrust_cmd * (ca.cos(roll) * ca.cos(pitch)) - g
        
        # Attitude dynamics (angular velocities integrate to angles)
        roll_dot = p + q * ca.sin(roll) * ca.tan(pitch) + r * ca.cos(roll) * ca.tan(pitch)
        pitch_dot = q * ca.cos(roll) - r * ca.sin(roll)
        yaw_dot = q * ca.sin(roll) / ca.cos(pitch) + r * ca.cos(roll) / ca.cos(pitch)
        
        # Angular rate dynamics (first-order approximation - rates track commands)
        tau_p = 0.05  # Time constant for roll rate
        tau_q = 0.05  # Time constant for pitch rate
        tau_r = 0.08  # Time constant for yaw rate
        
        p_dot = (p_cmd - p) / tau_p
        q_dot = (q_cmd - q) / tau_q
        r_dot = (r_cmd - r) / tau_r
        
        # Combine all dynamics
        rhs = ca.vertcat(
            x_dot, y_dot, z_dot,           # position rates
            vx_dot, vy_dot, vz_dot,        # velocity rates
            roll_dot, pitch_dot, yaw_dot,  # attitude rates
            p_dot, q_dot, r_dot            # angular acceleration
        )
        
        # Create nominal dynamics function
        self.f_nominal = ca.Function('f_nominal', [states, controls], [rhs])
        
        # GP prediction cache for current solve
        self.gp_predictions_cache = {}
        
        # Store symbolic variables for dynamic constraint updates
        self.X_sym = None
        self.U_sym = None
        self.P_sym = None
        self.obj_sym = None
        self.g_dynamics_indices = []  # Track which constraints are dynamics
        
        # === MPC OPTIMIZATION PROBLEM ===
        X = ca.SX.sym('X', self.n_states, self.N + 1)    # States over horizon
        U = ca.SX.sym('U', self.n_controls, self.N)      # Controls over horizon
        P = ca.SX.sym('P', self.n_states + 3)            # Parameters [current_state, target_pos]
        
        # Objective function weights
        Q_pos = ca.diag([100.0, 100.0, 120.0])     # Position tracking
        Q_vel = ca.diag([10.0, 10.0, 15.0])        # Velocity damping
        Q_att = ca.diag([5.0, 5.0, 8.0])           # Attitude regulation
        Q_rate = ca.diag([2.0, 2.0, 3.0])          # Rate damping
        
        R = ca.diag([1.0, 1.0, 1.5, 0.5])          # Control effort [p,q,r,thrust]
        
        obj = 0  # Objective function
        g = []   # Constraints
        
        # Store symbolic variables for dynamic updates
        self.X_sym = X
        self.U_sym = U
        self.P_sym = P
        
        # Initial condition constraint
        g.append(X[:, 0] - P[:self.n_states])
        
        for k in range(self.N):
            # Position tracking cost
            pos_error = X[:3, k] - P[self.n_states:]     # target position
            obj += ca.mtimes([pos_error.T, Q_pos, pos_error])
            
            # Velocity damping
            vel_error = X[3:6, k]  # target velocity = 0
            obj += ca.mtimes([vel_error.T, Q_vel, vel_error])
            
            # Attitude regulation (prefer level flight)
            att_error = X[6:9, k]  # target attitude = [0,0,yaw_ref]
            att_error[2] = 0  # Don't penalize yaw for now
            obj += ca.mtimes([att_error.T, Q_att, att_error])
            
            # Rate damping
            rate_error = X[9:12, k]  # target rates = 0
            obj += ca.mtimes([rate_error.T, Q_rate, rate_error])
            
            # Control effort cost
            obj += ca.mtimes([U[:, k].T, R, U[:, k]])
            
            # Dynamics constraint (will be enhanced with GP)
            x_next_nominal = X[:, k] + self.dt * self.f_nominal(X[:, k], U[:, k])
            
            # For now use nominal - will be replaced in solve() with GP-enhanced
            g.append(X[:, k + 1] - x_next_nominal)
            self.g_dynamics_indices.append(len(g) - 1)  # Track this constraint
        
        # Terminal cost
        pos_error = X[:3, -1] - P[self.n_states:]
        obj += 5 * ca.mtimes([pos_error.T, Q_pos, pos_error])
        
        # Store optimization structure (solver created dynamically in solve())
        self.obj_sym = obj
        self.g_base = g  # Base constraints without GP
        
        # Create optimization variables
        OPT_variables = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
        self.OPT_variables = OPT_variables
        
        # Solver options
        self.solver_opts = {
            'ipopt': {
                'max_iter': 80,
                'print_level': 0,
                'acceptable_tol': 1e-6,
                'acceptable_obj_change_tol': 1e-4
            },
            'print_time': 0
        }
        
        # Will create solver dynamically in solve() method
        self.solver = None
        
        # === CONSTRAINTS ===
        self.lbx = []
        self.ubx = []
        
        # State bounds
        for k in range(self.N + 1):
            # Position bounds
            self.lbx += [-50, -50, -2]        # x, y, z
            self.ubx += [50, 50, 25]
            
            # Velocity bounds
            self.lbx += [-12, -12, -8]        # vx, vy, vz
            self.ubx += [12, 12, 8]
            
            # Attitude bounds (radians)
            self.lbx += [-0.5, -0.5, -3.14]  # roll, pitch, yaw
            self.ubx += [0.5, 0.5, 3.14]
            
            # Angular rate bounds
            self.lbx += [-3.0, -3.0, -2.0]   # p, q, r
            self.ubx += [3.0, 3.0, 2.0]
        
        # Control bounds
        for k in range(self.N):
            self.lbx += [-2.5, -2.5, -1.8, 0.2]  # p_cmd, q_cmd, r_cmd, thrust_cmd
            self.ubx += [2.5, 2.5, 1.8, 1.5]     # thrust in [0.2, 1.5] range
        
        # Constraint bounds (equality constraints for dynamics)
        self.lbg = [0] * (self.n_states * (self.N + 1))
        self.ubg = [0] * (self.n_states * (self.N + 1))
    
    def add_training_data(self, state_prev, control_prev, state_current):
        """Add training data for GP learning"""
        if self.gp_dynamics is not None:
            self.gp_dynamics.add_training_data(
                state_prev, control_prev, state_current, self.dt
            )
    
    def _get_gp_predictions_for_horizon(self, X_guess, U_guess):
        """Get GP predictions for entire MPC horizon"""
        
        if (not self.use_gp or 
            self.gp_dynamics is None or 
            not self.gp_dynamics.is_trained or
            self.gp_enhanced is None):
            return {}
            
        gp_predictions = {}
        
        for k in range(self.N):
            try:
                # Extract state and control for this step
                if X_guess is not None and U_guess is not None:
                    state_k = X_guess[:, k] if X_guess.shape[1] > k else X_guess[:, 0]
                    control_k = U_guess[:, k] if U_guess.shape[1] > k else U_guess[:, 0]
                else:
                    # Use current state as fallback
                    state_k = np.zeros(self.n_states)
                    control_k = np.zeros(self.n_controls)
                
                # Get GP prediction for dynamics residual
                residual_mean, residual_var = self.gp_dynamics.predict_residual(
                    state_k, control_k
                )
                
                # Only use GP if confident (low uncertainty)
                uncertainty = np.sqrt(np.sum(residual_var))
                if uncertainty < self.gp_enhanced.confidence_threshold:
                    gp_predictions[k] = residual_mean
                else:
                    gp_predictions[k] = np.zeros(self.n_states)  # Use nominal if uncertain
                    
            except Exception as e:
                # Fallback to nominal dynamics on error
                gp_predictions[k] = np.zeros(self.n_states)
                
        return gp_predictions
    
    def _create_gp_enhanced_dynamics_function(self, gp_predictions):
        """Create CasADi function with GP-enhanced dynamics for this solve"""
        
        x = ca.SX.sym('x', self.n_states)  # State
        u = ca.SX.sym('u', self.n_controls)  # Control
        k_step = ca.SX.sym('k_step')  # Horizon step (for step-specific GP)
        
        # Nominal dynamics
        f_nom = self.f_nominal(x, u)
        
        # Add GP residuals as constants (pre-computed for this solve)
        if gp_predictions and len(gp_predictions) > 0:
            # Use step-specific GP predictions
            # For CasADi, we'll create separate functions for each step
            return self._create_step_specific_gp_functions(gp_predictions)
        else:
            # No GP enhancement - return nominal
            return {k: ca.Function(f'f_enhanced_{k}', [x, u], [f_nom]) 
                   for k in range(self.N)}
    
    def _create_step_specific_gp_functions(self, gp_predictions):
        """Create step-specific GP-enhanced dynamics functions"""
        
        x = ca.SX.sym('x', self.n_states)
        u = ca.SX.sym('u', self.n_controls)
        
        enhanced_functions = {}
        
        for k in range(self.N):
            # Nominal dynamics
            f_nom = self.f_nominal(x, u)
            
            # Add GP residual for this step
            if k in gp_predictions:
                gp_residual = ca.DM(gp_predictions[k])
                f_enhanced = f_nom + gp_residual
            else:
                f_enhanced = f_nom
            
            enhanced_functions[k] = ca.Function(
                f'f_enhanced_{k}', [x, u], [f_enhanced]
            )
        
        return enhanced_functions
    
    def solve(self, current_state, target_pos, target_yaw=0.0):
        """Solve MPC with GP-enhanced dynamics integration"""
        
        # Parameters: [current_state, target_pos]
        p = np.concatenate([current_state, target_pos])
        
        # Get initial guess
        if self.previous_solution is None:
            X_guess, U_guess = self._get_initial_guess(current_state)
        else:
            X_guess, U_guess = self._get_warm_start_guess(current_state)
        
        # === GP INTEGRATION: Get predictions for entire horizon ===
        gp_predictions = self._get_gp_predictions_for_horizon(X_guess, U_guess)
        
        # Create GP-enhanced dynamics functions for each step
        f_enhanced_functions = self._create_gp_enhanced_dynamics_function(gp_predictions)
        
        # Create GP-enhanced constraints
        g_gp_enhanced = self._create_gp_enhanced_constraints(f_enhanced_functions)
        
        # Create updated NLP problem
        nlp_prob_gp = {
            'f': self.obj_sym,
            'x': self.OPT_variables,
            'g': ca.vertcat(*g_gp_enhanced),
            'p': self.P_sym
        }
        
        # Create solver for this iteration
        solver_gp = ca.nlpsol('solver_gp', 'ipopt', nlp_prob_gp, self.solver_opts)
        
        # Prepare initial guess
        x0 = np.concatenate([X_guess.flatten(), U_guess.flatten()])
        
        # Solve GP-enhanced optimization
        try:
            sol = solver_gp(
                x0=x0,
                lbx=self.lbx,
                ubx=self.ubx,
                lbg=self.lbg,
                ubg=self.ubg,
                p=p
            )
            
            # Extract solution
            x_opt = sol['x'].full().flatten()
            X_opt = x_opt[:(self.N + 1) * self.n_states].reshape((self.n_states, self.N + 1))
            U_opt = x_opt[(self.N + 1) * self.n_states:].reshape((self.n_controls, self.N))
            
            # Save for warm start
            self.previous_solution = {'X': X_opt, 'U': U_opt}
            
            # Return first control action: [p_cmd, q_cmd, r_cmd, thrust_cmd]
            control_output = U_opt[:, 0].copy()
            
            # Apply safety limits
            control_output = np.clip(control_output, 
                                   [-2.5, -2.5, -1.8, 0.2], 
                                   [2.5, 2.5, 1.8, 1.5])
            
            return control_output, X_opt
            
        except Exception as e:
            print(f"GP-enhanced MPC solver failed: {e}")
            # Fallback: try nominal dynamics
            return self._solve_nominal_fallback(current_state, target_pos, target_yaw)
    
    def _create_gp_enhanced_constraints(self, f_enhanced_functions):
        """Create constraints with GP-enhanced dynamics"""
        
        g_enhanced = []
        
        # Initial condition constraint
        g_enhanced.append(self.X_sym[:, 0] - self.P_sym[:self.n_states])
        
        # Dynamics constraints with GP enhancement
        for k in range(self.N):
            # Position tracking cost (same as before)
            # This was in the objective, not constraints
            
            # GP-enhanced dynamics constraint
            x_next_enhanced = (self.X_sym[:, k] + 
                             self.dt * f_enhanced_functions[k](self.X_sym[:, k], self.U_sym[:, k]))
            g_enhanced.append(self.X_sym[:, k + 1] - x_next_enhanced)
        
        return g_enhanced
    
    def _solve_nominal_fallback(self, current_state, target_pos, target_yaw):
        """Fallback to nominal MPC if GP-enhanced version fails"""
        
        # Create simple nominal constraints
        g_nominal = []
        g_nominal.append(self.X_sym[:, 0] - self.P_sym[:self.n_states])
        
        for k in range(self.N):
            x_next = (self.X_sym[:, k] + 
                     self.dt * self.f_nominal(self.X_sym[:, k], self.U_sym[:, k]))
            g_nominal.append(self.X_sym[:, k + 1] - x_next)
            
        nlp_nominal = {
            'f': self.obj_sym,
            'x': self.OPT_variables, 
            'g': ca.vertcat(*g_nominal),
            'p': self.P_sym
        }
        
        solver_nominal = ca.nlpsol('solver_nominal', 'ipopt', nlp_nominal, self.solver_opts)
        
        # Get initial guess
        if self.previous_solution is None:
            X_guess, U_guess = self._get_initial_guess(current_state)
        else:
            X_guess, U_guess = self._get_warm_start_guess(current_state)
        
        x0 = np.concatenate([X_guess.flatten(), U_guess.flatten()])
        p = np.concatenate([current_state, target_pos])
        
        try:
            sol = solver_nominal(
                x0=x0, lbx=self.lbx, ubx=self.ubx,
                lbg=self.lbg, ubg=self.ubg, p=p
            )
            
            x_opt = sol['x'].full().flatten()
            X_opt = x_opt[:(self.N + 1) * self.n_states].reshape((self.n_states, self.N + 1))
            U_opt = x_opt[(self.N + 1) * self.n_states:].reshape((self.n_controls, self.N))
            
            return U_opt[:, 0].copy(), X_opt
            
        except Exception as e:
            print(f"Nominal MPC fallback also failed: {e}")
            return np.array([0.0, 0.0, 0.0, 0.8]), None
    
    def _get_initial_guess(self, current_state):
        """Generate initial guess for cold start"""
        
        X_guess = np.zeros((self.n_states, self.N + 1))
        U_guess = np.zeros((self.n_controls, self.N))
        
        # Initialize all states with current state
        for k in range(self.N + 1):
            X_guess[:, k] = current_state
            
        # Initialize controls with hover
        for k in range(self.N):
            U_guess[:, k] = [0.0, 0.0, 0.0, 0.8]  # [p,q,r,thrust]
            
        return X_guess, U_guess
    
    def _get_warm_start_guess(self, current_state):
        """Generate warm start guess from previous solution"""
        
        if self.previous_solution is None:
            return self._get_initial_guess(current_state)
            
        X_prev = self.previous_solution['X']
        U_prev = self.previous_solution['U']
        
        # Shift and update
        X_guess = np.hstack([X_prev[:, 1:], X_prev[:, -1:]])
        X_guess[:, 0] = current_state
        
        U_guess = np.hstack([U_prev[:, 1:], U_prev[:, -1:]])
        
        return X_guess, U_guess
    
    def shift_solution(self, prev_sol, current_state):
        """Shift previous solution for warm start"""
        X_prev = prev_sol['X']
        U_prev = prev_sol['U']
        
        # Shift states
        X_shifted = np.hstack([X_prev[:, 1:], X_prev[:, -1:]])
        X_shifted[:, 0] = current_state
        
        # Shift controls
        U_shifted = np.hstack([U_prev[:, 1:], U_prev[:, -1:]])
        
        # Combine into optimization variable format
        x0 = np.concatenate([X_shifted.flatten(), U_shifted.flatten()])
        
        return x0


class DirectRateMPCDemo(Node):
    """Direct Rate MPC Demo - cleaner architecture"""
    
    def __init__(self):
        super().__init__('direct_rate_mpc_demo')
        
        # Configure QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers (only what we need)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.vehicle_rates_setpoint_pub = self.create_publisher(VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', 10)

        # Monitoring publishers
        self.position_current_pub = self.create_publisher(PointStamped, '/mpc_direct/position_current', 10)
        self.position_setpoint_pub = self.create_publisher(PointStamped, '/mpc_direct/position_setpoint', 10)
        self.control_outputs_pub = self.create_publisher(Float64MultiArray, '/mpc_direct/control_outputs', 10)
        
        # Subscribers
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.position_callback, qos_profile)
        self.vehicle_attitude_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos_profile)
        self.vehicle_angular_velocity_sub = self.create_subscription(
            VehicleAngularVelocity, '/fmu/out/vehicle_angular_velocity', self.angular_velocity_callback, qos_profile)
        
        # State variables
        self.armed = False
        self.takeoff_height = 3.0
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_velocity = np.array([0.0, 0.0, 0.0])
        self.current_attitude = np.array([0.0, 0.0, 0.0])  # [roll, pitch, yaw]
        self.current_angular_velocity = np.array([0.0, 0.0, 0.0])  # [p, q, r]
        
        # Initialize Direct Rate MPC
        model_path = os.getenv('GP_MODEL_PATH', '/home/grandediw/ros2_px4_offboard_example_ws/gp_models/gp_model_20251119_030043.pkl')
        use_gp = os.getenv('GP_USE_MODEL', 'false').lower() == 'true'
        
        if CASADI_AVAILABLE:
            self.mpc = DirectRateMPC(
                dt=0.02, 
                N=20, 
                use_gp=use_gp, 
                gp_model_path=model_path if use_gp else None
            )
            self.get_logger().info("✅ Direct Rate MPC initialized")
            if use_gp:
                self.get_logger().info("🧠 GP enhancement enabled")
            else:
                self.get_logger().info("📊 Standard MPC (no GP)")
        else:
            self.get_logger().error("❌ CasADi not available")
            self.mpc = None
        
        # Trajectory parameters
        self.pattern_amplitude = 4.0
        self.pattern_frequency = 0.03
        
        # State history for GP learning
        self.previous_state = None
        self.previous_control = None
        
        # Control loop
        self.timer = self.create_timer(0.02, self.control_loop)
        self.start_time = time.time()
        
        self.get_logger().info("🎯 Direct Rate MPC Demo Started!")
    
    def vehicle_status_callback(self, msg):
        """Get vehicle arming status"""
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        
    def position_callback(self, msg):
        """Get current position and velocity feedback"""
        self.current_position = np.array([msg.x, msg.y, -msg.z])
        self.current_velocity = np.array([msg.vx, msg.vy, -msg.vz])
        
    def attitude_callback(self, msg):
        """Get current attitude feedback"""
        q = [msg.q[0], msg.q[1], msg.q[2], msg.q[3]]  # [w, x, y, z]
        roll, pitch, yaw = quaternion_to_euler(q)
        self.current_attitude = np.array([wrap_angle(roll), wrap_angle(pitch), wrap_angle(yaw)])
        
    def angular_velocity_callback(self, msg):
        """Get current angular velocity feedback"""
        self.current_angular_velocity = np.array([msg.xyz[0], msg.xyz[1], msg.xyz[2]])
    
    def get_current_state_vector(self):
        """Get current state as 12D vector: [x,y,z,vx,vy,vz,roll,pitch,yaw,p,q,r]"""
        return np.concatenate([
            self.current_position,        # [x, y, z]
            self.current_velocity,        # [vx, vy, vz] 
            self.current_attitude,        # [roll, pitch, yaw]
            self.current_angular_velocity # [p, q, r]
        ])
    
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
        
    def publish_rates_setpoint(self, rollrate=0.0, pitchrate=0.0, yawrate=0.0, thrust=0.5):
        """Publish body rate and thrust setpoint"""
        msg = VehicleRatesSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        msg.roll = float(rollrate)
        msg.pitch = float(pitchrate)
        msg.yaw = float(yawrate)
        
        normalized_thrust = np.clip(thrust, 0.0, 1.0)
        msg.thrust_body[0] = 0.0
        msg.thrust_body[1] = 0.0
        msg.thrust_body[2] = -normalized_thrust  # Negative for upward thrust in NED
        
        self.vehicle_rates_setpoint_pub.publish(msg)
    
    def calculate_reference_trajectory(self, t):
        """Calculate reference trajectory"""
        A = self.pattern_amplitude
        w = 2.0 * math.pi * self.pattern_frequency
        
        ramp = math.tanh(t / 3.0)  # 3s ramp-in
        
        x = ramp * A * math.sin(w * t)
        y = ramp * (A / 2.0) * math.sin(2.0 * w * t)  # Figure-8
        z = self.takeoff_height
        
        return np.array([x, y, z]), 0.0
    
    def control_loop(self):
        """Main control loop"""
        current_time = time.time() - self.start_time
        
        if current_time < 2.0:
            # Phase 1: Startup
            self.get_logger().info("🔄 Initializing Direct Rate MPC...")
            return
            
        elif current_time < 4.0:
            # Phase 2: Arm
            if not self.armed:
                self.arm_vehicle()
                self.get_logger().info("🔫 Arming vehicle...")
            return
            
        elif current_time < 6.0:
            # Phase 3: Switch to offboard
            self.switch_to_offboard()
            self.get_logger().info("🎯 Switching to offboard...")
            return
            
        else:
            # Phase 4: Direct Rate MPC Control
            if self.mpc is None:
                self.get_logger().error("❌ MPC not available!")
                return
            
            flight_time = current_time - 6.0
            
            # Calculate target
            target_pos, target_yaw = self.calculate_reference_trajectory(flight_time)
            
            # Get current state
            current_state = self.get_current_state_vector()
            
            # GP data collection
            if (self.mpc.gp_dynamics is not None and 
                self.previous_state is not None and 
                self.previous_control is not None):
                self.mpc.add_training_data(
                    self.previous_state, 
                    self.previous_control, 
                    current_state
                )
            
            # Solve MPC
            control_cmd, X_opt = self.mpc.solve(current_state, target_pos, target_yaw)
            
            if control_cmd is not None:
                # Extract commands: [p_cmd, q_cmd, r_cmd, thrust_cmd]
                rollrate_cmd = control_cmd[0]
                pitchrate_cmd = control_cmd[1]
                yawrate_cmd = control_cmd[2]
                thrust_cmd = control_cmd[3]
                
                # Apply safety limits
                rollrate_cmd = np.clip(rollrate_cmd, -2.0, 2.0)
                pitchrate_cmd = np.clip(pitchrate_cmd, -2.0, 2.0)
                yawrate_cmd = np.clip(yawrate_cmd, -1.5, 1.5)
                thrust_cmd = np.clip(thrust_cmd, 0.9, 1.4)
                
            else:
                # Safe hover
                rollrate_cmd = 0.0
                pitchrate_cmd = 0.0
                yawrate_cmd = 0.0
                thrust_cmd = 0.8
                self.get_logger().warn("⚠️ MPC failed - using safe hover")
            
            # Publish control commands
            self.publish_offboard_control_mode_rates()
            self.publish_rates_setpoint(rollrate_cmd, pitchrate_cmd, yawrate_cmd, thrust_cmd)
            
            # Update history
            self.previous_state = current_state.copy()
            self.previous_control = control_cmd.copy() if control_cmd is not None else np.array([0,0,0,0.8])
            
            # Publish monitoring data
            self.publish_monitoring_data(target_pos, current_state, control_cmd, flight_time)
            
            # Log performance
            if int(flight_time) % 3 == 0 and flight_time > 0:
                pos_error = np.linalg.norm(target_pos - self.current_position)
                self.get_logger().info(f"🎯 Direct Rate MPC (t={flight_time:.1f}s)")
                self.get_logger().info(f"   📍 Position: target=({target_pos[0]:.1f},{target_pos[1]:.1f},{target_pos[2]:.1f}), error={pos_error:.3f}m")
                self.get_logger().info(f"   🎪 Rates: cmd=({math.degrees(rollrate_cmd):.1f}°/s, {math.degrees(pitchrate_cmd):.1f}°/s, {math.degrees(yawrate_cmd):.1f}°/s)")
                self.get_logger().info(f"   🚀 Thrust: {thrust_cmd:.3f}")
                
                # GP status and integration verification
                if self.mpc.use_gp and self.mpc.gp_dynamics is not None:
                    gp_stats = self.mpc.gp_dynamics.get_stats()
                    is_gp_active = "ACTIVE" if (self.mpc.gp_enhanced is not None and gp_stats['is_trained']) else "COLLECTING"
                    gp_in_dynamics = "YES" if len(self.mpc.gp_predictions_cache) > 0 else "NO"
                    
                    self.get_logger().info(f"   🧠 GP: {gp_stats['data_points']} samples, status={is_gp_active}")
                    self.get_logger().info(f"   🔬 GP in MPC dynamics: {gp_in_dynamics} (predictions cached: {len(self.mpc.gp_predictions_cache)})")
                    
                    # Show the key difference
                    if flight_time < 8.0:  # Show this info early
                        self.get_logger().info(f"   ✅ ARCHITECTURE: GP → MPC Dynamics → Optimal Control (PROPER)")
                        self.get_logger().info(f"   ✅ vs OLD: MPC → Control → GP Correction (POST-HOC)")
    
    def publish_monitoring_data(self, target_pos, current_state, control_cmd, flight_time):
        """Publish monitoring data"""
        current_time = self.get_clock().now()
        
        # Position setpoint
        pos_setpoint_msg = PointStamped()
        pos_setpoint_msg.header.stamp = current_time.to_msg()
        pos_setpoint_msg.header.frame_id = "map"
        pos_setpoint_msg.point.x = float(target_pos[0])
        pos_setpoint_msg.point.y = float(target_pos[1])
        pos_setpoint_msg.point.z = float(target_pos[2])
        self.position_setpoint_pub.publish(pos_setpoint_msg)
        
        # Current position
        pos_current_msg = PointStamped()
        pos_current_msg.header.stamp = current_time.to_msg()
        pos_current_msg.header.frame_id = "map"
        pos_current_msg.point.x = float(self.current_position[0])
        pos_current_msg.point.y = float(self.current_position[1])
        pos_current_msg.point.z = float(self.current_position[2])
        self.position_current_pub.publish(pos_current_msg)
        
        # Control outputs
        if control_cmd is not None:
            control_msg = Float64MultiArray()
            control_msg.data = [
                float(target_pos[0]), float(target_pos[1]), float(target_pos[2]),  # target
                float(self.current_position[0]), float(self.current_position[1]), float(self.current_position[2]),  # current
                float(control_cmd[0]), float(control_cmd[1]), float(control_cmd[2]), float(control_cmd[3]),  # rates + thrust
                float(flight_time)
            ]
            self.control_outputs_pub.publish(control_msg)


def main(args=None):
    print('🚀 Starting Direct Rate MPC Controller...')
    print('📊 Control Architecture:')
    print('   Direct MPC -> Body Rates + Thrust -> PX4')
    print('🧠 True GP-enhanced dynamics (when enabled)')
    print('')
    
    rclpy.init(args=args)
    direct_rate_mpc_demo = DirectRateMPCDemo()
    
    try:
        rclpy.spin(direct_rate_mpc_demo)
    except KeyboardInterrupt:
        direct_rate_mpc_demo.get_logger().info('🛑 Direct Rate MPC stopped')
    finally:
        try:
            direct_rate_mpc_demo.destroy_node()
        except:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()