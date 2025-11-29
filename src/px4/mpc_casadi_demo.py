#!/usr/bin/env python3
"""
Model Predictive Control (MPC) Demo for PX4 with ROS2 using CasADi
================================================================
This demo implements a sophisticated MPC controller using CasADi for optimal
trajectory tracking with constraints on velocity, acceleration, and positions.

Available trajectories for testing:
- Use TRAJECTORY_CONFIG environment variable to select trajectory
- Example: export TRAJECTORY_CONFIG="fast_circle"
- Run trajectory_definitions.py standalone to see all available options
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition
from std_msgs.msg import Float64MultiArray, Float64, Header
from geometry_msgs.msg import PointStamped, Vector3Stamped, PoseStamped, TwistStamped
import time
import math
import numpy as np
import os
import sys

try:
    import casadi as ca
    CASADI_AVAILABLE = True
except ImportError:
    CASADI_AVAILABLE = False

# Import trajectory system
try:
    from .trajectory_definitions import get_trajectory_function, list_available_trajectories, TRAJECTORY_CONFIGS
    TRAJECTORIES_AVAILABLE = True
except ImportError:
    try: 
        from trajectory_definitions import get_trajectory_function, list_available_trajectories, TRAJECTORY_CONFIGS
        TRAJECTORIES_AVAILABLE = True
    except ImportError:
        TRAJECTORIES_AVAILABLE = False

class MPCController:
    """Model Predictive Controller using CasADi for trajectory optimization"""
    
    def __init__(self, dt=0.1, N=10):
        if not CASADI_AVAILABLE:
            raise ImportError("CasADi is required for MPC. Install with: pip install casadi")
            
        self.dt = dt  # Time step
        self.N = N    # Prediction horizon
        
        # State and control dimensions
        self.n_states = 6  # [x, y, z, vx, vy, vz]
        self.n_controls = 3  # [ax, ay, az] (acceleration commands)
        
        # System constraints - VERY AGGRESSIVE for strong tracking
        self.v_max = 2.5    # Maximum velocity (m/s) 
        self.a_max = 2.5    # Maximum acceleration (m/s²) 
        self.z_min = -5.0   # Maximum altitude (negative in NED)
        self.z_max = -0.5   # Minimum altitude (negative in NED)
        
        # MPC weights - MORE AGGRESSIVE TUNING to prevent landing
        self.Q = np.diag([30.0, 30.0, 40.0, 15.0, 15.0, 20.0])  # State weights [pos, vel] - higher Z tracking
        self.R = np.diag([2.5, 2.5, 1.0])  
        self.Qf = 2 * self.Q                                      # Terminal weights - moderate
        # self.Q = np.diag([5.0, 5.0, 12.0, 2.0, 2.0, 5.0])   # Low position weights
        # self.R = np.diag([0.8, 0.8, 1.0])  # High control penalty
        # self.Qf = 1.5 * self.Q
        
        self._setup_mpc_problem()
        
    def _setup_mpc_problem(self):
        """Setup the MPC optimization problem using CasADi"""
        
        # Symbolic variables
        x = ca.SX.sym('x', self.n_states)      # States: [x, y, z, vx, vy, vz]
        u = ca.SX.sym('u', self.n_controls)    # Controls: [ax, ay, az]
        
        # System dynamics (double integrator model)
        x_dot = ca.vertcat(
            x[3],        # dx/dt = vx
            x[4],        # dy/dt = vy  
            x[5],        # dz/dt = vz
            u[0],        # dvx/dt = ax
            u[1],        # dvy/dt = ay
            u[2]         # dvz/dt = az 
        )
        
        # Continuous-time dynamics function
        f = ca.Function('f', [x, u], [x_dot])
        
        # Discrete-time dynamics using RK4 integration
        self.F = self._rk4_discretization(f, x, u, self.dt)
        
        # Optimization variables
        X = ca.SX.sym('X', self.n_states, self.N + 1)      # State trajectory
        U = ca.SX.sym('U', self.n_controls, self.N)        # Control trajectory
        X_ref = ca.SX.sym('X_ref', self.n_states, self.N + 1)  # Reference trajectory
        
        # Decision variables vector
        w = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
        
        # Cost function
        J = 0
        
        # Stage costs
        for k in range(self.N):
            state_error = X[:, k] - X_ref[:, k]
            control_cost = U[:, k]
            J += ca.mtimes([state_error.T, self.Q, state_error]) + \
                 ca.mtimes([control_cost.T, self.R, control_cost])
        
        # Terminal cost
        terminal_error = X[:, self.N] - X_ref[:, self.N]
        J += ca.mtimes([terminal_error.T, self.Qf, terminal_error])
        
        # Constraints
        g = []
        
        # Dynamics constraints
        for k in range(self.N):
            x_next = self.F(X[:, k], U[:, k])
            g.append(X[:, k + 1] - x_next)
        
        # State and control bounds
        lbw = []  # Lower bounds on decision variables
        ubw = []  # Upper bounds on decision variables
        lbg = []  # Lower bounds on constraints
        ubg = []  # Upper bounds on constraints
        
        # State bounds
        for k in range(self.N + 1):
            # Position bounds (loose)
            lbw += [-50, -50, self.z_min, -self.v_max, -self.v_max, -self.v_max]
            ubw += [50, 50, self.z_max, self.v_max, self.v_max, self.v_max]
        
        # Control bounds
        for k in range(self.N):
            lbw += [-self.a_max, -self.a_max, -self.a_max]
            ubw += [self.a_max, self.a_max, self.a_max]
        
        # Dynamics constraint bounds (equality constraints)
        for k in range(self.N):
            lbg += [0] * self.n_states
            ubg += [0] * self.n_states
        
        # Flatten constraints
        g = ca.vertcat(*g)
        
        # Create NLP problem
        prob = {
            'f': J,
            'x': w,
            'g': g,
            'p': ca.reshape(X_ref, -1, 1)  # Parameters (reference trajectory)
        }
        
        # Solver options (tuned for stability and precision)
        opts = {
            'print_time': False,
            'ipopt.print_level': 0,
            'ipopt.max_iter': 50,              # Reduced iterations for real-time
            'ipopt.tol': 1e-4,                 # Looser tolerance for speed
            'ipopt.acceptable_tol': 1e-3,      # Acceptable tolerance
            'ipopt.acceptable_iter': 3,        # Accept sooner
            'ipopt.warm_start_init_point': 'yes',
        }
        
        # Create solver
        self.solver = ca.nlpsol('solver', 'ipopt', prob, opts)
        
        # Store bounds
        self.lbw = lbw
        self.ubw = ubw
        self.lbg = lbg
        self.ubg = ubg
        
        # Initialize solution
        self.w0 = np.zeros(len(lbw))
        
    def _rk4_discretization(self, f, x, u, dt):
        """Runge-Kutta 4th order discretization"""
        k1 = f(x, u)
        k2 = f(x + dt/2 * k1, u)
        k3 = f(x + dt/2 * k2, u)
        k4 = f(x + dt * k3, u)
        
        x_next = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
        return ca.Function('F', [x, u], [x_next])
    
    def solve(self, x0, x_ref):
        """
        Solve MPC optimization problem
        
        Args:
            x0: Current state [x, y, z, vx, vy, vz]
            x_ref: Reference trajectory (6 x N+1)
            
        Returns:
            u_opt: Optimal control [ax, ay, az]
            x_pred: Predicted state trajectory
        """
        
        # Set initial condition constraint
        self.lbw[:self.n_states] = x0
        self.ubw[:self.n_states] = x0
        
        # Flatten reference trajectory
        p = x_ref.flatten()
        
        try:
            # Solve optimization
            sol = self.solver(
                x0=self.w0,
                lbx=self.lbw,
                ubx=self.ubw,
                lbg=self.lbg,
                ubg=self.ubg,
                p=p
            )
            
            # Extract solution
            w_opt = np.array(sol['x']).flatten()
            
            # Parse solution
            n_state_vars = self.n_states * (self.N + 1)
            X_opt = w_opt[:n_state_vars].reshape((self.n_states, self.N + 1))
            U_opt = w_opt[n_state_vars:].reshape((self.n_controls, self.N))
            
            # # Update warm start
            # self.w0 = w_opt
            # === Build shifted warm-start for next call ===
            X_guess = np.hstack([X_opt[:, 1:], X_opt[:, -1:]])           # shift states
            U_guess = np.hstack([U_opt[:, 1:], U_opt[:, -1:]])           # shift controls (hold last)
            self.w0 = np.concatenate([X_guess.flatten(), U_guess.flatten()])

            
            # Return first control action and predicted trajectory
            return U_opt[:, 0], X_opt
            
        except Exception as e:
            print(f"MPC solver failed: {e}")
            # Return zero control in case of failure
            return np.zeros(3), np.zeros((self.n_states, self.N + 1))

class MPCDemo(Node):
    def __init__(self):
        super().__init__('mpc_demo')
        
        if not CASADI_AVAILABLE:
            self.get_logger().error("❌ CasADi not available! Install with: pip install casadi")
            return
        
        # QoS profile for PX4 communication - matches PX4's BEST_EFFORT reliability
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers - same as working demos
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        
        # PlotJuggler monitoring publishers
        self.reference_position_pub = self.create_publisher(PointStamped, '/mpc_monitor/reference_position', 10)
        self.current_position_pub = self.create_publisher(PointStamped, '/mpc_monitor/current_position', 10)
        self.position_error_pub = self.create_publisher(Vector3Stamped, '/mpc_monitor/position_error', 10)
        self.velocity_error_pub = self.create_publisher(Vector3Stamped, '/mpc_monitor/velocity_error', 10)
        self.mpc_output_pub = self.create_publisher(Vector3Stamped, '/mpc_monitor/mpc_acceleration', 10)
        self.trajectory_progress_pub = self.create_publisher(Float64, '/mpc_monitor/trajectory_progress', 10)
        self.flight_time_pub = self.create_publisher(Float64, '/mpc_monitor/flight_time', 10)
        self.error_magnitude_pub = self.create_publisher(Float64, '/mpc_monitor/position_error_magnitude', 10)
        self.velocity_magnitude_pub = self.create_publisher(Float64, '/mpc_monitor/velocity_error_magnitude', 10)
        self.control_effort_pub = self.create_publisher(Float64, '/mpc_monitor/control_effort', 10)
        self.trajectory_info_pub = self.create_publisher(Float64MultiArray, '/mpc_monitor/trajectory_info', 10)
        
        # Subscribers - get both status and position feedback with correct QoS
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
        
        # Flight state variables
        self.armed = False
        self.takeoff_height = 2.0
        
        # Current state (real and simulated)
        self.current_state = np.zeros(6)  # [x, y, z, vx, vy, vz] - from PX4
        self.simulated_state = np.zeros(6)  # Simulated state for fallback
        self.state_received = False
        self.last_position_time = 0
        self.use_simulated_state = False  # Flag to switch between real and simulated
        
        # MPC Controller with trajectory selection
        try:
            self.mpc = MPCController(dt=0.04, N=100)  # Shorter prediction horizon for hover stability
            self.get_logger().info("✅ MPC Controller initialized!")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize MPC: {e}")
            return
            
        # Trajectory selection and configuration
        self.trajectory_config = os.getenv('TRAJECTORY_CONFIG', 'slow_figure8')
        
        if TRAJECTORIES_AVAILABLE:
            try:
                self.trajectory_func = get_trajectory_function(self.trajectory_config)
                config_info = TRAJECTORY_CONFIGS[self.trajectory_config]
                self.get_logger().info(f"🎯 Selected trajectory: '{self.trajectory_config}' -> {config_info['name']}")
                self.get_logger().info(f"   Parameters: {config_info['params']}")
            except Exception as e:
                self.get_logger().warn(f"⚠️  Trajectory '{self.trajectory_config}' not found: {e}")
                self.get_logger().info("📋 Available trajectories:")
                for name in TRAJECTORY_CONFIGS.keys():
                    self.get_logger().info(f"   - {name}")
                # Fallback to default
                self.trajectory_func = get_trajectory_function('slow_figure8')
                self.trajectory_config = 'slow_figure8'
        else:
            self.get_logger().warn("⚠️  Trajectory definitions not available, using fallback figure-8")
            self.trajectory_func = None
            # Legacy parameters for fallback figure-8
            self.ref_amplitude_x = 3.0
            self.ref_amplitude_y = 1.5  
            self.ref_frequency = 0.08
        
        self.ref_speed = 1.5              # Reference speed
        
        # Timer for main control loop
        self.timer = self.create_timer(0.04, self.control_loop)  # 10Hz (matches MPC dt)
        self.start_time = time.time()
        self.last_control_time = self.start_time
        
        # Initialize simulated state at takeoff position
        self.simulated_state = np.array([0.0, 0.0, -self.takeoff_height, 0.0, 0.0, 0.0])
        
        self.get_logger().info(f"🚁 MPC Demo Started - {self.trajectory_config} trajectory with Model Predictive Control!")
        self.get_logger().info("🔧 Real MPC with CasADi optimization + Simulated state fallback for robust operation!")
        
    def vehicle_status_callback(self, msg):
        """Get vehicle arming status"""
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        
    def position_callback(self, msg):
        """Get current state feedback with enhanced error checking"""
        # Check if position data is valid
        if msg.xy_valid and msg.z_valid and msg.v_xy_valid and msg.v_z_valid:
            self.current_state = np.array([
                msg.x, msg.y, msg.z,      # Position in NED frame
                msg.vx, msg.vy, msg.vz    # Velocity in NED frame
            ])
            self.state_received = True
            self.last_position_time = time.time()
        else:
            # Log invalid data
            if not self.state_received:  # Only log if this is the first time
                self.get_logger().warn(f"⚠️  Invalid position data: xy_valid={msg.xy_valid}, z_valid={msg.z_valid}, v_xy_valid={msg.v_xy_valid}, v_z_valid={msg.v_z_valid}")
            
        # Log position data every 50 callbacks (about every 5 seconds)
        if hasattr(self, '_pos_callback_count'):
            self._pos_callback_count += 1
        else:
            self._pos_callback_count = 1
            
        if self._pos_callback_count % 50 == 0:
            self.get_logger().info(f"📍 Position feedback: pos=({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}), vel=({msg.vx:.2f}, {msg.vy:.2f}, {msg.vz:.2f})")
            self.get_logger().info(f"   Valid flags: xy={msg.xy_valid}, z={msg.z_valid}, v_xy={msg.v_xy_valid}, v_z={msg.v_z_valid}")
    
    def update_simulated_state(self, control_input, dt):
        """Update simulated state using simple integration"""
        # Simple double integrator model: 
        # position = position + velocity * dt
        # velocity = velocity + acceleration * dt
        
        # Update velocity first
        self.simulated_state[3:6] += control_input * dt
        
        # Update position
        self.simulated_state[0:3] += self.simulated_state[3:6] * dt
        
    def get_current_state(self):
        """Get current state (real if available, simulated as fallback)"""
        current_time = time.time()
        
        # Check if we have recent valid position data (within last 0.5 seconds)
        if self.state_received and (current_time - self.last_position_time) < 0.5:
            self.use_simulated_state = False
            return self.current_state, False  # Return real state, not simulated
        else:
            self.use_simulated_state = True
            return self.simulated_state, True  # Return simulated state, is simulated
        
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
        
    def publish_offboard_control_mode_acceleration(self):
        """Publish offboard control mode for acceleration control (MPC output)"""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = False
        msg.velocity = False
        msg.acceleration = True  # Enable acceleration control
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_pub.publish(msg)
        
    def publish_position_setpoint(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        """Publish position setpoint (for initial phases)"""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [x, y, -z]  # NED coordinates (z is negative up)
        msg.yaw = yaw
        self.trajectory_setpoint_pub.publish(msg)
        
    def publish_acceleration_setpoint(self, ax=0.0, ay=0.0, az=0.0, yaw=0.0):
        """Publish acceleration setpoint (MPC output)"""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.acceleration = [ax, ay, az]  # Acceleration in NED frame
        msg.yaw = yaw
        # Set position and velocity to NaN to indicate acceleration control
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        self.trajectory_setpoint_pub.publish(msg)
        
    def generate_reference_trajectory(self, t_start, N, dt):
        """Generate reference trajectory for MPC using selected trajectory function"""
        x_ref = np.zeros((6, N + 1))
        
        for i in range(N + 1):
            t = t_start + i * dt
            
            if self.trajectory_func is not None:
                # Use modern trajectory system
                try:
                    pos, vel, acc = self.trajectory_func(t)
                    x_ref[0:3, i] = pos     # position [x, y, z]
                    x_ref[3:6, i] = vel     # velocity [vx, vy, vz]
                except Exception as e:
                    self.get_logger().warn(f"Trajectory function error: {e}, using fallback")
                    # Use fallback if trajectory function fails
                    pos, vel = self._fallback_figure8_trajectory(t)
                    x_ref[0:3, i] = pos
                    x_ref[3:6, i] = vel
            else:
                # Fallback to legacy figure-8
                pos, vel = self._fallback_figure8_trajectory(t)
                x_ref[0:3, i] = pos
                x_ref[3:6, i] = vel
                
        return x_ref
    
    def _fallback_figure8_trajectory(self, t):
        """Fallback figure-8 trajectory for when trajectory system is unavailable"""
        omega = 2 * np.pi * self.ref_frequency
        
        # Position components for figure-8 in NED frame
        pos = np.array([
            self.ref_amplitude_x * np.sin(omega * t),                    # x
            self.ref_amplitude_y * np.sin(2 * omega * t),               # y  
            -self.takeoff_height                                         # z (NED: negative for altitude above ground)
        ])
        
        # Velocity components (derivatives of position)
        vel = np.array([
            self.ref_amplitude_x * omega * np.cos(omega * t),           # vx
            self.ref_amplitude_y * 2 * omega * np.cos(2 * omega * t),   # vy
            0.0                                                          # vz
        ])
        
        return pos, vel

    def control_loop(self):
        """Main control loop with MPC"""
        current_time = time.time() - self.start_time
        
        if current_time < 2.0:
            # Phase 1: Startup - position control
            self.publish_offboard_control_mode_position()
            self.publish_position_setpoint(0.0, 0.0, 0.0)
            self.get_logger().info("🔄 Preparing MPC controller...")
            
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
            
        elif current_time < 15.0:
            # Phase 4: Hover and stabilize - position control
            self.publish_offboard_control_mode_position()
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            self.get_logger().info("🎯 Hovering - Preparing for MPC control")
            
        else:
            # Phase 5: MPC control with robust state management
            flight_time = current_time - 15.0
            current_time_stamp = time.time()
            dt = current_time_stamp - self.last_control_time
            self.last_control_time = current_time_stamp
            
            # Get current state (real or simulated)
            current_state, is_simulated = self.get_current_state()
            
            # Always use acceleration control mode for MPC
            self.publish_offboard_control_mode_acceleration()
            
            # Generate reference trajectory
            x_ref = self.generate_reference_trajectory(flight_time, self.mpc.N, self.mpc.dt)

            # Solve MPC with current state
            u_opt, x_pred = self.mpc.solve(current_state, x_ref)
            
            # MPC outputs accelerations in NED frame - use directly for PX4
            # No sign flipping needed - MPC dynamics already account for gravity
            u_cmd = np.array([u_opt[0], u_opt[1], u_opt[2]])
            
            # Update simulated state with the control command
            plan_dt = self.mpc.dt
            if dt > 0 and dt < 0.08:  # Reasonable dt range
                self.update_simulated_state(u_cmd, plan_dt)

            # Publish NED acceleration commands directly to PX4
            self.publish_acceleration_setpoint(u_cmd[0], u_cmd[1], u_cmd[2])

            # Log MPC performance every 2 seconds
            if int(flight_time) % 2 == 0 and flight_time > 0:
                # Calculate tracking errors
                pos_error = np.linalg.norm(current_state[:3] - x_ref[:3, 0])
                vel_error = np.linalg.norm(current_state[3:6] - x_ref[3:6, 0])
                
                state_source = "🤖 Simulated" if is_simulated else "📡 PX4 Feedback"
                
                self.get_logger().info(f"🎯 Real MPC {self.trajectory_config} Control (flight_time={flight_time:.1f}s):")
                self.get_logger().info(f"   📊 Reference: ({x_ref[0,0]:.2f}, {x_ref[1,0]:.2f}, {x_ref[2,0]:.2f}) m")
                self.get_logger().info(f"   📈 {state_source}: ({current_state[0]:.2f}, {current_state[1]:.2f}, {current_state[2]:.2f}) m")
                self.get_logger().info(f"   🎮 MPC NED accel: ({u_cmd[0]:.2f}, {u_cmd[1]:.2f}, {u_cmd[2]:.2f}) m/s²")
                self.get_logger().info(f"   ⚡ Position error: {pos_error:.3f} m")
                self.get_logger().info(f"   💨 Velocity error: {vel_error:.3f} m/s")
                if is_simulated:
                    self.get_logger().info(f"   ⚠️  Using simulated state (PX4 feedback unavailable)")
                else:
                    self.get_logger().info(f"   ✅ Using real PX4 position feedback")
            
            # Publish monitoring data for PlotJuggler (every cycle)
            self.publish_monitoring_data(current_state, x_ref, u_opt, flight_time, is_simulated)

    def publish_monitoring_data(self, current_state, x_ref, u_opt, flight_time, is_simulated):
        """Publish comprehensive monitoring data for PlotJuggler visualization"""
        current_time = self.get_clock().now()
        
        # Reference position
        ref_pos_msg = PointStamped()
        ref_pos_msg.header.stamp = current_time.to_msg()
        ref_pos_msg.header.frame_id = "map"
        ref_pos_msg.point.x = float(x_ref[0, 0])
        ref_pos_msg.point.y = float(x_ref[1, 0])
        ref_pos_msg.point.z = float(x_ref[2, 0])
        self.reference_position_pub.publish(ref_pos_msg)
        
        # Current position
        curr_pos_msg = PointStamped()
        curr_pos_msg.header.stamp = current_time.to_msg()
        curr_pos_msg.header.frame_id = "map"
        curr_pos_msg.point.x = float(current_state[0])
        curr_pos_msg.point.y = float(current_state[1])
        curr_pos_msg.point.z = float(current_state[2])
        self.current_position_pub.publish(curr_pos_msg)
        
        # Position error
        pos_error_msg = Vector3Stamped()
        pos_error_msg.header.stamp = current_time.to_msg()
        pos_error_msg.header.frame_id = "map"
        pos_error_msg.vector.x = float(x_ref[0, 0] - current_state[0])
        pos_error_msg.vector.y = float(x_ref[1, 0] - current_state[1])
        pos_error_msg.vector.z = float(x_ref[2, 0] - current_state[2])
        self.position_error_pub.publish(pos_error_msg)
        
        # Velocity error
        vel_error_msg = Vector3Stamped()
        vel_error_msg.header.stamp = current_time.to_msg()
        vel_error_msg.header.frame_id = "map"
        vel_error_msg.vector.x = float(x_ref[3, 0] - current_state[3])
        vel_error_msg.vector.y = float(x_ref[4, 0] - current_state[4])
        vel_error_msg.vector.z = float(x_ref[5, 0] - current_state[5])
        self.velocity_error_pub.publish(vel_error_msg)
        
        # MPC acceleration output
        mpc_output_msg = Vector3Stamped()
        mpc_output_msg.header.stamp = current_time.to_msg()
        mpc_output_msg.header.frame_id = "map"
        mpc_output_msg.vector.x = float(u_opt[0])
        mpc_output_msg.vector.y = float(u_opt[1])
        mpc_output_msg.vector.z = float(u_opt[2])
        self.mpc_output_pub.publish(mpc_output_msg)
        
        # Scalar metrics
        flight_time_msg = Float64()
        flight_time_msg.data = float(flight_time)
        self.flight_time_pub.publish(flight_time_msg)
        
        # Position error magnitude
        pos_error_magnitude = np.linalg.norm(current_state[:3] - x_ref[:3, 0])
        error_mag_msg = Float64()
        error_mag_msg.data = float(pos_error_magnitude)
        self.error_magnitude_pub.publish(error_mag_msg)
        
        # Velocity error magnitude
        vel_error_magnitude = np.linalg.norm(current_state[3:6] - x_ref[3:6, 0])
        vel_mag_msg = Float64()
        vel_mag_msg.data = float(vel_error_magnitude)
        self.velocity_magnitude_pub.publish(vel_mag_msg)
        
        # Control effort (acceleration magnitude)
        control_effort = np.linalg.norm(u_opt)
        control_msg = Float64()
        control_msg.data = float(control_effort)
        self.control_effort_pub.publish(control_msg)
        
        # Trajectory progress (normalized time in cycle)
        if hasattr(self, 'trajectory_func') and self.trajectory_func:
            # For periodic trajectories, calculate progress within cycle
            if 'figure8' in self.trajectory_config or 'circle' in self.trajectory_config:
                cycle_time = 2 * np.pi / 0.08  # Approximate cycle time
                progress = (flight_time % cycle_time) / cycle_time
            else:
                progress = flight_time / 60.0  # Normalize to 60 seconds
        else:
            progress = flight_time / 60.0
        
        progress_msg = Float64()
        progress_msg.data = float(progress)
        self.trajectory_progress_pub.publish(progress_msg)
        
        # Comprehensive trajectory info array
        traj_info_msg = Float64MultiArray()
        traj_info_msg.data = [
            float(x_ref[0, 0]),  # ref_x
            float(x_ref[1, 0]),  # ref_y
            float(x_ref[2, 0]),  # ref_z
            float(x_ref[3, 0]),  # ref_vx
            float(x_ref[4, 0]),  # ref_vy
            float(x_ref[5, 0]),  # ref_vz
            float(current_state[0]),  # curr_x
            float(current_state[1]),  # curr_y
            float(current_state[2]),  # curr_z
            float(current_state[3]),  # curr_vx
            float(current_state[4]),  # curr_vy
            float(current_state[5]),  # curr_vz
            float(u_opt[0]),     # accel_x
            float(u_opt[1]),     # accel_y
            float(u_opt[2]),     # accel_z
            float(pos_error_magnitude),  # pos_error_mag
            float(vel_error_magnitude),  # vel_error_mag
            float(control_effort),       # control_effort
            float(flight_time),          # flight_time
            float(1.0 if is_simulated else 0.0)  # is_simulated flag
        ]
        self.trajectory_info_pub.publish(traj_info_msg)


def main(args=None):
    print('🎯 Starting MPC Demo with CasADi and Configurable Trajectories...')
    
    # Handle trajectory listing request
    if args and '--list-trajectories' in args:
        if TRAJECTORIES_AVAILABLE:
            print("\n📋 Available trajectory configurations:")
            list_available_trajectories()
        else:
            print("❌ Trajectory definitions not available!")
        return
    
    if not CASADI_AVAILABLE:
        print("❌ Error: CasADi not available!")
        print("📦 Install CasADi with: pip install casadi")
        return
    
    # Show trajectory selection info
    selected_traj = os.getenv('TRAJECTORY_CONFIG', 'slow_figure8')
    print(f"🎯 Selected trajectory: {selected_traj}")
    
    if TRAJECTORIES_AVAILABLE:
        if selected_traj in TRAJECTORY_CONFIGS:
            config = TRAJECTORY_CONFIGS[selected_traj]
            print(f"   Type: {config['name']}")
            print(f"   Parameters: {config['params']}")
        else:
            print(f"   ⚠️  Unknown trajectory '{selected_traj}', will use fallback")
            print("   Use --list-trajectories to see available options")
    else:
        print("   ⚠️  Using fallback figure-8 (trajectory_definitions.py not available)")
    
    print(f"💡 To change trajectory: export TRAJECTORY_CONFIG='trajectory_name'")
    print(f"📋 To list all trajectories: python3 mpc_casadi_demo.py --list-trajectories")
    print()
    
    rclpy.init(args=args)
    mpc_demo = MPCDemo()
    
    try:
        rclpy.spin(mpc_demo)
    except KeyboardInterrupt:
        mpc_demo.get_logger().info('🛑 MPC demo stopped by user')
    finally:
        mpc_demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()