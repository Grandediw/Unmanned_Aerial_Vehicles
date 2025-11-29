#!/usr/bin/env python3
"""
MPC Hover Stability Test
========================
Simple hover test for MPC controller to check stability and tune parameters.
This version focuses only on hovering at a fixed position to validate MPC performance.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition
from std_msgs.msg import Float64, Header
from geometry_msgs.msg import PointStamped, Vector3Stamped
import time
import math
import numpy as np

try:
    import casadi as ca
    CASADI_AVAILABLE = True
except ImportError:
    CASADI_AVAILABLE = False

class MPCHoverController:
    """Simplified MPC Controller for hover testing"""
    
    def __init__(self, dt=0.1, N=10):
        if not CASADI_AVAILABLE:
            raise ImportError("CasADi is required for MPC. Install with: pip install casadi")
            
        self.dt = dt  # Time step
        self.N = N    # Prediction horizon
        
        # State and control dimensions
        self.n_states = 6  # [x, y, z, vx, vy, vz]
        self.n_controls = 3  # [ax, ay, az] (acceleration commands)
        
        # System constraints - CONSERVATIVE for stability testing
        self.v_max = 1.5    # Maximum velocity (m/s) - reduced for stability
        self.a_max = 1.0    # Maximum acceleration (m/s²) - reduced for stability
        self.z_min = -5.0   # Maximum altitude (negative in NED)
        self.z_max = -0.5   # Minimum altitude (negative in NED)
        
        # MPC weights - TUNED FOR HOVER STABILITY
        self.Q = np.diag([100.0, 100.0, 150.0, 20.0, 20.0, 30.0])  # State weights [pos, vel] - high position weight
        self.R = np.diag([1.0, 1.0, 1.0])                           # Control weights [acc] - higher for smoother control
        self.Qf = 2 * self.Q                                         # Terminal weights
        
        self._setup_mpc_problem()
        
    def _setup_mpc_problem(self):
        """Setup the MPC optimization problem using CasADi"""
        
        # Symbolic variables
        x = ca.SX.sym('x', self.n_states)      # States: [x, y, z, vx, vy, vz]
        u = ca.SX.sym('u', self.n_controls)    # Controls: [ax, ay, az]
        
        # System dynamics (double integrator model)
        x_dot = ca.vertcat(
            x[3],     # dx/dt = vx
            x[4],     # dy/dt = vy  
            x[5],     # dz/dt = vz
            u[0],     # dvx/dt = ax
            u[1],     # dvy/dt = ay
            u[2]      # dvz/dt = az
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
            lbw += [-10, -10, self.z_min, -self.v_max, -self.v_max, -self.v_max]
            ubw += [10, 10, self.z_max, self.v_max, self.v_max, self.v_max]
        
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
        
        # Solver options (tuned for stability)
        opts = {
            'print_time': False,
            'ipopt.print_level': 0,
            'ipopt.max_iter': 100,             # More iterations for stability
            'ipopt.tol': 1e-6,                 # Tighter tolerance for precision
            'ipopt.acceptable_tol': 1e-4,      # Acceptable tolerance
            'ipopt.acceptable_iter': 5,        # Accept after more iterations
            'ipopt.warm_start_init_point': 'yes'  # Use warm start
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
            success: Whether optimization succeeded
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
            
            # Check solver status
            stats = self.solver.stats()
            success = stats['success']
            
            # Extract solution
            w_opt = np.array(sol['x']).flatten()
            
            # Parse solution
            n_state_vars = self.n_states * (self.N + 1)
            X_opt = w_opt[:n_state_vars].reshape((self.n_states, self.N + 1))
            U_opt = w_opt[n_state_vars:].reshape((self.n_controls, self.N))
            
            # Update warm start only if successful
            if success:
                self.w0 = w_opt
            
            # Return first control action and predicted trajectory
            return U_opt[:, 0], X_opt, success
            
        except Exception as e:
            print(f"MPC solver failed: {e}")
            # Return zero control in case of failure
            return np.zeros(3), np.zeros((self.n_states, self.N + 1)), False

class MPCHoverTest(Node):
    def __init__(self):
        super().__init__('mpc_hover_test')
        
        if not CASADI_AVAILABLE:
            self.get_logger().error("❌ CasADi not available! Install with: pip install casadi")
            return
        
        # QoS profile for PX4 communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        
        # Monitoring publishers
        self.reference_position_pub = self.create_publisher(PointStamped, '/mpc_hover/reference_position', 10)
        self.current_position_pub = self.create_publisher(PointStamped, '/mpc_hover/current_position', 10)
        self.position_error_pub = self.create_publisher(Vector3Stamped, '/mpc_hover/position_error', 10)
        self.mpc_output_pub = self.create_publisher(Vector3Stamped, '/mpc_hover/mpc_acceleration', 10)
        self.error_magnitude_pub = self.create_publisher(Float64, '/mpc_hover/position_error_magnitude', 10)
        self.solver_status_pub = self.create_publisher(Float64, '/mpc_hover/solver_success', 10)
        
        # Subscribers
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
        
        # Hover target position (can be changed for testing)
        self.hover_position = np.array([0.0, 0.0, -self.takeoff_height])  # NED coordinates
        
        # Current state
        self.current_state = np.zeros(6)  # [x, y, z, vx, vy, vz]
        self.state_received = False
        
        # MPC Controller - conservative settings for hover test
        try:
            self.mpc = MPCHoverController(dt=0.1, N=12)  # 1.2s prediction horizon
            self.get_logger().info("✅ MPC Hover Controller initialized!")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize MPC: {e}")
            return
        
        # Statistics tracking
        self.solver_success_count = 0
        self.solver_total_count = 0
        self.max_position_error = 0.0
        self.avg_position_error = 0.0
        self.error_history = []
        
        # Timer for main control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        self.start_time = time.time()
        
        self.get_logger().info("🚁 MPC Hover Test Started!")
        self.get_logger().info(f"🎯 Target hover position: ({self.hover_position[0]:.1f}, {self.hover_position[1]:.1f}, {-self.hover_position[2]:.1f}) m")
        self.get_logger().info("🔧 Testing MPC stability for hovering...")
        
    def vehicle_status_callback(self, msg):
        """Get vehicle arming status"""
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        
    def position_callback(self, msg):
        """Get current state feedback"""
        if msg.xy_valid and msg.z_valid and msg.v_xy_valid and msg.v_z_valid:
            self.current_state = np.array([
                msg.x, msg.y, msg.z,      # Position in NED frame
                msg.vx, msg.vy, msg.vz    # Velocity in NED frame
            ])
            self.state_received = True
    
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
        """Publish offboard control mode for acceleration control"""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = False
        msg.velocity = False
        msg.acceleration = True
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
        """Publish acceleration setpoint"""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.acceleration = [ax, ay, az]
        msg.yaw = yaw
        # Set position and velocity to NaN for acceleration control
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        self.trajectory_setpoint_pub.publish(msg)
        
    def generate_hover_reference(self):
        """Generate constant hover reference trajectory"""
        x_ref = np.zeros((6, self.mpc.N + 1))
        
        # All points in reference trajectory are the same (hover)
        for i in range(self.mpc.N + 1):
            x_ref[0:3, i] = self.hover_position  # Position [x, y, z]
            x_ref[3:6, i] = np.zeros(3)          # Velocity [0, 0, 0]
                
        return x_ref

    def control_loop(self):
        """Main control loop with MPC hover test"""
        current_time = time.time() - self.start_time
        
        if current_time < 2.0:
            # Phase 1: Startup
            self.publish_offboard_control_mode_position()
            self.publish_position_setpoint(0.0, 0.0, 0.0)
            self.get_logger().info("🔄 Preparing MPC hover test...")
            
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
            self.get_logger().info(f"🚁 Taking off to {self.takeoff_height}m...")
            
        elif current_time < 15.0:
            # Phase 4: Stabilize with position control
            self.publish_offboard_control_mode_position()
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            if int(current_time) % 2 == 0:  # Log every 2 seconds
                self.get_logger().info("🎯 Stabilizing with position control...")
            
        else:
            # Phase 5: MPC Hover Test
            flight_time = current_time - 15.0
            
            if not self.state_received:
                self.get_logger().warn("⚠️  No position feedback, cannot start MPC")
                return
            
            # Switch to acceleration control
            self.publish_offboard_control_mode_acceleration()
            
            # Generate hover reference
            x_ref = self.generate_hover_reference()
            
            # Solve MPC
            u_opt, x_pred, success = self.mpc.solve(self.current_state, x_ref)
            
            # Track solver statistics
            self.solver_total_count += 1
            if success:
                self.solver_success_count += 1
            
            # Calculate error metrics
            pos_error_vec = self.current_state[:3] - self.hover_position
            pos_error = np.linalg.norm(pos_error_vec)
            vel_error = np.linalg.norm(self.current_state[3:6])
            
            # Update error statistics
            self.error_history.append(pos_error)
            if len(self.error_history) > 100:  # Keep last 100 samples
                self.error_history.pop(0)
            self.max_position_error = max(self.max_position_error, pos_error)
            self.avg_position_error = np.mean(self.error_history)
            
            # Publish acceleration commands
            self.publish_acceleration_setpoint(u_opt[0], u_opt[1], u_opt[2])
            
            # Log performance every 3 seconds
            if int(flight_time) % 3 == 0 and flight_time > 0:
                success_rate = (self.solver_success_count / self.solver_total_count) * 100
                
                self.get_logger().info(f"🎯 MPC Hover Test (t={flight_time:.1f}s):")
                self.get_logger().info(f"   📍 Current: ({self.current_state[0]:.3f}, {self.current_state[1]:.3f}, {self.current_state[2]:.3f}) m")
                self.get_logger().info(f"   🎯 Target:  ({self.hover_position[0]:.3f}, {self.hover_position[1]:.3f}, {self.hover_position[2]:.3f}) m")
                self.get_logger().info(f"   📏 Position error: {pos_error:.4f} m (avg: {self.avg_position_error:.4f} m, max: {self.max_position_error:.4f} m)")
                self.get_logger().info(f"   💨 Velocity: ({self.current_state[3]:.3f}, {self.current_state[4]:.3f}, {self.current_state[5]:.3f}) m/s")
                self.get_logger().info(f"   🎮 MPC accel: ({u_opt[0]:.3f}, {u_opt[1]:.3f}, {u_opt[2]:.3f}) m/s²")
                self.get_logger().info(f"   ✅ Solver success rate: {success_rate:.1f}% ({self.solver_success_count}/{self.solver_total_count})")
            
            # Publish monitoring data
            self.publish_monitoring_data(x_ref, u_opt, pos_error, success)

    def publish_monitoring_data(self, x_ref, u_opt, pos_error, solver_success):
        """Publish monitoring data for analysis"""
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
        curr_pos_msg.point.x = float(self.current_state[0])
        curr_pos_msg.point.y = float(self.current_state[1])
        curr_pos_msg.point.z = float(self.current_state[2])
        self.current_position_pub.publish(curr_pos_msg)
        
        # Position error vector
        pos_error_vec = self.current_state[:3] - x_ref[:3, 0]
        pos_error_msg = Vector3Stamped()
        pos_error_msg.header.stamp = current_time.to_msg()
        pos_error_msg.header.frame_id = "map"
        pos_error_msg.vector.x = float(pos_error_vec[0])
        pos_error_msg.vector.y = float(pos_error_vec[1])
        pos_error_msg.vector.z = float(pos_error_vec[2])
        self.position_error_pub.publish(pos_error_msg)
        
        # MPC output
        mpc_output_msg = Vector3Stamped()
        mpc_output_msg.header.stamp = current_time.to_msg()
        mpc_output_msg.header.frame_id = "map"
        mpc_output_msg.vector.x = float(u_opt[0])
        mpc_output_msg.vector.y = float(u_opt[1])
        mpc_output_msg.vector.z = float(u_opt[2])
        self.mpc_output_pub.publish(mpc_output_msg)
        
        # Error magnitude
        error_mag_msg = Float64()
        error_mag_msg.data = float(pos_error)
        self.error_magnitude_pub.publish(error_mag_msg)
        
        # Solver success flag
        solver_msg = Float64()
        solver_msg.data = float(1.0 if solver_success else 0.0)
        self.solver_status_pub.publish(solver_msg)


def main(args=None):
    print('🎯 Starting MPC Hover Stability Test...')
    
    if not CASADI_AVAILABLE:
        print("❌ Error: CasADi not available!")
        print("📦 Install CasADi with: pip install casadi")
        return
    
    rclpy.init(args=args)
    mpc_hover_test = MPCHoverTest()
    
    try:
        rclpy.spin(mpc_hover_test)
    except KeyboardInterrupt:
        mpc_hover_test.get_logger().info('🛑 MPC hover test stopped by user')
    finally:
        mpc_hover_test.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()