"""
12-State Rigid-Body MPC controller for ROS2 PX4 Offboard Example

Key Features:
- Phase 5: Full Rigid-Body MPC -> Torque + Thrust -> Body Rate commands
- State: [x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r] (12 states)
- Control: [uT, tau_x, tau_y, tau_z] (total thrust N, body torques Nm)
- Output: Converted to [rollrate, pitchrate, yawrate, thrust_norm] for PX4
- Framework: CasADi optimization with ROS2 integration
- Safety: Conservative parameters, emergency fallback, stability monitoring
- Learning: Optional GP-enhanced dynamics with async training
"""

import numpy as np
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Import PX4 message types
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleRatesSetpoint
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleAngularVelocity

# Check for required dependencies
try:
    import casadi as ca
    CASADI_AVAILABLE = True
except ImportError:
    CASADI_AVAILABLE = False
    print("❌ CasADi not available! Install with: pip install casadi")

# Check for optional GP dependency
try:
    from .simple_gp import SimpleQuadrotorGP, SimpleGPEnhancedMPC
    GP_AVAILABLE = True
    print("🧠 Simple GP dynamics learning available")
except (ImportError, ModuleNotFoundError):
    GP_AVAILABLE = False
    print("📖 GP learning not available (optional)")

# Import trajectory definitions
try:
    from .trajectory_definitions import (
        figure8_trajectory, sine_trajectory, circular_trajectory, 
        hover_trajectory, oval_trajectory, lemniscate_trajectory
    )
    TRAJECTORIES_AVAILABLE = True
except (ImportError, ModuleNotFoundError):
    TRAJECTORIES_AVAILABLE = False
    print("⚠️ Trajectory definitions not available")

def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles (radians) to quaternion [w, x, y, z]"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5) 
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return [w, x, y, z]

def wrap_angle(angle):
    """Wrap angle to [-π, π] range"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def wrap_angle_ca(angle):
    """CasADi-compatible angle wrapping"""
    return ca.fmod(angle + ca.pi, 2*ca.pi) - ca.pi

class QuadrotorMPC:
    """
    12-state rigid-body MPC for quadrotor (torque + thrust inputs).

    State x:
        [x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r]
        position (world, z-up),
        velocity (world),
        Euler angles (ZYX),
        body rates [p, q, r].

    Control u:
        [uT, tau_x, tau_y, tau_z]
        total thrust (N) along body z,
        body torques (Nm).

    Parameters P:
        [x0(12),
         x_ref, y_ref, z_ref, yaw_ref]
    """

    def __init__(
        self,
        dt: float = 0.02,
        N: int = 15,
        mass: float = 2.0,
        Jx: float = 0.0217,
        Jy: float = 0.0217,
        Jz: float = 0.04,
        use_gp: bool = False,
    ):
        if not CASADI_AVAILABLE:
            raise ImportError("CasADi is required for MPC. Install with: pip install casadi")

        self.dt = dt
        self.N = N

        # Physical parameters (match gz_x500 as closely as possible)
        self.m = mass
        self.Jx = Jx
        self.Jy = Jy
        self.Jz = Jz
        
        # GP learning (optional)
        self.use_gp = use_gp and GP_AVAILABLE
        if self.use_gp and GP_AVAILABLE:
            self.gp_dynamics = SimpleQuadrotorGP(max_data_points=800)
            self.gp_enhanced = SimpleGPEnhancedMPC(self.gp_dynamics, confidence_threshold=0.1)
            print("🧠 Simple GP-enhanced dynamics initialized")
        else:
            self.gp_dynamics = None
            self.gp_enhanced = None
            self.use_gp = False
            print("📖 Using nominal dynamics only")
        
        # State and control dimensions
        self.n_states = 12  # [x,y,z, vx,vy,vz, roll,pitch,yaw, p,q,r]
        self.n_controls = 4  # [uT, tau_x, tau_y, tau_z]

        # Build the MPC problem
        self._setup_mpc()
    
    # ------------------------------------------------------------------
    # MPC problem setup
    # ------------------------------------------------------------------
    def _setup_mpc(self):
        g = 9.81
        m = self.m
        Jx, Jy, Jz = self.Jx, self.Jy, self.Jz

        # ---------- State symbols ----------
        x     = ca.SX.sym('x')
        y     = ca.SX.sym('y')
        z     = ca.SX.sym('z')
        vx    = ca.SX.sym('vx')
        vy    = ca.SX.sym('vy')
        vz    = ca.SX.sym('vz')
        phi   = ca.SX.sym('phi')      # roll
        theta = ca.SX.sym('theta')    # pitch
        psi   = ca.SX.sym('psi')      # yaw
        p_rate = ca.SX.sym('p')       # body rate x
        q_rate = ca.SX.sym('q')       # body rate y
        r_rate = ca.SX.sym('r')       # body rate z

        states = ca.vertcat(
            x, y, z,
            vx, vy, vz,
            phi, theta, psi,
            p_rate, q_rate, r_rate
        )
        self.n_states = int(states.size1())

        # ---------- Control symbols ----------
        uT    = ca.SX.sym('uT')      # total thrust [N]
        tau_x = ca.SX.sym('tau_x')   # body torque x [Nm]
        tau_y = ca.SX.sym('tau_y')   # body torque y [Nm]
        tau_z = ca.SX.sym('tau_z')   # body torque z [Nm]

        controls = ca.vertcat(uT, tau_x, tau_y, tau_z)
        self.n_controls = int(controls.size1())

        # ---------- Rotation matrix body->world (ZYX) ----------
        Rx = ca.vertcat(
            ca.horzcat(1, 0, 0),
            ca.horzcat(0, ca.cos(phi), -ca.sin(phi)),
            ca.horzcat(0, ca.sin(phi),  ca.cos(phi)),
        )
        Ry = ca.vertcat(
            ca.horzcat( ca.cos(theta), 0, ca.sin(theta)),
            ca.horzcat( 0,             1, 0),
            ca.horzcat(-ca.sin(theta), 0, ca.cos(theta)),
        )
        Rz = ca.vertcat(
            ca.horzcat(ca.cos(psi), -ca.sin(psi), 0),
            ca.horzcat(ca.sin(psi),  ca.cos(psi), 0),
            ca.horzcat(0,            0,           1),
        )

        R = Rz @ Ry @ Rx

        # ---------- Translational dynamics ----------
        v = ca.vertcat(vx, vy, vz)
        F_body = ca.vertcat(0, 0, uT)
        F_world = R @ F_body
        g_world = ca.vertcat(0, 0, -g)

        p_dot = v
        v_dot = (1.0 / m) * F_world + g_world

        # ---------- Attitude kinematics (Euler rates from body rates) ----------
        # ZYX convention
        # [phi_dot, theta_dot, psi_dot]^T = W(phi,theta) * [p,q,r]^T
        W = ca.vertcat(
            ca.horzcat(1, ca.sin(phi)*ca.tan(theta),  ca.cos(phi)*ca.tan(theta)),
            ca.horzcat(0, ca.cos(phi),              -ca.sin(phi)),
            ca.horzcat(0, ca.sin(phi)/ca.cos(theta), ca.cos(phi)/ca.cos(theta)),
        )
        euler_dot = W @ ca.vertcat(p_rate, q_rate, r_rate)

        # ---------- Rotational dynamics ----------
        Jomega = ca.vertcat(Jx * p_rate, Jy * q_rate, Jz * r_rate)
        omega_cross_Jomega = ca.vertcat(
            q_rate * Jomega[2] - r_rate * Jomega[1],
            r_rate * Jomega[0] - p_rate * Jomega[2],
            p_rate * Jomega[1] - q_rate * Jomega[0],
        )
        tau = ca.vertcat(tau_x, tau_y, tau_z)

        omega_dot = ca.vertcat(
            (tau[0] - omega_cross_Jomega[0]) / Jx,
            (tau[1] - omega_cross_Jomega[1]) / Jy,
            (tau[2] - omega_cross_Jomega[2]) / Jz,
        )

        # ---------- Continuous dynamics RHS ----------
        rhs = ca.vertcat(
            p_dot,
            v_dot,
            euler_dot,
            omega_dot
        )

        # f(x,u) for MPC
        self.f = ca.Function('f', [states, controls], [rhs])

        # ==================================================================
        #  NLP variables: X (states) and U (controls)
        # ==================================================================
        X = ca.SX.sym('X', self.n_states, self.N + 1)
        U = ca.SX.sym('U', self.n_controls, self.N)

        # Parameters P = [x0(12), x_ref, y_ref, z_ref, yaw_ref]
        P = ca.SX.sym('P', self.n_states + 4)

        # Extract reference from P
        x_ref = P[self.n_states + 0]
        y_ref = P[self.n_states + 1]
        z_ref = P[self.n_states + 2]
        yaw_ref = P[self.n_states + 3]

        pos_ref = ca.vertcat(x_ref, y_ref, z_ref)

        # ---------- Weights - CONSERVATIVE for stable hovering ----------
        Q_pos   = ca.diag(ca.DM([12.0, 12.0, 18.0]))  # position tracking
        Q_vel   = ca.diag(ca.DM([3.0, 3.0, 4.0]))      # velocity damping
        Q_att   = ca.diag(ca.DM([2.0, 2.0, 1.5]))      # attitude error (roll, pitch, yaw)
        Q_rates = ca.diag(ca.DM([0.3, 0.3, 0.3]))      # body rates damping

        # hover thrust in Newtons
        uT_hover = m * g
        # control penalty - INCREASED to prevent aggressive commands
        R_u   = ca.diag(ca.DM([0.3, 0.1, 0.1, 0.1]))  # [thrust, τx, τy, τz] - higher penalties
        # explicit penalty on (uT - hover) to strongly prefer hovering thrust
        w_thrust_dev = 0.5  # Increased from 0.2

        obj = 0
        g_constr = []

        # Initial condition constraint: X[:,0] == x0
        x0_param = P[0:self.n_states]
        g_constr.append(X[:, 0] - x0_param)

        # ---------- Stage cost + dynamics ----------
        for k in range(self.N):
            xk = X[:, k]
            uk = U[:, k]

            # unpack state
            pos_k   = xk[0:3]
            vel_k   = xk[3:6]
            phi_k   = xk[6]
            theta_k = xk[7]
            psi_k   = xk[8]
            rates_k = xk[9:12]

            uT_k    = uk[0]

            # errors
            pos_err = pos_k - pos_ref
            vel_err = vel_k  # want velocity ~ 0

            # attitude error: roll, pitch to 0; yaw to yaw_ref
            roll_err  = wrap_angle_ca(phi_k)
            pitch_err = wrap_angle_ca(theta_k)
            yaw_err   = wrap_angle_ca(psi_k - yaw_ref)
            att_err   = ca.vertcat(roll_err, pitch_err, yaw_err)

            rates_err = rates_k  # want body rates ~ 0

            # CRITICAL FIX: Only penalize torques, not absolute thrust
            # Thrust is handled by thrust_dev term below
            torque_err = uk[1:4]  # Only [τx, τy, τz]

            thrust_dev = uT_k - uT_hover

            # quadratic costs
            obj += ca.mtimes([pos_err.T,   Q_pos,   pos_err])
            obj += ca.mtimes([vel_err.T,   Q_vel,   vel_err])
            obj += ca.mtimes([att_err.T,   Q_att,   att_err])
            obj += ca.mtimes([rates_err.T, Q_rates, rates_err])
            # Torque penalty only (not thrust)
            R_torque = ca.diag(ca.DM([0.1, 0.1, 0.1]))  # [τx, τy, τz]
            obj += ca.mtimes([torque_err.T,  R_torque,  torque_err])
            # Thrust deviation from hover (this is the correct thrust penalty)
            obj += w_thrust_dev * thrust_dev * thrust_dev

            # Discrete dynamics (forward Euler)
            x_next = xk + self.dt * self.f(xk, uk)
            g_constr.append(X[:, k+1] - x_next)

        # ---------- Terminal cost (stronger) ----------
        xN = X[:, -1]
        posN   = xN[0:3]
        velN   = xN[3:6]
        phiN   = xN[6]
        thetaN = xN[7]
        psiN   = xN[8]
        ratesN = xN[9:12]

        pos_errN = posN - pos_ref
        vel_errN = velN
        roll_errN  = wrap_angle_ca(phiN)
        pitch_errN = wrap_angle_ca(thetaN)
        yaw_errN   = wrap_angle_ca(psiN - yaw_ref)
        att_errN   = ca.vertcat(roll_errN, pitch_errN, yaw_errN)

        obj += 2.5 * ca.mtimes([pos_errN.T, Q_pos, pos_errN])  # Terminal position
        obj += 1.5 * ca.mtimes([vel_errN.T, Q_vel, vel_errN])  # Terminal velocity
        obj += 1.5 * ca.mtimes([att_errN.T, Q_att, att_errN])  # Terminal attitude
        obj += 0.8 * ca.mtimes([ratesN.T,   Q_rates, ratesN])  # Terminal rates

        # ---------- Pack NLP ----------
        OPT_variables = ca.vertcat(
            ca.reshape(X, -1, 1),
            ca.reshape(U, -1, 1)
        )

        nlp_prob = {
            'f': obj,
            'x': OPT_variables,
            'g': ca.vertcat(*g_constr),
            'p': P
        }

        # ---------- Solver options - BALANCED for performance and feasibility ----------
        opts = {
            'ipopt': {
                'max_iter': 80,
                'print_level': 0,
                'acceptable_tol': 2e-3,
                'acceptable_obj_change_tol': 1e-4,
                'tol': 2e-3,
                'warm_start_init_point': 'yes',
                'mu_strategy': 'adaptive',
                'acceptable_iter': 3,
                'nlp_scaling_method': 'gradient-based',
            },
            'print_time': 0
        }

        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

        # ---------- Bounds ----------
        self.lbg = 0  # equality constraints
        self.ubg = 0

        self.lbx = []
        self.ubx = []

        # State bounds (repeated N+1 times) - RELAXED to prevent infeasibility
        # [x, y, z]
        pos_bounds = [-50.0, -50.0, -1.0,  50.0, 50.0, 20.0]  # min/max interleaved below
        # [vx, vy, vz]
        vel_bounds = [-15.0, -15.0, -15.0,  15.0, 15.0, 15.0]
        # [phi, theta] (roll, pitch)
        att_bounds = [-1.2, -1.2,  1.2, 1.2]  # yaw unbounded (~70 deg)
        # [p, q, r]
        rate_bounds = [-10.0, -10.0, -10.0,  10.0, 10.0, 10.0]

        for _ in range(self.N + 1):
            # position
            self.lbx += pos_bounds[0:3]
            self.ubx += pos_bounds[3:6]
            # velocity
            self.lbx += vel_bounds[0:3]
            self.ubx += vel_bounds[3:6]
            # roll, pitch, yaw
            self.lbx += [att_bounds[0], att_bounds[1], -ca.inf]
            self.ubx += [att_bounds[2], att_bounds[3],  ca.inf]
            # p, q, r
            self.lbx += rate_bounds[0:3]
            self.ubx += rate_bounds[3:6]

        # Control bounds (repeated N times) - CONSERVATIVE for stability
        mg = m * g
        uT_min = 0.3 * mg  # Minimum thrust (5.9N)
        uT_max = 1.2 * mg  # Maximum thrust (23.5N) - REDUCED to prevent skyrocketing

        tau_max_roll  = 0.8  # Roll torque limit
        tau_max_pitch = 0.8  # Pitch torque limit
        tau_max_yaw   = 0.4  # Yaw torque limit

        for _ in range(self.N):
            self.lbx += [uT_min, -tau_max_roll, -tau_max_pitch, -tau_max_yaw]
            self.ubx += [uT_max,  tau_max_roll,  tau_max_pitch,  tau_max_yaw]

        self.lbx = np.array(self.lbx, dtype=np.float64)
        self.ubx = np.array(self.ubx, dtype=np.float64)

    # ------------------------------------------------------------------
    # Solve MPC
    # ------------------------------------------------------------------
    def solve(
        self,
        current_state: np.ndarray,
        target_pos: np.ndarray,
        target_yaw: float = 0.0,
        current_rates: np.ndarray | None = None,
    ):
        """
        Solve MPC for given state and target.

        current_state: shape (9,)
            [x,y,z, vx,vy,vz, roll,pitch,yaw]
        current_rates: shape (3,) or None
            [p,q,r]; if None, assumed zero.
        target_pos: shape (3,)
            [x_ref,y_ref,z_ref]
        target_yaw: float
            desired yaw [rad]
        """
        if current_rates is None:
            current_rates = np.zeros(3, dtype=np.float64)
        else:
            current_rates = np.asarray(current_rates, dtype=np.float64)

        current_state = np.asarray(current_state, dtype=np.float64)
        target_pos    = np.asarray(target_pos,    dtype=np.float64)

        # Clamp velocities and attitudes a bit for numerical stability
        current_state[3:6] = np.clip(current_state[3:6], -6.0, 6.0)
        current_state[6]   = wrap_angle(current_state[6])
        current_state[7]   = wrap_angle(current_state[7])
        current_state[8]   = wrap_angle(current_state[8])
        target_yaw         = wrap_angle(target_yaw)

        # Full 12-state vector for MPC
        x_full = np.concatenate([current_state, current_rates], axis=0)

        # Clamp target position to reasonable bounds
        current_pos = current_state[0:3]
        pos_error = target_pos - current_pos
        if np.linalg.norm(pos_error) > 4.0:
            # Limit target within 4 m of current position
            direction = pos_error / (np.linalg.norm(pos_error) + 1e-6)
            target_pos = current_pos + 4.0 * direction

        target_pos = np.clip(target_pos, [-15.0, -15.0, 0.2],
                                       [ 15.0,  15.0, 8.0])

        # Build parameters P
        P_val = np.concatenate([x_full, target_pos, np.array([target_yaw])], axis=0)

        # ---------- Initial guess ----------
        num_x = (self.N + 1) * self.n_states
        num_u = self.N * self.n_controls
        x0 = np.zeros(num_x + num_u, dtype=np.float64)

        # States: repeat current state
        for k in range(self.N + 1):
            idx = k * self.n_states
            x0[idx:idx + self.n_states] = x_full

        # Controls: start from hover (uT = mg) and zero torques
        mg = self.m * 9.81
        u_hover = np.array([mg, 0.0, 0.0, 0.0], dtype=np.float64)
        for k in range(self.N):
            idx_u = num_x + k * self.n_controls
            x0[idx_u:idx_u + self.n_controls] = u_hover

        # ---------- Solve NLP ----------
        try:
            sol = self.solver(
                x0=x0,
                lbx=self.lbx,
                ubx=self.ubx,
                lbg=self.lbg,
                ubg=self.ubg,
                p=P_val,
            )

            stats = self.solver.stats()
            if not stats.get('success', False):
                print(f"❌ MPC solver failed: {stats.get('return_status', 'unknown')}")
                return None, None

            sol_x = sol['x'].full().flatten()

            # Extract optimal U and X
            idx_u_start = (self.N + 1) * self.n_states
            U_opt = sol_x[idx_u_start:idx_u_start + self.N * self.n_controls]
            U_opt = U_opt.reshape(self.N, self.n_controls)

            X_opt = sol_x[0:idx_u_start].reshape(self.N + 1, self.n_states)

            u_first = U_opt[0, :].copy()

            # Final safety clamp on controls - CONSERVATIVE
            uT_min = 0.3 * mg
            uT_max = 1.2 * mg
            u_first[0] = float(np.clip(u_first[0], uT_min, uT_max))

            tau_max_roll  = 0.8
            tau_max_pitch = 0.8
            tau_max_yaw   = 0.4
            u_first[1] = float(np.clip(u_first[1], -tau_max_roll,  tau_max_roll))
            u_first[2] = float(np.clip(u_first[2], -tau_max_pitch, tau_max_pitch))
            u_first[3] = float(np.clip(u_first[3], -tau_max_yaw,   tau_max_yaw))

            return u_first, X_opt

        except Exception as e:
            print(f"❌ MPC solver exception: {e}")
            return None, None


def torque_input_to_px4_rates(
    u_mpc: np.ndarray,
    mass: float = 2.0,
    Jx: float = 0.0217,
    Jy: float = 0.0217,
    Jz: float = 0.04,
    Kp_att: float = 5.0,  # Reduced from 8.0 for smoother control
    current_rates: np.ndarray | None = None
):
    """
    Convert MPC torque+thrust outputs to PX4 body rate commands.
    
    Args:
        u_mpc: [uT (N), tau_x (Nm), tau_y (Nm), tau_z (Nm)]
        mass: quadrotor mass (kg)
        Jx, Jy, Jz: moments of inertia (kg·m²)
        Kp_att: proportional gain for attitude control (rad/s per rad/s²)
        current_rates: [p, q, r] current body rates (rad/s), or None
        
    Returns:
        rollrate_cmd, pitchrate_cmd, yawrate_cmd, thrust_norm
    """
    g = 9.81
    
    # Extract MPC outputs
    uT = float(u_mpc[0])      # Total thrust (N)
    tau_x = float(u_mpc[1])   # Roll torque (Nm)
    tau_y = float(u_mpc[2])   # Pitch torque (Nm)
    tau_z = float(u_mpc[3])   # Yaw torque (Nm)
    
    # Normalize thrust for PX4 (expects 0.0-1.0 range where ~0.5 = hover)
    # Note: PX4 thrust mapping is NOT linear - empirical testing shows:
    # - 0.5 normalized ≈ hover thrust
    # - Safe range: 0.3-0.8 for stable flight
    thrust_norm = uT / (mass * g)
    # Safety clamping with more headroom for proper hover
    thrust_norm = float(np.clip(thrust_norm, 0.30, 0.80))
    
    # Convert torques to angular accelerations
    # tau = J * alpha  =>  alpha = tau / J
    p_ddot = tau_x / Jx  # roll angular acceleration (rad/s²)
    q_ddot = tau_y / Jy  # pitch angular acceleration (rad/s²)
    r_ddot = tau_z / Jz  # yaw angular acceleration (rad/s²)
    
    # Simplified feedforward control (no angular velocity feedback available)
    # Convert angular accelerations directly to rate commands
    # Using smaller gain since we don't have damping feedback
    dt_control = 0.05  # Effective time constant
    
    rollrate_cmd = p_ddot * dt_control * Kp_att
    pitchrate_cmd = q_ddot * dt_control * Kp_att
    yawrate_cmd = r_ddot * dt_control * Kp_att
    
    # Safety limits on rate commands (rad/s) - RELAXED for better authority
    rollrate_cmd = float(np.clip(rollrate_cmd, -3.0, 3.0))    # ±172°/s
    pitchrate_cmd = float(np.clip(pitchrate_cmd, -3.0, 3.0))
    yawrate_cmd = float(np.clip(yawrate_cmd, -2.0, 2.0))      # ±115°/s
    
    return rollrate_cmd, pitchrate_cmd, yawrate_cmd, thrust_norm


class QuadrotorController(Node):
    """ROS2 Node for MPC-based quadrotor control with enhanced safety"""
    
    def __init__(self, takeoff_height=3.0):
        super().__init__('quadrotor_mpc_full')
        
        # === CONFIGURATION ===
        self.takeoff_height = takeoff_height  # Configurable height
        self.control_frequency = 50.0  # Hz
        self.dt = 1.0 / self.control_frequency
        
        # === INITIALIZE MPC ===
        self.mpc = QuadrotorMPC(dt=self.dt, N=15, use_gp=False)  # Shorter horizon for stability
        
        # === SYSTEM STATE ===
        self.phase = 'initialization'
        self.phase_start_time = None
        
        # Vehicle state
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_velocity = np.array([0.0, 0.0, 0.0])
        self.current_attitude = np.array([0.0, 0.0, 0.0])  # roll, pitch, yaw
        self.current_angular_velocity = np.array([0.0, 0.0, 0.0])
        self.vehicle_status = None
        
        # Setpoints and references
        self.position_setpoint = np.array([0.0, 0.0, self.takeoff_height])
        self.yaw_setpoint = 0.0
        
        # Trajectory tracking
        self.trajectory_type = 'hover'
        self.trajectory_scale = 1.0
        self.trajectory_start_time = None
        self.hover_position = np.array([0.0, 0.0, self.takeoff_height])
        
        # Safety and monitoring  
        self.last_position_time = time.time()
        self.last_angular_velocity_time = time.time()
        self.position_timeout = 2.0  # seconds
        self.max_position_error = 10.0  # meters - relaxed threshold
        self.emergency_landing = False
        self.consecutive_failures = 0
        self.max_failures = 5
        
        # === QOS PROFILES ===
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # === SUBSCRIBERS ===
        self.position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.position_callback, qos_profile)
            
        self.attitude_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self.attitude_callback, qos_profile)
            
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.status_callback, qos_profile)
            
        # NOTE: vehicle_angular_velocity is NOT published by PX4 over DDS
        # We'll use a simpler control approach without angular velocity feedback
        
        # === PUBLISHERS ===
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
            
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
            
        self.vehicle_rates_setpoint_pub = self.create_publisher(
            VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos_profile)
        
        # === TIMERS ===
        self.control_timer = self.create_timer(self.dt, self.control_callback)
        self.offboard_heartbeat = self.create_timer(0.1, self.publish_offboard_control_mode)
        
        # === INITIALIZATION ===
        self.get_logger().info('🚀 12-State Rigid-Body MPC Controller initialized')
        self.get_logger().info(f'   State: [x,y,z, vx,vy,vz, φ,θ,ψ] (9D - no angular vel feedback)')
        self.get_logger().info(f'   Control: [uT(N), τx,τy,τz(Nm)] → body rates')
        self.get_logger().info(f'   ⚠️  Angular velocity feedback NOT available from PX4')
        self.get_logger().info(f'   Takeoff height: {self.takeoff_height}m')
        self.get_logger().info(f'   Control frequency: {self.control_frequency} Hz')
        self.get_logger().info(f'   MPC horizon: {self.mpc.N} steps')

    def position_callback(self, msg):
        """Handle position updates from PX4"""
        self.current_position = np.array([msg.x, msg.y, -msg.z])  # NED to ENU
        self.current_velocity = np.array([msg.vx, msg.vy, -msg.vz])
        self.last_position_time = time.time()

    def attitude_callback(self, msg):
        """Handle attitude updates from PX4"""
        # Convert quaternion to Euler (PX4 uses NED, we use ENU-like)
        q = [msg.q[0], msg.q[1], msg.q[2], msg.q[3]]  # w, x, y, z
        
        # Convert to Euler angles (roll, pitch, yaw)
        roll = math.atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), 
                         1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]))
        pitch = math.asin(2.0 * (q[0] * q[2] - q[3] * q[1]))
        yaw = math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 
                        1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]))
        
        self.current_attitude = np.array([roll, pitch, yaw])

    # NOTE: angular_velocity_callback removed - topic not published by PX4

    def status_callback(self, msg):
        """Handle vehicle status updates"""
        self.vehicle_status = msg

    def publish_offboard_control_mode(self):
        """Publish offboard control mode message"""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False  
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = True  # We want body rate control
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_rates_setpoint(self, rollrate=0.0, pitchrate=0.0, yawrate=0.0, thrust=0.5):
        """Publish body rate and thrust setpoint"""
        msg = VehicleRatesSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # Body rates in rad/s - allow higher rates for better control
        msg.roll = float(np.clip(rollrate, -3.0, 3.0))
        msg.pitch = float(np.clip(pitchrate, -3.0, 3.0))
        msg.yaw = float(np.clip(yawrate, -2.0, 2.0))
        
        # Normalized thrust (0.0 to 1.0)
        normalized_thrust = np.clip(thrust, 0.0, 1.0)
        msg.thrust_body[0] = 0.0
        msg.thrust_body[1] = 0.0
        msg.thrust_body[2] = -normalized_thrust  # Negative for upward thrust in NED
        
        self.vehicle_rates_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Publish vehicle command"""
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def arm_vehicle(self):
        """Send arm command to vehicle"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("🔓 Arm command sent")

    def disarm_vehicle(self):
        """Send disarm command to vehicle"""  
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info("🔒 Disarm command sent")

    def enable_offboard_mode(self):
        """Enable offboard mode"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("🎮 Offboard mode enabled")

    def _fallback_pd_control(self, target_pos, target_yaw=0.0):
        """Simple PD fallback controller when MPC fails"""
        if self.emergency_landing:
            return np.array([0.0, 0.0, 0.0, 0.4])  # Emergency landing
            
        pos_error = target_pos - self.current_position
        vel_error = -self.current_velocity  # target velocity is zero
        yaw_error = wrap_angle(target_yaw - self.current_attitude[2])
        
        # Very conservative PD gains - much smaller to prevent divergence
        kp_pos = 0.05  # Much smaller proportional gain
        kd_pos = 0.02  # Much smaller derivative gain
        kp_yaw = 0.1
        
        # Limit position error to prevent large commands
        pos_error = np.clip(pos_error, -2.0, 2.0)  # Limit to ±2m
        vel_error = np.clip(vel_error, -1.0, 1.0)  # Limit velocity error
        
        roll_cmd = kp_pos * pos_error[1] + kd_pos * vel_error[1]   # y error -> roll
        pitch_cmd = -(kp_pos * pos_error[0] + kd_pos * vel_error[0])  # x error -> pitch (negative)
        yaw_rate_cmd = kp_yaw * yaw_error
        thrust_cmd = 0.75 + 0.1 * pos_error[2]  # z error -> thrust (smaller gain)
        
        # Strict safety limits on output commands
        roll_cmd = np.clip(roll_cmd, -0.1, 0.1)     # Very small attitude commands
        pitch_cmd = np.clip(pitch_cmd, -0.1, 0.1)
        yaw_rate_cmd = np.clip(yaw_rate_cmd, -0.2, 0.2)
        thrust_cmd = np.clip(thrust_cmd, 0.6, 0.9)   # Safe thrust range
        
        return np.array([roll_cmd, pitch_cmd, yaw_rate_cmd, thrust_cmd])

    def update_mpc_control(self, target_pos, target_yaw=0.0):
        """Update control using 12-state MPC with torque+thrust, then convert to PX4 rates"""
        if self.emergency_landing:
            return self._fallback_pd_control(target_pos, target_yaw)
        
        # Construct current state for MPC (9 states: position, velocity, attitude)
        current_state = np.concatenate([
            self.current_position, self.current_velocity, self.current_attitude])
        
        # Call MPC solver with current body rates
        u_mpc, X_opt = self.mpc.solve(
            current_state=current_state,
            target_pos=target_pos,
            target_yaw=target_yaw,
            current_rates=self.current_angular_velocity
        )
        
        if u_mpc is not None:
            self.consecutive_failures = 0
            
            # Log MPC output periodically (every 2 seconds)
            if not hasattr(self, '_last_debug_time'):
                self._last_debug_time = time.time()
            
            if time.time() - self._last_debug_time > 2.0:
                self.get_logger().info(f"🔧 MPC Output: thrust={u_mpc[0]:.2f}N, τ=[{u_mpc[1]:.3f}, {u_mpc[2]:.3f}, {u_mpc[3]:.3f}]Nm")
                self._last_debug_time = time.time()
            
            # CRITICAL: Emergency thrust limiting to prevent skyrocketing
            mg = self.mpc.m * 9.81
            if u_mpc[0] > 1.15 * mg:  # More than 1.15x hover thrust
                self.get_logger().warn(f"⚠️ MPC thrust too high: {u_mpc[0]:.1f}N, clamping to {1.15*mg:.1f}N")
                u_mpc[0] = 1.15 * mg
            if u_mpc[0] < 0.7 * mg:  # Less than 0.7x hover thrust
                self.get_logger().warn(f"⚠️ MPC thrust too low: {u_mpc[0]:.1f}N, raising to {0.7*mg:.1f}N")
                u_mpc[0] = 0.7 * mg
            
            # Convert MPC torque+thrust output to PX4 body rate commands
            # Note: No angular velocity feedback available from PX4
            rollrate_cmd, pitchrate_cmd, yawrate_cmd, thrust_norm = \
                torque_input_to_px4_rates(
                    u_mpc,
                    mass=self.mpc.m,
                    Jx=self.mpc.Jx,
                    Jy=self.mpc.Jy,
                    Jz=self.mpc.Jz,
                    current_rates=None  # Not available
                )
            
            # CRITICAL: Additional thrust safety limiting (final check)
            thrust_norm = float(np.clip(thrust_norm, 0.35, 0.65))
            
            # Log converted commands
            if time.time() - self._last_debug_time > 1.9:  # Log right after MPC output
                self.get_logger().info(f"📤 PX4 Commands: rates=[{rollrate_cmd:.3f}, {pitchrate_cmd:.3f}, {yawrate_cmd:.3f}]rad/s, thrust={thrust_norm:.3f}")
            
            # Return as array compatible with old interface
            return np.array([rollrate_cmd, pitchrate_cmd, yawrate_cmd, thrust_norm])
        else:
            self.consecutive_failures += 1
            if self.consecutive_failures >= self.max_failures:
                self.get_logger().error(f"❌ MPC failed {self.max_failures} times - Emergency landing!")
                self.emergency_landing = True
                
            self.get_logger().warn("⚠️ MPC failed - using PD fallback")
            return self._fallback_pd_control(target_pos, target_yaw)

    def get_trajectory_point(self, t):
        """Get trajectory reference at time t"""
        if not TRAJECTORIES_AVAILABLE:
            return np.array([0.0, 0.0, 0.0]), 0.0  # hover
            
        if self.trajectory_type == 'hover':
            return hover_trajectory(t, scale=self.trajectory_scale)
        elif self.trajectory_type == 'figure8':
            return figure8_trajectory(t, scale=self.trajectory_scale)
        elif self.trajectory_type == 'sine':
            return sine_trajectory(t, scale=self.trajectory_scale) 
        elif self.trajectory_type == 'circular':
            return circular_trajectory(t, scale=self.trajectory_scale)
        elif self.trajectory_type == 'oval':
            return oval_trajectory(t, scale=self.trajectory_scale)
        elif self.trajectory_type == 'lemniscate':
            return lemniscate_trajectory(t, scale=self.trajectory_scale)
        else:
            return hover_trajectory(t, scale=self.trajectory_scale)

    def set_trajectory(self, trajectory_type='figure8', scale=1.0):
        """Set the trajectory to follow"""
        self.trajectory_type = trajectory_type
        self.trajectory_scale = scale
        self.trajectory_start_time = time.time()
        self.get_logger().info(f"🎯 Trajectory set: {trajectory_type} (scale: {scale})")

    def control_callback(self):
        """Main control loop callback"""
        current_time = time.time()
        
        # Check for position timeout
        if current_time - self.last_position_time > self.position_timeout:
            self.get_logger().error("❌ Position timeout - Emergency landing!")
            self.emergency_landing = True

        # === PHASE MANAGEMENT ===
        if self.phase == 'initialization':
            if self.phase_start_time is None:
                self.phase_start_time = current_time
            
            # Publish conservative setpoint during initialization
            self.publish_rates_setpoint(0.0, 0.0, 0.0, 0.0)  # Zero thrust during init
            
            if current_time - self.phase_start_time > 2.0:
                self.get_logger().info("🔄 Phase: Arming")
                self.phase = 'arming'
                self.phase_start_time = current_time
                self.arm_vehicle()
                
        elif self.phase == 'arming':
            self.publish_rates_setpoint(0.0, 0.0, 0.0, 0.0)  # Zero thrust while arming
            
            if current_time - self.phase_start_time > 2.0:
                self.get_logger().info("🚀 Phase: Takeoff")
                self.phase = 'takeoff'
                self.phase_start_time = current_time
                self.enable_offboard_mode()
                
        elif self.phase == 'takeoff':
            # Gentle takeoff with constant sufficient thrust
            takeoff_thrust = 0.75  # Constant gentle climb thrust
            self.publish_rates_setpoint(0.0, 0.0, 0.0, takeoff_thrust)
            
            # Log progress periodically
            if int(current_time) % 2 == 0:
                altitude = self.current_position[2]
                self.get_logger().info(f"📈 Takeoff: altitude={altitude:.2f}m, target={self.takeoff_height:.1f}m, thrust={takeoff_thrust:.2f}")
            
            # Transition to hover when reaching target altitude (with margin)
            if self.current_position[2] > (self.takeoff_height - 0.5):
                self.get_logger().info(f"🎯 Phase: Hover stabilization at {self.current_position[2]:.1f}m")
                self.phase = 'hover'
                self.phase_start_time = current_time
                self.hover_position = self.current_position.copy()  # Remember where we reached
                
        elif self.phase == 'hover':
            # Phase 4: Extended hover and stabilize with MPC
            hover_duration = current_time - self.phase_start_time
            
            # Log hover progress
            if int(current_time) % 2 == 0:
                self.get_logger().info(f"🎯 Hovering {hover_duration:.1f}s - MPC stabilization")
            
            # Target position: maintain current hover position
            target_position = self.hover_position.copy()
            target_position[2] = self.takeoff_height  # Ensure correct altitude
            
            u_control = self.update_mpc_control(target_position, 0.0)
            
            # Trust MPC during hover - no artificial rate limiting
            if u_control is not None:
                self.publish_rates_setpoint(u_control[0], u_control[1], u_control[2], u_control[3])
            else:
                # Fallback if MPC fails
                self.publish_rates_setpoint(0.0, 0.0, 0.0, 0.55)
                
            # Check if stabilized (small position error)
            pos_error = np.linalg.norm(target_position - self.current_position)
            vel_magnitude = np.linalg.norm(self.current_velocity)
            
            if hover_duration > 8.0 and pos_error < 0.5 and vel_magnitude < 0.3:
                self.phase = 'mpc_control'
                self.phase_start_time = current_time
                self.get_logger().info(f"✅ Stabilized! pos_err={pos_error:.2f}m, transitioning to trajectory control")
            elif hover_duration > 15.0:
                # Force transition after 15s even if not perfectly stable
                self.phase = 'mpc_control'
                self.phase_start_time = current_time
                self.get_logger().warn(f"⚠️ Forcing transition after 15s (pos_err={pos_error:.2f}m)")
            
        elif self.phase == 'mpc_control':
            # Phase 5: Main MPC trajectory following
            if self.trajectory_start_time is None:
                self.trajectory_start_time = current_time
            
            trajectory_time = current_time - self.trajectory_start_time
            trajectory_offset, target_yaw = self.get_trajectory_point(trajectory_time)
            
            # The MPC trajectory is relative to hover position
            target_position = np.array([
                trajectory_offset[0] + self.hover_position[0],  # x offset
                trajectory_offset[1] + self.hover_position[1],  # y offset 
                trajectory_offset[2] + self.takeoff_height  # z offset + hover altitude
            ])
            
            self.position_setpoint = target_position
            
            # Emergency position safety checks
            pos_error = np.linalg.norm(target_position - self.current_position)
            altitude = self.current_position[2]
            
            # Emergency conditions
            if pos_error > self.max_position_error or altitude > 15.0 or altitude < -1.0:
                self.get_logger().error(f"❌ CRITICAL: pos_error={pos_error:.1f}m, alt={altitude:.1f}m - Emergency landing!")
                self.emergency_landing = True
                
            # Force hover mode for large errors to prevent MPC infeasibility
            if pos_error > 2.0:
                self.get_logger().warn(f"⚠️ LARGE POSITION ERROR: {pos_error:.1f}m - Forcing hover mode!")
                target_position = self.current_position.copy()  # Hover at current position
                target_position[2] = max(1.0, min(target_position[2], 5.0))  # Safe altitude
                target_yaw = 0.0
            
            u_control = self.update_mpc_control(target_position, target_yaw)
            self.publish_rates_setpoint(u_control[0], u_control[1], u_control[2], u_control[3])

            # Periodically log status
            if int(current_time) % 5 == 0:  # Every 5 seconds
                self.get_logger().info(f"📍 Pos: [{self.current_position[0]:.1f}, {self.current_position[1]:.1f}, {self.current_position[2]:.1f}]m")
                self.get_logger().info(f"🎯 Target: [{target_position[0]:.1f}, {target_position[1]:.1f}, {target_position[2]:.1f}]m")
                self.get_logger().info(f"🎮 Control: [{u_control[0]:.3f}, {u_control[1]:.3f}, {u_control[2]:.3f}, {u_control[3]:.3f}]")
                
        # Emergency landing mode
        if self.emergency_landing:
            # Immediate stabilization: zero rates with controlled descent
            if not hasattr(self, 'emergency_start_time'):
                self.emergency_start_time = current_time
                self.get_logger().error("🚨 EMERGENCY MODE ACTIVATED - Stabilizing first, then descending")
            
            emergency_duration = current_time - self.emergency_start_time
            
            if emergency_duration < 3.0:
                # First 3 seconds: stabilize with zero rates and hover thrust
                self.publish_rates_setpoint(0.0, 0.0, 0.0, 0.75)
            else:
                # After stabilization: gradual descent
                descent_thrust = max(0.4, 0.7 - 0.05 * (emergency_duration - 3.0))
                self.publish_rates_setpoint(0.0, 0.0, 0.0, descent_thrust)
            
            if self.current_position[2] < 0.3:  # Landed
                self.disarm_vehicle()
                self.get_logger().info("🛬 Emergency landing completed")


def main(args=None):
    """Main function to run the MPC controller"""
    import sys
    rclpy.init(args=args)
    
    # Get takeoff height from environment or use default
    import os
    takeoff_height = float(os.getenv('TAKEOFF_HEIGHT', '3.0'))
    
    controller = QuadrotorController(takeoff_height=takeoff_height)
    
    try:
        # Set trajectory (can be: 'hover', 'figure8', 'sine', 'circular', 'oval', 'lemniscate') 
        controller.set_trajectory('hover', scale=1.0)
 
        #controller.set_trajectory('figure8', scale=2.0)
        
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("🛑 Shutting down...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()