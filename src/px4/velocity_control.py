#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Braden Wagstaff & Gemini (MPC)"
__contact__ = "braden@arkelectron.com"

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Bool

# --- ADDED ---
# Import MPC solver and new message types
import casadi as ca
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleAttitudeSetpoint
# --- END ADDED ---

# --- MODIFIED ---
# Removed unused messages: TrajectorySetpoint, Twist, VehicleAttitude
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand
from geometry_msgs.msg import Vector3, Point
from math import pi
# --- END MODIFIED ---


class OffboardControl(Node):

    def __init__(self):
        super().__init__('offboard_control_mpc') # Renamed node
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- Subscriptions ---
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        
        # --- REMOVED KEYBOARD DEPENDENCY ---
        # self.my_bool_sub = self.create_subscription(
        #     Bool,
        #     '/arm_message',
        #     self.arm_message_callback,
        #     qos_profile)

        # --- ADDED ---
        # Subscription for MPC state input
        self.odometry_sub_ = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos_profile)
        
        # --- AUTONOMOUS MODE - NO KEYBOARD NEEDED ---
        # Subscription for target position from control node (optional)
        # from geometry_msgs.msg import Point
        # self.target_position_sub = self.create_subscription(
        #     Point,
        #     '/mpc_target_position',
        #     self.target_position_callback,
        #     qos_profile)
        # --- END ADDED ---
        
        # --- REMOVED ---
        # These subscriptions are no longer needed by the MPC
        # self.offboard_velocity_sub = self.create_subscription(...)
        # self.attitude_sub = self.create_subscription(...)
        # --- END REMOVED ---


        # --- Publishers ---
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        # --- ADDED ---
        # Publisher for the MPC's output
        self.attitude_setpoint_publisher_ = self.create_publisher(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile)
        # --- END ADDED ---

        # --- REMOVED ---
        # These publishers are no longer needed
        # self.publisher_velocity = self.create_publisher(...)
        # self.publisher_trajectory = self.create_publisher(...)
        # --- END REMOVED ---
        

        # --- MPC Parameters ---
        self.T = 0.1  # MPC sampling time (seconds)
        self.N = 10   # MPC prediction horizon (reduced for speed)
        self.g = 9.81 # Gravity
        self.mass = 1.5 # Mass of the drone (kg)

        # --- State Variables ---
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED
        self.flightCheck = False
        self.myCnt = 0
        self.arm_message = True  # AUTONOMOUS: Always want to arm
        self.failsafe = False
        self.current_state = "IDLE"
        self.last_state = self.current_state
        
        # --- ADDED ---
        self.mpc_solver = self.setup_mpc() # Initialize the MPC solver
        self.current_odometry = None # Will store [x, y, z, vx, vy, vz, roll, pitch, yaw]
        
        # --- AUTONOMOUS TRAJECTORY WAYPOINTS ---
        self.waypoints = [
            np.array([0.0, 0.0, -5.0]),   # Hover at 5m
            np.array([5.0, 0.0, -5.0]),   # Move 5m North
            np.array([5.0, 5.0, -5.0]),   # Move 5m East  
            np.array([0.0, 5.0, -5.0]),   # Move back West
            np.array([0.0, 0.0, -5.0]),   # Return to origin
            np.array([0.0, 0.0, -3.0]),   # Climb to 3m
            np.array([0.0, 0.0, -7.0]),   # Descend to 7m
            np.array([0.0, 0.0, -5.0]),   # Back to 5m hover
        ]
        self.current_waypoint_index = 0
        self.waypoint_threshold = 1.0  # meters - how close to get to waypoint
        self.waypoint_hold_time = 3.0  # seconds - how long to hold at each waypoint
        self.waypoint_timer = 0.0
        self.target_position = self.waypoints[0]  # Start with first waypoint
        
        self.get_logger().info("MPC solver setup complete.")
        self.get_logger().info(f"AUTONOMOUS MODE: Will follow {len(self.waypoints)} waypoints")
        # --- END ADDED ---

        # --- REMOVED ---
        # self.velocity = Vector3()
        # self.yaw = 0.0
        # self.trueYaw = 0.0
        # self.offboardMode = False
        # --- END REMOVED ---

        # creates callback function for the arm timer
        arm_timer_period = .1 # seconds
        self.arm_timer_ = self.create_timer(arm_timer_period, self.arm_timer_callback)

        # --- MODIFIED ---
        # MPC loop timer
        self.timer = self.create_timer(self.T, self.cmdloop_callback)
        # --- END MODIFIED ---

    # --- REMOVED KEYBOARD DEPENDENCY ---
    # def arm_message_callback(self, msg):
    #     self.arm_message = msg.data
    #     self.get_logger().info(f"Arm Message: {self.arm_message}")

    # --- AUTONOMOUS WAYPOINT MANAGEMENT ---
    def update_autonomous_target(self):
        """Update target position for autonomous waypoint following."""
        if self.current_odometry is None:
            return
            
        # Current position
        current_pos = self.current_odometry[0:3]  # [x, y, z]
        
        # Distance to current target
        distance_to_target = np.linalg.norm(current_pos - self.target_position)
        
        # Check if we're close enough to the current waypoint
        if distance_to_target < self.waypoint_threshold:
            self.waypoint_timer += self.T  # Increment timer
            
            # Hold at waypoint for specified time
            if self.waypoint_timer >= self.waypoint_hold_time:
                # Move to next waypoint
                self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
                self.target_position = self.waypoints[self.current_waypoint_index]
                self.waypoint_timer = 0.0
                
                self.get_logger().info(f"Moving to waypoint {self.current_waypoint_index}: "
                                     f"[{self.target_position[0]:.1f}, {self.target_position[1]:.1f}, {self.target_position[2]:.1f}]")
        else:
            self.waypoint_timer = 0.0  # Reset timer if not at waypoint

    # This state machine for arming/takeoff is good and remains unchanged
    def arm_timer_callback(self):
        # --- AUTONOMOUS: Always try to arm if flight checks pass ---
        match self.current_state:
            case "IDLE":
                if(self.flightCheck):  # Removed arm_message dependency
                    self.current_state = "ARMING"
                    self.get_logger().info(f"AUTONOMOUS: Arming initiated")

            case "ARMING":
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Arming, Flight Check Failed")
                elif(self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.myCnt > 10):
                    self.current_state = "TAKEOFF"
                    self.get_logger().info(f"AUTONOMOUS: Proceeding to Takeoff")
                self.arm() 

            case "TAKEOFF":
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Takeoff, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF):
                    self.current_state = "LOITER"
                    self.get_logger().info(f"AUTONOMOUS: Takeoff complete, entering Loiter")
                self.arm() 
                self.take_off() 

            case "LOITER": 
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Loiter, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):
                    self.current_state = "OFFBOARD"
                    self.get_logger().info(f"AUTONOMOUS: Entering MPC Offboard Control")
                self.arm()

            case "OFFBOARD":
                if(not(self.flightCheck) or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe == True):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Offboard, Flight Check Failed")
                # This function is new, it just sets the mode
                self.state_offboard()
                # --- AUTONOMOUS WAYPOINT FOLLOWING ---
                self.update_autonomous_target()

        # --- AUTONOMOUS: No arm message dependency ---
        # if(self.arm_state != VehicleStatus.ARMING_STATE_ARMED):
        #     self.arm_message = False

        if (self.last_state != self.current_state):
            self.last_state = self.current_state
            self.get_logger().info(self.current_state)

        self.myCnt += 1

    def state_offboard(self):
        self.myCnt = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        # self.offboardMode = True # This variable is no longer needed

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")

    def take_off(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7=5.0) # param7 is altitude in meters
        self.get_logger().info("Takeoff command send")

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)
 
    def vehicle_status_callback(self, msg):
        if (msg.nav_state != self.nav_state):
            self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")
        
        if (msg.arming_state != self.arm_state):
            self.get_logger().info(f"ARM STATUS: {msg.arming_state}")

        if (msg.failsafe != self.failsafe):
            self.get_logger().info(f"FAILSAFE: {msg.failsafe}")
        
        if (msg.pre_flight_checks_pass != self.flightCheck):
            self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")

        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass

    # --- ADDED ---
    def target_position_callback(self, msg):
        """Callback for target position from control node."""
        # AUTONOMOUS MODE: This callback is not used
        pass
    # --- END ADDED ---

    # --- ADDED ---
    def odometry_callback(self, msg):
        """Callback for vehicle odometry (our MPC state)."""
        # Convert quaternion to euler - PX4 quaternion format: [w, x, y, z]
        try:
            q = msg.q
            if len(q) >= 4:  # Ensure we have all quaternion components
                # PX4 quaternion: q[0]=w, q[1]=x, q[2]=y, q[3]=z
                qw, qx, qy, qz = q[0], q[1], q[2], q[3]
                
                # Convert to Euler angles
                roll = np.arctan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
                pitch = np.arcsin(2.0 * (qw * qy - qz * qx))
                yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

                # Update the current state vector
                self.current_odometry = np.array([
                    msg.position[0], msg.position[1], msg.position[2],
                    msg.velocity[0], msg.velocity[1], msg.velocity[2],
                    roll, pitch, yaw
                ])
            else:
                self.get_logger().warn("Invalid quaternion data received")
        except Exception as e:
            self.get_logger().error(f"Odometry callback error: {e}")

    def setup_mpc(self):
        """
        Defines and sets up the Optimal Control Problem (OCP) using CasADi.
        """
        self.get_logger().info("Setting up MPC problem...")
        
        # --- 1. Define Symbolic Variables ---
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        z = ca.SX.sym('z')
        vx = ca.SX.sym('vx')
        vy = ca.SX.sym('vy')
        vz = ca.SX.sym('vz')
        phi = ca.SX.sym('phi')
        theta = ca.SX.sym('theta')
        psi = ca.SX.sym('psi')
        states = ca.vertcat(x, y, z, vx, vy, vz, phi, theta, psi)
        n_states = states.numel() # 9

        T = ca.SX.sym('T')
        tau_x = ca.SX.sym('tau_x')
        tau_y = ca.SX.sym('tau_y')
        tau_z = ca.SX.sym('tau_z')
        controls = ca.vertcat(T, tau_x, tau_y, tau_z)
        n_controls = controls.numel() # 4

        # --- 2. Define System Dynamics ---
        # (This is a simplified model, a real one would be more complex)
        rhs = ca.vertcat(
            vx,                                 
            vy,                                 
            vz,                                 
            T/self.mass * (ca.cos(phi)*ca.sin(theta)*ca.cos(psi) + ca.sin(phi)*ca.sin(psi)), 
            T/self.mass * (ca.cos(phi)*ca.sin(theta)*ca.sin(psi) - ca.sin(phi)*ca.cos(psi)), 
            T/self.mass * (ca.cos(phi)*ca.cos(theta)) - self.g, 
            tau_x, # Simplification: p = tau_x
            tau_y, # Simplification: q = tau_y
            tau_z  # Simplification: r = tau_z
        )

        f = ca.Function('f', [states, controls], [rhs])
        X = ca.SX.sym('X', n_states)
        U = ca.SX.sym('U', n_controls)
        X_next = X + self.T * f(X, U)
        F = ca.Function('F', [X, U], [X_next], ['x0', 'p0'], ['xf'])

        # --- 3. Define Cost Function ---
        Q = ca.diag([10.0, 10.0, 20.0, 1.0, 1.0, 1.0, 0.5, 0.5, 0.1])
        R = ca.diag([0.1, 0.05, 0.05, 0.01])

        # --- 4. Formulate the OCP ---
        opti = ca.Opti()
        
        x = opti.variable(n_states, self.N + 1)
        u = opti.variable(n_controls, self.N)
        
        x_current = opti.parameter(n_states)
        x_target = opti.parameter(n_states)
        
        cost = 0
        for k in range(self.N):
            state_error = x[:, k] - x_target
            control_effort = u[:, k]
            cost += ca.mtimes([state_error.T, Q, state_error])
            cost += ca.mtimes([control_effort.T, R, control_effort])
            
        terminal_error = x[:, self.N] - x_target
        cost += ca.mtimes([terminal_error.T, Q, terminal_error])
        
        opti.minimize(cost)

        # --- 5. Add Constraints ---
        for k in range(self.N):
            opti.subject_to(x[:, k+1] == F(x[:, k], u[:, k]))

        thrust_min = 0.0 
        thrust_max = self.mass * self.g * 2.0 # Max 2G hover thrust
        torque_limit = 1.0 

        for k in range(self.N):
            opti.subject_to(opti.bounded(thrust_min, u[0, k], thrust_max))
            opti.subject_to(opti.bounded(-torque_limit, u[1, k], torque_limit))
            opti.subject_to(opti.bounded(-torque_limit, u[2, k], torque_limit))
            opti.subject_to(opti.bounded(-torque_limit, u[3, k], torque_limit))
            
        opti.subject_to(x[:, 0] == x_current)

        # --- 6. Create the Solver ---
        opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.sb': 'yes'}
        opti.solver('ipopt', opts)
        
        return opti, x, u, x_current, x_target

    def publish_attitude_setpoint(self, thrust, roll, pitch, yaw):
        """Publish the attitude setpoint from MPC."""
        msg = VehicleAttitudeSetpoint()

        # Convert Euler to Quaternion (PX4 expects W, X, Y, Z)
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        q = [0.0] * 4
        q[0] = cr * cp * cy + sr * sp * sy  # W
        q[1] = sr * cp * cy - cr * sp * sy  # X
        q[2] = cr * sp * cy + sr * cp * sy  # Y
        q[3] = cr * cp * sy - sr * sp * cy  # Z
        
        msg.q_d = q

        # Set thrust (in NED, so Z is negative for 'up')
        # We normalize thrust to a value between 0.0 and 1.0
        normalized_thrust = np.clip(thrust / (self.mass * self.g * 2.0), 0.0, 1.0)
        
        msg.thrust_body[0] = 0.0
        msg.thrust_body[1] = 0.0
        msg.thrust_body[2] = -normalized_thrust # Invert for NED frame (positive thrust is down)

        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.attitude_setpoint_publisher_.publish(msg)
    # --- END ADDED ---


    # --- MODIFIED ---
    # This is now the main MPC loop
    def cmdloop_callback(self):
        
        # We only run the MPC if we are in OFFBOARD state
        if(self.current_state == "OFFBOARD"):
            
            # Check if we have odometry data
            if self.current_odometry is None:
                self.get_logger().warn("MPC waiting for odometry...")
                return

            # --- 1. Publish offboard control heartbeat ---
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            offboard_msg.position = False
            offboard_msg.velocity = False # We are not sending velocity
            offboard_msg.acceleration = False
            offboard_msg.attitude = True # We ARE sending attitude
            offboard_msg.body_rate = False
            self.publisher_offboard_mode.publish(offboard_msg)            

            # --- 2. Run the MPC Solver ---
            opti, x, u, x_current, x_target = self.mpc_solver
            
            # Target is to hover 5m high (x=0, y=0, z=-5.0)
            target_state = np.array([self.target_position[0], self.target_position[1], self.target_position[2],
                                     0, 0, 0, 0, 0, 0])
            
            opti.set_value(x_current, self.current_odometry)
            opti.set_value(x_target, target_state)

            try:
                # Solve the optimization problem
                sol = opti.solve()

                # Get the first optimal control input
                u_optimal = sol.value(u[:, 0])
                thrust_command = u_optimal[0]
                
                # This is a simplification!
                # The MPC model outputs torques, but we are sending attitude.
                # A more complex model would output attitude, or we would
                # map these torques to attitude commands.
                # For this demo, we'll use the torques as roll/pitch commands.
                roll_command = u_optimal[1] 
                pitch_command = u_optimal[2]
                yaw_command = u_optimal[3] # This is a rate, not an angle

                # --- 3. Publish the MPC Command ---
                self.publish_attitude_setpoint(
                    thrust=thrust_command,
                    roll=roll_command,
                    pitch=pitch_command,
                    yaw=self.current_odometry[8] # Hold current yaw
                )

            except Exception as e:
                self.get_logger().error(f"MPC solver failed: {e}")
                # Failsafe: command a gentle hover
                self.publish_attitude_setpoint(thrust=self.mass * self.g, roll=0.0, pitch=0.0, yaw=self.current_odometry[8])
                
        # --- REMOVED ---
        # All the old TrajectorySetpoint logic is gone
        # --- END REMOVED ---
    # --- END MODIFIED ---


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()