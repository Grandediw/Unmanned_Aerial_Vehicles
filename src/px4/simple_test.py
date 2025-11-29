#!/usr/bin/env python3
############################################################################
# Simple PX4 Communication Test
# Test receiving messages from PX4 and sending basic commands
# Use with: px4 sitl gz_x500 (which automatically opens Gazebo)
############################################################################

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class SimplePX4Test(Node):

    def __init__(self):
        super().__init__('simple_px4_test')
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscriptions - test receiving data from PX4
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        
        self.odometry_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos_profile)

        # Create publishers - test sending commands to PX4
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        # Simple state variables
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_DISARMED
        self.current_position = [0.0, 0.0, 0.0]
        self.current_velocity = [0.0, 0.0, 0.0]
        self.flightCheck = False
        self.message_count = 0
        self.test_state = "WAITING"

        # Test timer - run every 100ms
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.test_loop)

        self.get_logger().info("🧪 SIMPLE PX4 COMMUNICATION TEST STARTED")
        self.get_logger().info("📡 Testing message reception and command sending...")
        self.get_logger().info("🌎 Make sure to run: px4 sitl gz_x500")
        self.get_logger().info("👀 This will open Gazebo with the x500 drone automatically!")

    def setup_gazebo(self):
        """Launch Gazebo with x500 drone automatically"""
        try:
            self.get_logger().info("🚀 Starting Gazebo with x500 drone...")
            
            # Check if Gazebo is already running
            try:
                subprocess.run(['gz', 'service', '-s', '/gazebo/resource_paths'], 
                             check=True, capture_output=True, timeout=2)
                self.get_logger().info("🌎 Gazebo already running, spawning x500...")
                self.spawn_x500()
                return
            except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
                pass
            
            # Start Gazebo in the background
            self.gazebo_process = subprocess.Popen([
                'gz', 'sim', '-v', '4', '-r', 
                '/opt/ros/humble/share/px4_ros_com/worlds/px4_ros2.world'
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
            # Wait for Gazebo to start (check if gz service is available)
            self.get_logger().info("⏳ Waiting for Gazebo to initialize...")
            timeout = 15  # 15 second timeout
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                try:
                    subprocess.run(['gz', 'service', '-s', '/gazebo/resource_paths'], 
                                 check=True, capture_output=True, timeout=2)
                    self.get_logger().info("✅ Gazebo is ready!")
                    break
                except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
                    time.sleep(0.5)
            else:
                self.get_logger().warn("⚠️ Gazebo startup timeout, continuing anyway...")
            
            # Give Gazebo a moment to fully initialize
            time.sleep(2)
            
            # Spawn the x500 drone
            self.spawn_x500()
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to start Gazebo: {e}")
            self.get_logger().info("📝 Continuing without Gazebo visualization...")

    def spawn_x500(self):
        """Spawn the x500 drone in Gazebo"""
        try:
            self.get_logger().info("🚁 Spawning x500 drone in Gazebo...")
            
            # Spawn x500 drone
            self.spawn_process = subprocess.Popen([
                'gz', 'service', '-s', '/world/px4_ros2/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '5000',
                '--req', 'sdf_filename: "/opt/ros/humble/share/px4_ros_com/models/x500/model.sdf", name: "x500", pose: {position: {x: 0, y: 0, z: 0.2}}'
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            
            # Wait for spawn to complete
            stdout, stderr = self.spawn_process.communicate(timeout=10)
            
            if self.spawn_process.returncode == 0:
                self.get_logger().info("✅ x500 drone spawned successfully!")
                self.get_logger().info("👀 You can now see the drone in Gazebo!")
            else:
                self.get_logger().warn(f"⚠️ Drone spawn may have failed: {stderr.decode()}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Failed to spawn x500: {e}")

    def cleanup_gazebo(self):
        """Clean up Gazebo processes"""
        try:
            if self.spawn_process and self.spawn_process.poll() is None:
                self.spawn_process.terminate()
                
            if self.gazebo_process and self.gazebo_process.poll() is None:
                self.get_logger().info("🧹 Cleaning up Gazebo...")
                self.gazebo_process.terminate()
                # Give it a moment to terminate gracefully
                try:
                    self.gazebo_process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.gazebo_process.kill()
                    
        except Exception as e:
            self.get_logger().error(f"Error cleaning up Gazebo: {e}")

    def vehicle_status_callback(self, msg):
        """Test receiving vehicle status from PX4"""
        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.flightCheck = msg.pre_flight_checks_pass
        
        # Log every 10th message to avoid spam
        self.message_count += 1
        if self.message_count % 50 == 0:  # Every 5 seconds
            self.get_logger().info(f"📊 Status - Nav State: {msg.nav_state}, "
                                 f"Arm State: {msg.arming_state}, "
                                 f"Flight Check: {msg.pre_flight_checks_pass}")

    def odometry_callback(self, msg):
        """Test receiving odometry from PX4"""
        self.current_position = [msg.position[0], msg.position[1], msg.position[2]]
        self.current_velocity = [msg.velocity[0], msg.velocity[1], msg.velocity[2]]
        
        # Log position every 50 messages to avoid spam
        if self.message_count % 50 == 0:
            self.get_logger().info(f"📍 Position: [{msg.position[0]:.2f}, {msg.position[1]:.2f}, {msg.position[2]:.2f}]")

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        """Test sending commands to PX4"""
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

    def publish_offboard_heartbeat(self):
        """Test sending offboard control mode heartbeat"""
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = False
        offboard_msg.velocity = True
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        self.publisher_offboard_mode.publish(offboard_msg)

    def publish_hover_setpoint(self):
        """Test sending trajectory setpoint for hover"""
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        
        # Simple hover command - zero velocity
        trajectory_msg.velocity[0] = 0.0
        trajectory_msg.velocity[1] = 0.0
        trajectory_msg.velocity[2] = 0.0
        
        # Set position and acceleration to NaN (velocity control)
        trajectory_msg.position[0] = float('nan')
        trajectory_msg.position[1] = float('nan')
        trajectory_msg.position[2] = float('nan')
        trajectory_msg.acceleration[0] = float('nan')
        trajectory_msg.acceleration[1] = float('nan')
        trajectory_msg.acceleration[2] = float('nan')
        trajectory_msg.yaw = float('nan')
        trajectory_msg.yawspeed = 0.0

        self.publisher_trajectory.publish(trajectory_msg)

    def test_loop(self):
        """Main test loop - simple state machine"""
        
        if self.test_state == "WAITING":
            # Wait for flight checks to pass
            if self.flightCheck:
                self.test_state = "READY"
                self.get_logger().info("✅ Flight checks passed - ready for commands")
                
        elif self.test_state == "READY":
            # Test basic communication
            self.get_logger().info("🔧 Testing ARM command...")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.test_state = "ARMING"
            
        elif self.test_state == "ARMING":
            if self.arm_state == VehicleStatus.ARMING_STATE_ARMED:
                self.get_logger().info("✅ Successfully ARMED! Testing takeoff command...")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=1.0, param7=5.0)
                self.test_state = "TAKEOFF"
                
        elif self.test_state == "TAKEOFF":
            if self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
                self.get_logger().info("🚀 Takeoff initiated! Waiting for loiter...")
                self.test_state = "LOITER_WAIT"
                
        elif self.test_state == "LOITER_WAIT":
            if self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                self.get_logger().info("🎯 In loiter mode! Testing offboard control...")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.test_state = "OFFBOARD_TEST"
                
        elif self.test_state == "OFFBOARD_TEST":
            # Test sending offboard commands
            self.publish_offboard_heartbeat()
            self.publish_hover_setpoint()
            
            # Just hover and report status
            if self.message_count % 50 == 0:  # Every 5 seconds
                self.get_logger().info(f"🎮 OFFBOARD CONTROL ACTIVE - Position: [{self.current_position[0]:.2f}, {self.current_position[1]:.2f}, {self.current_position[2]:.2f}]")

    def arm(self):
        """Simplified arm function"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def take_off(self):
        """Simplified takeoff function"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=1.0, param7=5.0)


def main(args=None):
    rclpy.init(args=args)

    simple_test = SimplePX4Test()

    try:
        rclpy.spin(simple_test)
    except KeyboardInterrupt:
        print("\n🛑 Simple PX4 test stopped by user")
        print("📊 Communication test results:")
        print(f"   - Messages received: {simple_test.message_count}")
        print(f"   - Final state: {simple_test.test_state}")
        print(f"   - Flight checks: {simple_test.flightCheck}")
        print(f"   - Arm state: {simple_test.arm_state}")

    simple_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()