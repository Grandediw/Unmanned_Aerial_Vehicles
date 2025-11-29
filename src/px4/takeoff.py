#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math 



class ArmAndTakeoff(Node):


    def __init__(self):
        super().__init__('arm_and_takeoff')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # match PX4 publisher
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

     
        # Publishers
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)


        self.waypoints = [
            {"x": 0.0, "y": 0.0, "z": -5.0},   # decollo a 5m
            {"x": 5.0, "y": 0.0, "z": -5.0},   # avanti 5m
            {"x": 5.0, "y": 5.0, "z": -5.0},   # diagonale
            {"x": 0.0, "y": 0.0, "z": -5.0}    # ritorno
        ]
        self.current_wp = 0
        self.current_position_wp = {"x": 0.0, "y": 0.0, "z": -5.0}

        # Timers
        self.create_timer(0.1, self.publish_offboard_mode)   # 10 Hz
        self.create_timer(0.1,  lambda: self.publish_setpoint(coordinate=self.waypoints[self.current_wp]))        # 10 Hz
        self.arm_timer = self.create_timer(2.0, self.arm)                     # arm after 2 s
        self.offboard_tiemr = self.create_timer(3.0, self.set_offboard_mode)       # switch to offboard after 3 s


        self.current_position_recived = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',self.callback_pos,qos_profile)

    def callback_pos(self,msg:VehicleLocalPosition):
        
        self.current_position_wp = self.waypoints[self.current_wp]
        distance = math.sqrt((msg.x-self.current_position_wp["x"])**2+(msg.y-self.current_position_wp["y"])**2+(msg.z-self.current_position_wp["z"])**2)
        print(f"la distanza {distance}")
        if distance < 0.5:
            self.current_wp +=1
            self.current_position_wp = self.waypoints[self.current_wp]
    

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True   # we want to control position
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_pub.publish(msg)

    def publish_setpoint(self, coordinate:dict= None):
        sp = TrajectorySetpoint()
     
        sp.position[0] = coordinate["x"]
        sp.position[1] = coordinate["y"]
        sp.position[2] = coordinate["z"]
 # z (NED, so -10 = climb up 10 m)
        sp.yaw = 0.0
        self.setpoint_pub.publish(sp)

    def arm(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)
        self.get_logger().info("Arm command sent")
        self.arm_timer.cancel()  # STOP SEND THE LOG AFTER IT ARMS IT

    def set_offboard_mode(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0  # base mode
        msg.param2 = 6.0  # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)
        self.get_logger().info("Set OFFBOARD mode command sent")
        self.offboard_tiemr.cancel()  #STOP SENDING THE LOG AFTER IT TAKEOFF
"""
    def takeoff(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
        msg.param7 = 10.0  # target altitude in meters
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.pub.publish(msg)
        self.get_logger().info("Takeoff command sent")

        IT USES MISSION LOGIC, without this i ma flying to mission point

        """

def main(args=None):
    rclpy.init(args=args)
    node = ArmAndTakeoff()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
