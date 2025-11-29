#!/usr/bin/env python
############################################################################
# (Copyright Header)
# --- MODIFIED FOR MPC ---
############################################################################

__author__ = "Braden Wagstaff & Gemini"
__contact__ = "braden@arkelectron.com"

import rclpy
from rclpy.node import Node
import tty
import select
import sys
import termios
from std_msgs.msg import Bool
# --- MODIFIED ---
# We now publish a Point (position) instead of Twist (velocity)
from geometry_msgs.msg import Point 
# --- END MODIFIED ---

# store settings
old_attr = termios.tcgetattr(sys.stdin)

#set terminal to raw mode
tty.setcbreak(sys.stdin.fileno())


class Control(Node):

    def __init__(self):
        super().__init__('control')
        #Create publishers
        # --- MODIFIED ---
        # Changed publisher to Point
        self.velocity_publisher_ = self.create_publisher(Point, '/mpc_target_position', 10)
        # --- END MODIFIED ---
        self.arm_publisher_ = self.create_publisher(Bool, '/arm_message', 10)

        #creates callback function for the arm timer
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.arm_timer = self.create_timer(timer_period, self.arm_timer_callback)

        #set variables
        self.arm_message = Bool()
        self.arm_message.data = False
        self.toggle = False

        # --- MODIFIED ---
        # These variables now control the target position
        self.target_x = 0.0  # Target North
        self.target_y = 0.0  # Target East
        self.target_z = -5.0 # Target Down (start 5m high)
        self.target_yaw_change = 0.0 # This is no longer used by the MPC
        
        self.pos_increment = 0.5  # meters
        self.height_increment = 0.25 # meters
        # --- END MODIFIED ---

    def arm_timer_callback(self):
        self.arm_publisher_.publish(self.arm_message)
    
    #publishes Twist messages
    def timer_callback(self):
        
        print('Arm toggle is now:', self.arm_message.data, '\t',
              'Target X:', self.target_x, '\t',
              'Target Y:', self.target_y, '\t',
              'Target Z:', self.target_z)

        # --- MODIFIED ---
        # Publish a Point message with the target coordinates
        msg = Point()
        msg.x = self.target_x
        msg.y = self.target_y
        msg.z = self.target_z
        self.velocity_publisher_.publish(msg)
        # --- END MODIFIED ---


def main(args=None):
    rclpy.init(args=args)

    control = Control()

    print("This node takes keypresses from the keyboard and publishes them \n"
            "as target position messages for the MPC.\n"
            "W: Move Target North (+X)\n"
            "S: Move Target South (-X)\n"
            "A: Move Target West (-Y)\n"
            "D: Move Target East (+Y)\n"
            "Up Arrow: Move Target Up (-Z)\n"
            "Down Arrow: Move Target Down (+Z)\n\n"
            "Press SPACE to arm/disarm the drone")

    while(rclpy.ok()):
        #check if there's any key pressed
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            key = sys.stdin.read(1)
            # --- MODIFIED: Key logic ---
            if key == 'w':
                control.target_x += control.pos_increment
            elif key == 's':
                control.target_x -= control.pos_increment
            elif key == 'a':
                control.target_y -= control.pos_increment # East is +Y, so West is -Y
            elif key == 'd':
                control.target_y += control.pos_increment
            elif key == '\x1b[A':  # Up Arrow
                control.target_z -= control.height_increment # Down is +Z, so Up is -Z
            elif key == '\x1b[B':  # Down Arrow
                control.target_z += control.height_increment
            elif key == ' ': #spacebar
                if(control.toggle == False):
                    control.arm_message.data = True
                    control.toggle = True
                elif(control.toggle == True):
                    control.arm_message.data = False
                    control.toggle = False
            # --- END MODIFIED ---

        rclpy.spin_once(control)

    # restore terminal settings
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)

    control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()