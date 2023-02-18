#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from rover_msgs.msg import JoyCtrl
import numpy as np
import signal

class JoystickController:
    def __init__(self):
        # Button variables
        self.cross_button_pressed = False
        self.triangle_button_pressed = False
        self.run = True
        self.reset = False

        # Setting up a publisher
        self.pub_ = rospy.Publisher("JoyCtrl", JoyCtrl, queue_size=5)

        # Setting up joystick subscriber
        self.joy_sub_ =  rospy.Subscriber("joy", Joy, self.joystick_callback)

    def joystick_callback(self, msg):
        # Reading buttonss

        # Checking cross buttons
        if (msg.buttons[1] and not self.cross_button_pressed):
            self.cross_buttons_pressed = True
            self.run != self.run

        # Checking triangle buttons
        if (msg.buttons[3] and not self.triangle_button_pressed):
            self.triangle_buttons_pressed = True
            self.reset = True
        
        # Resets buttons
        if (not msg.buttons[1]):
            self.cross_button_pressed = False
        if (not msg.buttons[3]):
            self.triangle_button_pressed = False

        # Reading joystick, Y is up-down, left if left-right
        x_right = msg.axes[2]
        y_right = msg.axes[5] 

        # Setting zero speed values
        step_left = 0
        step_right = 0

        # Checking deadzones
        if abs(y_right) > 0.1:
            step_left = y_right*63
            step_right = y_right*63

        # Turning
        if abs(x_right) > 0.1:
            step_left = min(max(step_left - x_right*32, -63), 63)
            step_right = max(min(step_right + x_right*32, 63), -63)

        # Publishing control message
        msg = JoyCtrl()
        msg.LeftWheels = np.int8(step_left)
        msg.RightWheels = np.int8(step_right)
        msg.Run = self.run
        self.pub_.publish(msg)

def main():
    # ROS1 Syntax
    rospy.init_node("joystick_controller")
    joy_node = JoystickController()
    rospy.spin()
    del joy_node

if __name__ == "__main__":
    main()
