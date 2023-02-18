#!/usr/bin/env python
import sys
import rospy
import serial
import struct
import time
import numpy as np
from rover_msgs.msg import JoyCtrl
import signal

class JoystickCallback:
    
    def __init__(self):
        # Setting up serial connection, should probably have some handeling here
        try:
            self.serial_port = serial.Serial(
                    port="/dev/ttyACM0",
                    baudrate="115200",
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    )
        except:
            print("[ERROR] Not able to connect to port {}, exiting program...".format("/dev/ttyACM0"))
            sys.exit(0)

        # Sleep to give serial time to connect properly
        time.sleep(5)

        # Variable to keep controller in check
        self.cnt = 0
        self.running = True

        # Variables to keep track of Encoder data, each encoder have a maximum of 3000 steps
        self.left_front_wheel = 0
        self.right_front_wheel = 0
        self.left_rear_wheel = 0
        self.right_rear_wheel = 0
        self.total_steps = 3000
        self.wheel_circumference = 100

        # Choosen inertial frame is flat surface with 3DOF, x, y and yaw. When the robot is booted its position will always be (0.0, 0.0, 0.0). May be changed later if a GPS is implemented.
        x = 0.0  
        y = 0.0
        yaw = 0.0
        
        # Joystick control callback
        self.sub_ = rospy.Subscriber("/JoyCtrl", JoyCtrl, self.joystick_callback)

        # Asking for ENCODER data
        self.serial_port.write("ENC_SEND".encode())

        # Timed listener
        rospy.Timer(rospy.Duration(0.01), self.encoder_listner)

    # Handeling when unexpected shutdown occurs or node ends up outside scope
    def __del__(self):
        # Cleaning up serial
        self.serial_port.write("MOTOR_ST".encode())
        self.serial_port.write("ENC_STOP".encode()) # Arduino should stop sending serial data
        self.serial_port.close()

    # Handeling when launch file is closed by Ctrl+C
    def ctrlc(self,signum, frame):
        # Cleaning up serial
        self.serial_port.write("MOTOR_ST".encode())
        self.serial_port.write("ENC_STOP".encode()) # Arduino should stop sending serial data
        self.serial_port.close()

    def encoder_listner(self, event):
        # Waiting for data, it is ended with a readline
        data = self.serial_port.readline()
        
        # Format for data
        # codec, front-left, back-left, front-right, back-right, \n
        # Decoding string
        #wheel_left = data[4:]
        #data_d = data[0:4].decode()
        #print(data_d)
        #print(list(bytearray(wheel_left)))
        #print(data_d)



        first_wheel = data[4:8]
        first_wheel_data = struct.unpack('<f', first_wheel)
        print(first_wheel_data)

    def to_bytes(n, length, endianess='big'):
        h = '%x' % n
        s = ('0'*(len(h) % 2) + h).zfill(length*2).decode('hex')
        return s if endianess == 'big' else s[::-1]

    def joystick_callback(self, msg):
        # Check if code should be run
        if (msg.Run != self.running):
            self.running = msg.Run
            self.cnt = 0

        # If running sends command to wheels
        if msg.Run:
            # Motor 1
            self.serial_port.write("MOTOR_1".encode())
            command_1 = b''
            command_1 += struct.pack('!b', msg.LeftWheels)
            self.serial_port.write(command_1)

            # Motor 2
            self.serial_port.write("MOTOR_2".encode())
            command_2 = b''
            command_2 += struct.pack('!b', msg.RightWheels)
            self.serial_port.write(command_2)

        # Else tells motor to stop
        elif self.cnt < 10:
            # Send stop signal to motors
            self.serial_port.write("MOTOR_ST".encode())
            self.cnt += 1

def main():
    # ROS1 syntax
    rospy.init_node("nano_to_arduino_comm", anonymous=True)
    joy_node = JoystickCallback()
    signal.signal(signal.SIGINT, joy_node.ctrlc)
    rospy.spin()
    del joy_node

if __name__ == "__main__":
    main()
