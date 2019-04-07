#!/usr/bin/env python
import time
import serial
import sys, termios, tty, os, time
import fcntl
from geometry_msgs.msg import Twist #as cmdvel
import rospy

# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

def cmdvel_callback(data):
	vel_linear_x=data.linear.x
	print(vel_linear_x)
	vel_angular_z=data.angular.z
	print(vel_angular_z)
	vel_linear_x_send= 100*vel_linear_x+100
	vel_angular_z_send= 30*vel_angular_z+90
        str1=str(chr(int(vel_linear_x_send)))
	print str1
	print vel_linear_x_send
	str1=str(chr(int(vel_angular_z_send)))
	print str1
	print vel_angular_z_send
	#ser.write(str(chr(int(vel_linear_x_send))))
	#ser.write(str(chr(int(vel_angular_z_send))))
	ser.write(chr(int(vel_linear_x_send)))
	ser.write(chr(int(vel_angular_z_send)))

ser.isOpen()
rospy.init_node('a_star', anonymous=True)
#while 1: 
while not rospy.is_shutdown():
        time.sleep(0.5)
	rospy.Subscriber("/cmd_vel", Twist, cmdvel_callback) 
	
	
''' def cmdvel_callback(data):
	vel_linear_x=data.linear.x
	vel_angular_z=data.angular.z
	vel_linear_x_send=100*vel_linear_x+100
	vel_angular_z_send=30*vel_angular_z+90
        str1=str(chr(int(vel_linear_x_send)))
	print str1
	ser.write(str(chr(int(vel_linear_x_send)))) '''

	
		
		
		
		
		
		
		

