#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""
Dummy sensor message creator. It will only be used for test purposes
"""

import rospy
import math
import serial
import rosparam
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

pub = rospy.Publisher("/auto_arm_topic", Float64MultiArray, queue_size=10)

def main():
	rospy.init_node('dummy_sensor')
	rate = rospy.Rate(20)

	while not rospy.is_shutdown():
		send_message = raw_input("Send message (y/n): ")
		if (send_message == "y"):
			x = raw_input("Enter x (double): ")
			x = float(x)
			y = raw_input("Enter y (double): ") 
			y = float(y)
			z = raw_input("Enter z (double): ")
			z = float(z)
			element = 1 			# Arbitrarily given
			goal_info = [x,y,z,element]
			message = Float64MultiArray(data=(goal_info))
			pub.publish(message)

		elif (send_message == "n"):
			print("\nK bro no offense\n")
		else:
			print("\nInvalid input\n")


if __name__ == '__main__':
	main()