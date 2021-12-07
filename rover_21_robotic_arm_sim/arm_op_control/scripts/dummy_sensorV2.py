#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""
Dummy sensor message creator. It will only be used for test purposes
"""

import rospy
import math
import serial
import rosparam
import random
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

pub = rospy.Publisher("/auto_arm_topic", Float64MultiArray, queue_size=10)

def main():
	rospy.init_node('dummy_sensor')
	rate = rospy.Rate(20)

	while not rospy.is_shutdown():
		x = random.uniform(0.97,0.98)
		y = random.uniform(0,0.02)
		z = random.uniform(0.54,0.55)
		element = 1 			# Arbitrarily given
		goal_info = [x,y,z,element]
		message = Float64MultiArray(data=(goal_info))
		rospy.loginfo(goal_info)
		pub.publish(message)
		rate.sleep()
if __name__ == '__main__':
	main()