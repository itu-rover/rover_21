#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import serial
import rosparam
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
import time
from JointClass import *

joint_info = JointClass()

pub = rospy.Publisher("/rover_serial_topic", String, queue_size=10)


def jointCallback(data):
	mode_bit = data.data[-3]
	joint_info.mode = mode_bit
	joint_info.activity_indicator = data.data[-2]
	joint_info.probe_servo = data.data[-1]

	if mode_bit == 8:
		joint_info.positions = data.data[0:7]
	elif mode_bit == 5:
		joint_info.velocities = data.data[0:7]

#TODO: wheels callback

def main():
	rospy.init_node('message_creator')
	rospy.Subscriber("joint_states/send", Float64MultiArray, jointCallback)

	#TODO: wheels subscriber

	rate = rospy.Rate(20)

	while not rospy.is_shutdown():

		
		if joint_info.mode == 5:
			joint_info.velocityToString()
			
		elif joint_info.mode == 8:
			joint_info.angleToString()

		final_message = joint_info.returnFinalMsg()
		rospy.loginfo(final_message)
		pub.publish(final_message)
		rate.sleep()


if __name__ == '__main__':
	main()