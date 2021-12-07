#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import serial
import rosparam
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
import time
import serial_funcs

pub = rospy.Publisher('joint_states/get',Float64MultiArray,queue_size=50)

serial_msg = ""

joints = [0.0,0.0,0.0,0.0,0.0,0.0]

global send
send = False


def feedbackCallback(data):
	global serial_msg
	serial_msg = data.data
	print(serial_msg)

'''
def parseEncoderMessage(str):
	global send
	if check(str):
		send = True
	else:
		send = False
'''

def check(str):
	global send
	if len(str)>2:
		if str[0] == 'A' and str[-1] == 'B':
			print("YES")
			send =  True

		else:
			print("No")
			send = False
	else:
		print("NO")
		send = False


def main():
	global send, joints
	rospy.init_node('encoder_recorder')
	rospy.Subscriber("/serial_feedback", String, feedbackCallback)

	rate = rospy.Rate(20)

	while not rospy.is_shutdown():

		check(serial_msg)
		if send:
			joints = serial_funcs.manipulateEncoder(serial_msg)
			rospy.loginfo(joints)
			message = Float64MultiArray(data=(joints))
			pub.publish(message)
			database_data = serial_funcs.convertAngleToString(joints)
			serial_funcs.saveDataToFile(database_data,"encoder_db.txt")
		rate.sleep()




if __name__ == '__main__':
	print("\n\n\n\n Reading Encoders! \n\n\n}")
	main()