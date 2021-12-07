#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import serial
import rosparam
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
import time


serial_msg = ""

def serialCallback(data):
	global serial_msg
	serial_msg = data.data
	#rospy.loginfo(serial_msg)



def letsSerial():

	global serial_msg

	rospy.init_node("serial")
	sub = rospy.Subscriber("/rover_serial_topic",String,serialCallback)
	pub = rospy.Publisher("/rover_serial_encoder",String,queue_size=10)

	ser = serial.Serial(port='/dev/ttyUSB0', baudrate=int(115200), parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS,timeout=0.05)


	while not rospy.is_shutdown():

		while ser.isOpen() and not rospy.is_shutdown():

			receive = ser.readline()

			ser.writelines(serial_msg + "\n")

			ser.flushInput()
			ser.flushOutput()

			pub.publish(receive)
			rospy.loginfo(receive)
			rospy.loginfo(serial_msg)


	rospy.spin()



if __name__ == '__main__':
    try:
        letsSerial()
    except rospy.ROSInterruptException:
        pass
