#!/usr/bin/env python
# -*- coding: utf-8 -*-

import signal
import os
import rospy
import math
import serial
import rosparam
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
import time


serial_msg = ""
comm_check = "0"
callback_check = False
ser = serial.Serial(port='/dev/ttyUSB0', baudrate=int(115200), parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS,timeout=0.08)


def serialCallback(data):
	global serial_msg, callback_check
	serial_msg = data.data
	callback_check = True
	

def receiveSignal(sigNum,frame):

	global serial_msg, comm_check, ser
	comm_check = "2"
	
	#while ser.isOpen() and not rospy.is_shutdown():
	for i in range(5):
		new_msg = serial_msg[0:-1] + comm_check + serial_msg[-1]
		ser.writelines(new_msg + "\n")
		ser.flushInput()
		ser.flushOutput()
		rospy.loginfo(new_msg)

	time.sleep(0.5)
	print("\n\nGOODBYE :)\n\n")
	

	raise SystemExit('Exiting')


def letsSerial():

	global serial_msg, comm_check, callback_check, ser

	comm_check_cnt = 0

	signal.signal(signal.SIGINT, receiveSignal)

	rospy.init_node("serial")
	sub = rospy.Subscriber("/rover_serial_topic",String,serialCallback)
	pub = rospy.Publisher("/serial_feedback",String,queue_size=10)

	
	while not rospy.is_shutdown():

		while ser.isOpen() and not rospy.is_shutdown():

			comm_check_cnt += 1

			if callback_check:
				if (comm_check_cnt >= 5):
					if comm_check == "0":
						comm_check = "1"
					else:
						comm_check = "0"
					comm_check_cnt = 0

				new_msg = serial_msg[0:-1] + comm_check + serial_msg[-1]

				receive = ser.readline()

				ser.writelines(new_msg + "\n")


				ser.flushInput()
				ser.flushOutput()

				pub.publish(receive)
				rospy.loginfo(receive)
				rospy.loginfo(new_msg)
				


	rospy.spin()



if __name__ == '__main__':
    try:
        letsSerial()
    except rospy.ROSInterruptException:
        pass
