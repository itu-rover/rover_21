#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import serial
import rosparam
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

test = "A,0500,1500,0250,1250,0000,0000,000000000000000000000000000000000000000000000000 ,B"

def manipulateEncoder(str):
	angle_list = [0,0,0,0,0,0]
	if checkMessage(str):
		parsed = parseEncoderMessage(str)
		print("Parsed: ",parsed)
		for i in range(5):
			data = parsed[i]
			print(data)
			sign = detectSign(data)
			radian = convertToRadians(data,sign)
			angle_list[i] = radian
		return angle_list
	else:
		pass

def checkMessage(str):
	if (str[0]=="A" and str[-1] == "B"):
		return True
	else:
		return False

def parseEncoderMessage(str):
	parsed_data = str.split(",")
	parsed_data = parsed_data[5:10]
	return parsed_data

def detectSign(data):
	if data[0] == '0':
		return -1
	else:
		return 1

def convertToRadians(data,sign):
	data = int(data[1:])/5
	data = data*math.pi/180
	return data*sign

def convertAngleToString(angle_list):
	angles_as_string = ""
	for i in angle_list:
		angles_as_string += (str(i)+",")
	angle_as_string = angles_as_string[0:-1]
	return angles_as_string

def saveDataToFile(string_angle,file_path):
	file_object = open(file_path,"w+")
	file_object.write(string_angle)
	file_object.close() 
	print("Yazdim")


