#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
from geometry_msgs.msg import Twist


class SerialMessageClass():

	def __init__(self):


		self.arm_node_check = False
		self.joint_positions = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
		self.joint_velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
		self.arm_mode = 8
		self.string_list = ["0000","0000","0000","0000","0000","0000","0000"]
		self.activity_indicator = 0
		self.probe_servo = 0

		self.twist_var = Twist()
		self.wheel_velocities = ["0000","0000"]

		self.right_wheel = 0
		self.left_wheel = 0

		self.comm_check = "0"


		#self.drive_mode = 0

	def wheelVelToString(self):
		self.left_wheel = 0
		self.right_wheel = 0

		if self.twist_var.linear.x > 0:
			self.left_wheel = self.twist_var.linear.x - self.twist_var.angular.z
			self.right_wheel = self.twist_var.linear.x + self.twist_var.angular.z
		elif self.twist_var.linear.x < 0:
			self.left_wheel = self.twist_var.linear.x + self.twist_var.angular.z
			self.right_wheel = self.twist_var.linear.x - self.twist_var.angular.z

		if(self.left_wheel<0):
			way_left =1
			self.left_wheel *= -1
		else:
			way_left = 0
		if(self.right_wheel<0):
			way_right =1
			self.right_wheel *= -1
		else:
			way_right=0

		print("left: "+str(self.left_wheel)+ " right: "+ str(self.right_wheel))
		if abs(self.left_wheel) < 10:
			self.left_wheelString = "00" + str(int(self.left_wheel))
		elif abs(self.left_wheel) < 100:
			self.left_wheelString = "0" + str(int(self.left_wheel))

		elif abs(self.left_wheel) > 100:
			self.left_wheelString = str(int(self.left_wheel))

		if abs(self.right_wheel) < 10:
			self.right_wheelString = "00" + str(int(self.right_wheel))
		elif abs(self.right_wheel) < 100:
			self.right_wheelString = "0" + str(int(self.right_wheel))

		elif abs(self.right_wheel) > 100:
			self.right_wheelString = str(int(self.right_wheel))
		if(self.left_wheel < 120 and self.left_wheel != 0):
			self.left_wheelString = "120"

		if(self.right_wheel < 120 and self.right_wheel != 0):
			self.right_wheelString = "120"

		if(self.left_wheel > 180):
			self.left_wheelString = "180"

		if(self.right_wheel > 180):
			self.right_wheelString = "180"

		self.wheel_velocities[0] =  str(way_left) + self.left_wheelString
		self.wheel_velocities[1] =  str(way_right) + self.right_wheelString
		

	def jointAngleToString(self):

		print("Joint angle to string")
		for i in range(len(self.joint_positions)):
			radian = self.joint_positions[i]

			degrees = 180*radian/math.pi
		
			if degrees > 0:
				string_angle = "1"
			elif degrees <= 0:
				string_angle = "0"
			degrees = abs(degrees)
			if degrees > 180:
				degrees = 180

			mapped_degree = str(int(degrees*5))

			string_angle = string_angle + "0"*(3-len(mapped_degree)) + mapped_degree
			self.string_list[i] = string_angle


	def jointVelocityToString(self):

		#print(len(self.joint_velocities))
		for i in range(len(self.joint_velocities)):

			velocity = self.joint_velocities[i]
			if velocity < 0:
				direction = "0"
			else:
				direction = "1"
			velocity = abs(velocity*1.25)
			string_vel = str(int(velocity))
			string_vel = direction + "0"*(3-len(string_vel)) + string_vel
			self.string_list[i] = string_vel

	def returnFinalMsg(self):

		final_msg = "S"
			

		final_msg += self.wheel_velocities[0] + self.wheel_velocities[1]

	 	for i in self.string_list:
	 		final_msg += i

	 	final_msg += 16*"0" + str(int(self.arm_mode)) + str(int(self.activity_indicator))+ str(int(self.probe_servo))+"F"
	 	return final_msg 

