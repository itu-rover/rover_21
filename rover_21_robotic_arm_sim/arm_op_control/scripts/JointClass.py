#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

#TODO: change joint class to serial message creator class
#TODO: add class variables and methods (for wheels) into the class

class JointClass():

	def __init__(self):

		self.positions = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
		self.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
		self.mode = 8
		self.string_list = ["0000","0000","0000","0000","0000","0000","0000"]
		self.activity_indicator = 0
		self.probe_servo = 0

		#self.wheels = ["0000","0000"]

	def angleToString(self):

		for i in range(len(self.positions)):
			radian = self.positions[i]

			degrees = 180*radian/math.pi
		
			if degrees >= 0:
				string_angle = "1"
			elif degrees < 0:
				string_angle = "0"
			degrees = abs(degrees)
			if degrees > 180:
				degrees = 180

			mapped_degree = str(int(degrees*5))

			string_angle = string_angle + "0"*(3-len(mapped_degree)) + mapped_degree
			self.string_list[i] = string_angle
		


	def velocityToString(self):

		print(len(self.velocities))
		for i in range(len(self.velocities)):

			velocity = self.velocities[i]
			if velocity < 0:
				direction = "0"
			else:
				direction = "1"
			velocity = abs(velocity*1.25)
			string_vel = str(int(velocity))
			string_vel = direction + "0"*(3-len(string_vel)) + string_vel
			self.string_list[i] = string_vel

	def returnFinalMsg(self):

		final_msg = "S00000000"
		# final msg tekerle baslayacak

	 	for i in self.string_list:
	 		final_msg += i

	 	final_msg += 16*"0" + str(int(self.mode)) + str(int(self.activity_indicator))+ str(int(self.probe_servo))+"F"
	 	return final_msg 

	 #TODO: wheel to str method