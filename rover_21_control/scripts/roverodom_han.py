#!/usr/bin/env python2
# -- coding: utf-8 --
"""
----------
| NOTES  |
----------
Th hesabi yapilacak!!

WIP = Work in Progress
"""

import math
from math import sin, cos, pi, sqrt, pow
import rospy
import tf
import numpy as np
#import tf2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import String
from sensor_msgs.msg import Imu, JointState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker

class Localization(object):
	def __init__(self):
		rospy.init_node('odom_publisher')

		#x, y, z positions of the rover
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0

		self.th = 0.0 #yaw angle calculated from wheel encoder data		
		self.v = 0.0
		self.vx = 0.0
		self.vy = 0.0
		self.vth = 0.0 #angular velocity

		#known locations of ar-tag
		self.artag= {}
		self.artag["1"] = ( 9.80,  0.00)
		self.artag["2"] = ( 9.80,  3.50)
		self.artag["3"] = (34.00,  1.50)
		self.artag["4"] = (23.63, -4.62)
		self.artag["5"] = (10.27,  9.76)
		self.artag["6"] = (10.10,-21.38)
		self.artag["7"] = ( 5.44,-15.17)
		self.artag["8"] = (31.00, -9.13)
		self.artag["9"] = (18.37, 11.00)
		self.artag["10"]= ( 1.36,  9.60)
		self.artag["11"]= (17.00,-22.46)
		self.artag["12"]= (19.63, -0.02)
		self.artag["13"]= (18.29,-13.90)
		self.artag["15"]= ( 3.02,-17.34)
		
		#a counter to prevent ar-tag vibration
		self.last_time=rospy.get_rostime()

		self.dist_btw_wheels = 0.85 #0.85 for rover // 0.55 for husky #distance between wheels in meters
		self.surrounding_of_wheel_new = 0.12 * math.pi * 2 #0.135 for rover21 // 0.155 for rover20 // 0.165 for husky #surrounding of the wheels in meters
		self.surrounding_of_wheel_old = 0.12 * math.pi * 2

		self.alpha = 45.072 * math.pi / 180
		self.beta = math.pi / 2 - self.alpha

		self.init_yaw = 0 #Initial yaw angle of the rover
		self.last_yaw = 0
		self.curr_yaw = 0
		self.yaw_change = 0
		self.yaw_counter = 0

		#Wheel velocities
		self.front_left = 0
		self.back_left = 0
		self.front_right = 0
		self.back_right = 0
		self.left_wheel = 0.0
		self.right_wheel= 0.0

		self.frequency = 10 #Controller work frequency
		self.encoder_data = ""
		self.flag = 0 #Imu callback flag to check if initial yaw is calculated.
		#0 -> not calculated, 1 -> calculated

		self.current_time =  rospy.Time.now()
		self.last_time =  rospy.Time.now()
		self.odom_cur = Odometry()

		self.odom_pub = rospy.Publisher('/odometry/wheel', Odometry, queue_size = 10)
		rospy.Subscriber('/rover_serial_encoder', String, self.serial_callback)
		rospy.Subscriber('/imu/data', Imu, self.imu_cb)
		
		self.controller()
	
	#data: IMU message
	def imu_cb(self, data):
		self.last_yaw = self.curr_yaw
		#Read quaternion orientation from IMU and convert to euler angles
		[self.curr_roll, self.curr_pitch, self.curr_yaw] = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]) #Convert quaternion to euler angles
		#print(self.curr_yaw)
		#Calculate initial yaw angle by taking average of first 5 measurements
		if (self.flag == 0) and self.yaw_counter < 5:
			self.yaw_counter += 1
			self.init_yaw += self.curr_yaw

			if self.yaw_counter == 5:
				self.init_yaw /= 5
				self.init_yaw += math.pi / 2
				self.flag = 1
		else:
			#IMU is using magnetic east as reference, to overcome this, we add 90 degrees to our measurement
			#and we get magnetic north
			self.curr_yaw += math.pi / 2
			self.yaw_change = self.curr_yaw - self.last_yaw
		
	#Encoder callback
	#data: Serial messageimport math
 
	def euler_from_quaternion(self,x, y, z, w):
		"""
		Convert a quaternion into euler angles (roll, pitch, yaw)
		roll is rotation around x in radians (counterclockwise)
		pitch is rotation around y in radians (counterclockwise)
		yaw is rotation around z in radians (counterclockwise)
		"""
		t0 = +2.0 * (w * x + y * z)
		t1 = +1.0 - 2.0 * (x * x + y * y)
		roll_x = math.atan2(t0, t1)
	     
		t2 = +2.0 * (w * y - z * x)
		t2 = +1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		pitch_y = math.asin(t2)
	     
		t3 = +2.0 * (w * z + x * y)
		t4 = +1.0 - 2.0 * (y * y + z * z)
		yaw_z = math.atan2(t3, t4)
	     
		return roll_x, pitch_y, yaw_z # in radians
	
	def ar_odom_tuner(self,data):

		now = rospy.get_rostime()

		tag_id= data.id
		
		#if self.last_time - now < 3:
		#	return 0
		
		if ((( 1 > tag_id ) and (tag_id > 15)) and tag!=14):
			return 0

		seen_x = data.pose.position.z   
		seen_y = -data.pose.position.y   
		seen_z = -data.pose.position.x   
		
		self.x = self.artag[str(tag_id)][0] - seen_x
		self.y = self.artag[str(tag_id)][1] - seen_y

		self.last_time = now
		
		 
		
	def quaternion_from_euler(self,yaw, pitch, roll):
		qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
		qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
		qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
		qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
		return qx, qy, qz, qw

	def serial_callback(self,data):
		self.encoder_data = data.data
		self.splitted = data.data.split(',')

		#Calculate velocities from encoder data
		if(self.splitted[0] == 'A'):
			if(float(self.splitted[1]) >= 1000):
				self.front_left = self.surrounding_of_wheel_old*(-(float(self.splitted[1])-1000)) / 60
			if(float(self.splitted[1]) < 1000):
				self.front_left = self.surrounding_of_wheel_old*(float(self.splitted[1])) / 60

			if(float(self.splitted[2]) >= 1000):
				self.back_left = self.surrounding_of_wheel_new*(-(float(self.splitted[2])-1000)) / 60
			if(float(self.splitted[2]) < 1000):
				self.back_left = self.surrounding_of_wheel_new*(float(self.splitted[2])) / 60

			if(float(self.splitted[3]) >= 1000):
				self.front_right = self.surrounding_of_wheel_old*(-(float(self.splitted[3])-1000)) / 60
			if(float(self.splitted[3]) < 1000):
				self.front_right = self.surrounding_of_wheel_old*(float(self.splitted[3])) / 60

			if(float(self.splitted[4]) >= 1000):
				self.back_right = self.surrounding_of_wheel_new*(-(float(self.splitted[4])-1000)) / 60
			if(float(self.splitted[4]) < 1000):
				self.back_right = self.surrounding_of_wheel_new*(float(self.splitted[4])) / 60

	def controller(self):
		rate = rospy.Rate(self.frequency) #10 Hz

		while not rospy.is_shutdown():
			self.current_time = rospy.Time.now()
			self.dt = (self.current_time - self.last_time).to_sec() #Calculate time difference
			self.last_time = self.current_time

			#Calculate average side velocities
			self.right_wheel = ((self.front_right + self.back_right) / 2)
			self.left_wheel = ((self.front_left + self.back_left) / 2)

			self.vx = ((self.right_wheel + self.left_wheel) / 2)
			self.vy = 0
			self.vth = ((self.right_wheel - self.left_wheel) / self.dist_btw_wheels) #Angular velocity
			
			#Yaw change
			if self.vth < 0:
				direction = 1 #Right
				self.delta_th = abs(self.vth) * self.dt * -1
			else:
				direction = 0 #Left
				self.delta_th = abs(self.vth) * self.dt

			self.th += self.delta_th #Calculate total yaw change from the start

			#Position change
			self.delta_x = (self.vx * cos(self.curr_yaw - self.init_yaw) * self.dt)*1.05
			self.delta_y = (self.vx * sin(self.curr_yaw - self.init_yaw) * self.dt)*1.13

			#If velocity difference is greater than 5, calculate expected error
			if abs(self.right_wheel - self.left_wheel) > 5:
				self.delta_x *= (1 - sin((math.pi + self.yaw_change - 2 * self.beta) / 2))
				self.delta_y *= (1 - cos((math.pi + self.yaw_change - 2 * self.beta) / 2))
			
			#Total position change
			self.x += self.delta_x
			self.y += self.delta_y

			#If Ar-tag is seen changing odom
			rospy.Subscriber('/visualization_marker', Marker,self.ar_odom_tuner)
			
			#Debug calculated velocities
			#print(str(self.front_left) + ", " + str(self.back_left) + ", " + str(self.front_right) + ", " + str(self.back_right))
			print(str(self.x) + ", " + str(self.y))
			#Odometry message is created with calculated parameters
			self.q = quaternion_from_euler(0, 0, (self.curr_yaw - self.init_yaw)) #Rotation of the rover in quaternions
			self.odom = Odometry()
			self.odom.header.stamp = self.current_time
			self.odom.header.frame_id = "odom"
			self.odom.pose.pose = Pose(Point(self.x, self.y, self.z), Quaternion(*self.q)) #Position of the rover
			self.odom.child_frame_id = "base_link"
			self.odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth)) #Velocity of the rover

			#Publish odometry message and reset encoder data
			self.odom_pub.publish(self.odom)
			self.encoder_data = ""

			rate.sleep()

if __name__ == '__main__':
	Localization()
