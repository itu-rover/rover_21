#!/usr/bin/env python
# -*- coding: utf-8 -*-

#https://www.youtube.com/watch?v=IUfX5XMqnTc

import rospy
from sensor_msgs.msg import Imu, NavSatFix, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from math import sin, cos, atan2
import math
from collections import defaultdict

class AutonomousDrive:
	def __init__(self):
		#Init ros node
		rospy.init_node("autonomous_drive")

		#Get references for states
		self.gtg = GoToGoal(self)
		self.oa = ObstacleAvoidance(self)
		self.fw = FollowWall(self)

		self.alpha = 0.55 #Object avoidance coefficient
		self.beta = 0.5
		self.epsilon = 1e-5
		self.oa_threshold = 1.25 #Object avoidance threshold

		self.goal = (20.0, 20.0) #Init goal
		self.goal_threshold = 0.5 #Threshold to goal in meters

		#PID constants
		self.kp = 5.0
		self.ki = 0.1
		self.kd = 0.1

		self.e_K = 0.0 #Integration of errors
		self.e_k_1 = 0.0 #Last error

		#Time constants
		self.dt = 1e-3

		#Position and orientation variables
		self.x = 0.0
		self.y = 0.0

		self.yaw = 0.0

		self.lat = 0.0
		self.lon = 0.0

		#Scan variables
		self.latest_scan = None
		self.scan_min_angle = -math.pi / 2 #Minimum scan angle
		self.scan_max_angle = math.pi / 2 #Maximum scan angle
		self.angle_increment = math.pi / 314.16

		#Define subscriber topics
		self.odom_topic = "/odometry/filtered"
		self.imu_topic = "/imu/data"
		self.gps_topic = "/gps/fix"
		self.lidar_topic = "/scan"
		#Define publisher topics
		self.twist_topic = "/cmd_vel"

		#Initialize subscribers
		rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)
		rospy.Subscriber(self.imu_topic, Imu, self.imu_cb)
		rospy.Subscriber(self.gps_topic, NavSatFix, self.gps_cb)
		rospy.Subscriber(self.lidar_topic, LaserScan, self.lidar_cb)
		#Initialize publishers
		self.twist_pub = rospy.Publisher(self.twist_topic, Twist, queue_size = 10)
		self.u_g_pub = rospy.Publisher("/debug/u_g", String, queue_size = 10) #For debugging
		self.omega_pub = rospy.Publisher("/debug/omega", String, queue_size = 10) #For debugging
		self.alpha_pub = rospy.Publisher("/debug/alpha", String, queue_size = 10) #For debugging

		#Run main function
		self.main()

	#Odometry callback function
	def odom_cb(self, data):
		position = data.pose.pose.position #Position of the rover
		orientation = data.pose.pose.orientation #Quaternion orientation of the rover

		self.x, self.y = position.x, position.y

	#IMU callback function
	def imu_cb(self, data):
		q = data.orientation #Orientation in quaternion
		[roll, pitch, self.yaw] = euler_from_quaternion([q.x, q.y, q.z, q.w])
		self.yaw = self.fix_yaw(self.yaw)

	#Fix yaw to -pi, +pi interval
	def fix_yaw(self, yaw):
		return atan2(sin(yaw), cos(yaw))

	#GPS callback function
	def gps_cb(self, data):
		self.lat = data.latitude
		self.lon = data.longitude

	#LiDAR callback function
	def lidar_cb(self, data):
		self.latest_scan = data.ranges

	#2D rotation matrix
	def rotate_matrix(self, angle):
		a = self.fix_yaw(angle)
		r = np.float32([[cos(a), -sin(a)], [sin(a), cos(a)]])

		return r

	#Set goal point
	def set_goal(self, point):
		self.goal = point

	#Find obstacles from latest scan data
	def detect_obstacles(self):
		#Dictionary with default float key
		self.distances = defaultdict(float)

		#Group points by 15 and find the closest point
		for i in range(0, len(self.latest_scan), 15):
			min_dist = min(self.latest_scan[i: i + 14])

			#Calculate yaw of the mid-point of the current point group
			yaw = self.fix_yaw(self.scan_min_angle + (i / 2) * self.angle_increment)
			self.distances[yaw] = min_dist

		return self.distances

	#Check if distance is lower than the threshold
	def control_dist(self):
		dist = math.sqrt(math.pow((self.goal[0] - self.x), 2) + math.pow((self.goal[1] - self.y), 2))
		return dist >= self.goal_threshold

	#Find desired vector to the goal point
	def find_u_g(self):
		self.u_gtg = self.gtg.find_u_gtg() #Get go-to-goal unit vector
		self.u_oa = self.oa.find_u_oa() #Get object avoidance unit vector

		#Calculate "to goal" vector by using alpha coeffient
		#self.alpha = 1 - np.exp(self.beta * min(self.distances))
		self.alpha_pub.publish(str(self.alpha)) #For debugging
		self.u_g = self.alpha * self.u_oa + (1 - self.alpha) * self.u_gtg

		self.u_g_pub.publish("{}{}".format(self.u_g[0, 0], self.u_g[0, 1])) #For debugging

		return self.u_g

	#Navigate rover with calculated goal behaviour
	def navigate(self, verbose = False):
		#Angle between goal and current position
		self.theta_g = atan2(self.u_g[0, 1], self.u_g[0, 0])

		#Calculate yaw error and fix it
		self.theta_g_error = self.theta_g - self.yaw
		self.theta_g_error = self.fix_yaw(self.theta_g_error)

		#Proportional error
		self.e_P = self.theta_g_error
		#Integration error
		self.e_I = (self.e_K + self.e_P) * self.dt
		#Derivative error
		self.e_D = (self.e_P - self.e_k_1) / self.dt

		#Calculate angular velocity
		self.w = self.kp * self.e_P + self.ki * self.e_I + self.kd * self.e_D
		#Assign calculated values as old values to keep them
		self.e_K = self.e_I
		self.e_k_1 = self.e_P
		
		#If verbose, print debug values
		if verbose:
			print("e_P: {}, e_I: {}, e_D: {}, w: {}".format(self.e_P, self.e_I, self.e_D, self.w))

		self.omega_pub.publish(str(self.w)) #For debugging

		return self.w

	#Main function thats runs while roscore is available
	def main(self):
		while not rospy.is_shutdown():
			control = self.control_dist()

			#Wait until a lidar scan is read
			if self.latest_scan is not None:
				self.detect_obstacles() #Construct obstacles dictionary

				if control:
					self.find_u_g() #Calculate "to goal" vector
					fw = self.fw.check_fw()
					#print(self.fw.find_u_fw())
					#fw = False

					if fw:
						self.u_g = self.fw.find_u_fw()
						w = self.navigate()
					else:
						w = self.navigate() #Get angular velocity using PID

					#Publish new speed values
					new_twist = Twist()
					new_twist.linear.x = 0.5
					new_twist.angular.z = w
					self.twist_pub.publish(new_twist)
				else:
					#Reset speed values and assign new goal point
					new_twist = Twist()
					new_twist.linear.x = 0.0
					new_twist.angular.z = 0.0
					self.twist_pub.publish(new_twist)
					self.goal = (5.0, 5.0)

class GoToGoal:
	#ad: Autonomous Drive object reference
	def __init__(self, ad):
		self.ad = ad

	#Find go to goal behaviour unit vector
	def find_u_gtg(self):
		#Unit vector to goal
		self.u_gtg = np.float32([[self.ad.goal[0] - self.ad.x, self.ad.goal[1] - self.ad.y]])
		self.u_gtg /= np.linalg.norm(self.u_gtg)

		return self.u_gtg

	def navigate(self, verbose = False):
		#Angle between goal and current position
		self.theta_g = atan2(self.u_gtg[0, 1], self.u_gtg[0, 0])

		#Calculate yaw error and fix it
		self.theta_g_error = self.theta_g - self.ad.yaw
		self.theta_g_error = self.ad.fix_yaw(self.theta_g_error)

		#Proportional error
		self.ad.e_P = self.theta_g_error
		#Integration error
		self.ad.e_I = (self.ad.e_K + self.ad.e_P) * self.ad.dt
		#Derivative error
		self.ad.e_D = (self.ad.e_P - self.ad.e_k_1) / self.ad.dt

		#Calculate angular velocity
		self.ad.w = self.ad.kp * self.ad.e_P + self.ad.ki * self.ad.e_I + self.ad.kd * self.ad.e_D
		#Assign calculated values as old values to keep them
		self.ad.e_K = self.ad.e_I
		self.ad.e_k_1 = self.ad.e_P
		
		#If verbose, print debug values
		if verbose:
			print("e_P: {}, e_I: {}, e_D: {}, w: {}".format(self.ad.e_P, self.ad.e_I, self.ad.e_D, self.ad.w))

		return self.ad.w

class ObstacleAvoidance:
	#ad: Autonomous Drive object reference
	def __init__(self, ad):
		self.ad = ad
		self.epsilon = 1e-7 #Use epsilon to overcome division by zero

	#Find obstacle avoidance behaviour unit vector
	def find_u_oa(self):
		#Unit vectors to avoid obstacle
		u_array = [np.float32([-1., -1.]) / math.sqrt(2) for i in range(len(self.ad.distances))]
		yaw = self.ad.yaw

		#Sort and enumerate distance dictionary
		for i, d in enumerate(sorted(self.ad.distances)):
			#If distance to the obstacle is greater than threshold, do not use it
			if self.ad.distances[d] > self.ad.oa_threshold:
				u_array[i] = np.float32([0., 0.])
			else:
				#Calculate angle of the obstacle and rotate the unit vector			
				angle = self.ad.fix_yaw(yaw + d)
				u_array[i] = np.dot(self.ad.rotate_matrix(angle), u_array[i])

		#Get unit vector of the sum of the object avoidance vectors
		sum_vector = np.sum(u_array, axis = 0, keepdims = True) / (np.linalg.norm(u_array) + self.epsilon)

		self.u_oa = sum_vector

		return self.u_oa	

class FollowWall:
	#ad: Autonomous Drive object reference
	def __init__(self, ad):
		self.ad = ad

	#Check if switching to the follow wall behaviour is necessary
	def check_fw(self):
		self.ad.find_u_g() #Calculate u_gtg and u_oa

		r = np.dot(self.ad.u_gtg, self.ad.u_oa.T)

		return r < 0

	#Find follow wall behaviour unit vector
	def find_u_fw(self):
		self.ad.find_u_g() #Calculate u_oa and u_gtg

		L_f2_g = (1 - self.ad.alpha) * np.dot(self.ad.u_oa, self.ad.u_oa.T)
		L_f1_g = self.ad.alpha * np.dot(self.ad.u_oa, self.ad.u_gtg.T)

		self.u_fw = (1 / (L_f2_g - L_f1_g)) * (L_f2_g * self.ad.u_gtg - L_f1_g * self.ad.u_oa)

		return self.u_fw
 
if __name__ == '__main__':
	ad = AutonomousDrive()