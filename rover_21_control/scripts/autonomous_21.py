#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu, NavSatFix, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from math import sin, cos, atan2
import math

class AutonomousDrive():
	def __init__(self):
		#Init ros node
		rospy.init_node("autonomous_drive")

		#Get references for states
		self.gtg = GoToGoal(self)
		self.oa = ObstacleAvoidance(self)
		self.fw = FollowWall(self)

		self.goal = (-8.0, -3.0) #Init goal
		self.goal_threshold = 0.5 #Threshold to goal in meters

		#PID constants
		self.kp = 5.0
		self.ki = 0.1
		self.kd = 0.1

		#Time constants
		self.curr_time = rospy.Time.now()
		self.last_time = rospy.Time.now()
		self.dt = 0.001

		#Position and orientation variables
		self.x = 0.0
		self.y = 0.0

		self.yaw = 0.0

		self.lat = 0.0
		self.lon = 0.0

		#Scan variables
		self.latest_scan = None

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

		#Run main function
		self.main()

	#Odometry callback function
	def odom_cb(self, data):
		self.last_time = self.curr_time
		self.curr_time = rospy.Time.now()
		self.dt = (self.curr_time - self.last_time).to_sec()

		position = data.pose.pose.position #Position of the rover
		orientation = data.pose.pose.orientation #Quaternion orientation of the rover

		self.x, self.y = position.x, position.y

	#IMU callback function
	def imu_cb(self, data):
		q = data.orientation
		[roll, pitch, self.yaw] = euler_from_quaternion([q.x, q.y, q.z, q.w])
		self.yaw = self.fix_yaw(self.yaw)

	#Fix yaw to -pi, +pi interval
	def fix_yaw(self, yaw):
		if yaw > math.pi:
			while yaw > math.pi:
				yaw -= 2 * math.pi
		elif yaw < -math.pi:
			while yaw < -math.pi:
				yaw += 2 * math.pi

		return yaw

	#GPS callback function
	def gps_cb(self, data):
		self.lat = data.latitude
		self.lon = data.longitude

	#LiDAR callback function
	def lidar_cb(self, data):
		self.latest_scan = data.ranges

	#Set goal point
	def set_goal(self, point):
		self.goal = point

	#Find obstacles from latest scan data
	def detect_obstacles(self):
		pass

	#Go forward
	def go_forward(self):
		pass

	#Rotate to desired yaw angle
	def rotate(self, target_yaw):
		pass

	#Check if distance is lower than the threshold
	def control_dist(self):
		dist = math.sqrt(math.pow((self.goal[0] - self.x), 2) + math.pow((self.goal[1] - self.y), 2))
		return dist >= self.goal_threshold

	#Main function thats runs while roscore is available
	def main(self):
		while not rospy.is_shutdown():
			control = self.control_dist()
			if control:
				w = self.gtg.find_u_gtg()
				new_twist = Twist()
				new_twist.linear.x = 0.5
				new_twist.angular.z = w
				self.twist_pub.publish(new_twist)
			else:
				new_twist = Twist()
				new_twist.linear.x = 0.0
				new_twist.angular.z = 0.0
				self.twist_pub.publish(new_twist)
				self.goal = (5.0, 5.0)

class GoToGoal():
	#ad: Autonomous Drive object reference
	def __init__(self, ad):
		self.ad = ad

		self.e_K = 0.0 #Integration of errors
		self.e_k_1 = 0.0 #Last error

	#Find go to goal behaviour unit vector
	def find_u_gtg(self, verbose = False):
		#Unit vector to goal
		self.u_gtg = np.float32([self.ad.goal[0] - self.ad.x, self.ad.goal[1] - self.ad.y])
		self.u_gtg /= np.linalg.norm(self.u_gtg)

		#Angle between goal and current position
		self.theta_g = self.ad.fix_yaw(atan2(self.u_gtg[1], self.u_gtg[0]))

		#Calculate yaw error and fix it
		self.theta_g_error = self.theta_g - self.ad.yaw
		self.theta_g_error = self.ad.fix_yaw(atan2(sin(self.theta_g_error), cos(self.theta_g_error)))

		#Proportional error
		self.e_P = self.theta_g_error
		#Integration error
		self.e_I = (self.e_K + self.e_P) * 1e-3 #1e-3 is constant as dt
		#Derivative error
		self.e_D = (self.e_P - self.e_k_1) * 1e3 #1e3 is constant as dt

		#Calculate angular velocity
		self.w = self.ad.kp * self.e_P + self.ad.ki * self.e_I + self.ad.kd * self.e_D
		#Assign calculated values as old values to keep them
		self.e_K = self.e_I
		self.e_k_1 = self.e_P
		
		#If verbose, print debug values
		if verbose:
			print("e_P: {}, e_I: {}, e_D: {}, w: {}".format(self.e_P, self.e_I, self.e_D, self.w))

		return self.w

class ObstacleAvoidance():
	#ad: Autonomous Drive object reference
	def __init__(self, ad):
		self.ad = ad

	#Find obstacle avoidance behaviour unit vector
	def find_u_oa(self):
		pass

class FollowWall:
	#ad: Autonomous Drive object reference
	def __init__(self, ad):
		self.ad = ad

	#Find follow wall behaviour unit vector
	def find_u_fw(self):
		pass

if __name__ == '__main__':
	ad = AutonomousDrive()