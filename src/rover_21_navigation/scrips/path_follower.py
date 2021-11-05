#!/usr/bin/env python

"""
	Date: 24.02.2021
	Berke Algul

	This is a prototype, and needs pygame
"""

import rospy 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion
from math import sin, cos, atan2, pi, sqrt

import pygame

class Controller:
	def __init__(self):
		rospy.Subscriber("/odometry/filtered_map", Odometry, self.odom_cb)
		rospy.Subscriber("/path", Path, self.path_cb)

		self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
		self.path = Path()
		self.yaw = 0.0
		self.max_yaw_error = pi / 4
		self.x = 0.0
		self.y = 0.0
		self.kp = 5.0
		self.L = 0.8
		self.t_y = 0.0
		self.t_x = 0
		self.last_target_index = 0

		#pygame stuff
		pygame.init()
		self.s = 200
		self.screen = pygame.display.set_mode((self.s, self.s))

		print("controller initialized")

		while not rospy.is_shutdown():
			#pygame stuff mandatory
			for event in pygame.event.get():
				if event.type == pygame.QUIT:
					pygame.quit()

			if len(self.path.poses) == 0:
				print("path empty")
				continue

			# phrase 1 pick target in the path
			self.pick_target()

			# if goal reached
			if self.last_path_index == len(self.path.poses):
				self.twist_pub.publish(Twist())
				print("goal reached")
				continue

			# phrase 2 seek target
			target_yaw = atan2(self.t_y, self.t_x)
			error_yaw = target_yaw - self.yaw
			error_yaw = atan2(sin(error_yaw), cos(error_yaw))

			twist = Twist()
			twist.angular.z = self.kp * error_yaw

			if abs(error_yaw) > self.max_yaw_error:
				twist.linear.x = 0.0
				print("too big ", (error_yaw * 180)/pi)
			else:
				twist.linear.x = 1.5

			self.twist_pub.publish(twist)

			#pygame stuff
			self.screen.fill((0,0,0))

			# red yaw
			yy = -50*sin(self.yaw)
			yx = 50*cos(self.yaw)
			pygame.draw.line(self.screen,(255,0,0),(self.s/2, self.s/2),(self.s/2+yx, self.s/2+yy),width=3)

			# green target yaw
			py = -50*sin(target_yaw)
			px = 50*cos(target_yaw)            
			pygame.draw.line(self.screen,(0,255,0),(self.s/2, self.s/2),(self.s/2+px, self.s/2+py),width=3)

			pygame.display.flip()

	def pick_target(self):
		#self.t_x = self.path.poses[1].pose.position.x - self.x
		#self.t_y = self.path.poses[1].pose.position.y - self.y

		for i in range(self.last_path_index, len(self.path.poses)):
			x = self.path.poses[i].pose.position.x
			y = self.path.poses[i].pose.position.y

			dx = x - self.x 
			dy = y - self.y

			d = sqrt(dx**2 + dy**2)

			if d >= self.L:
				self.t_x = dx
				self.t_y = dy
				self.last_path_index = i
				return

		self.last_path_index = len(self.path.poses)

	def path_cb(self, path):
		self.path = path
		self.last_path_index = 0
	
	def odom_cb(self, odom):
		self.x = odom.pose.pose.position.x
		self.y = odom.pose.pose.position.y
		_, _, self.yaw = euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])

if __name__ == "__main__":
	rospy.init_node("path_follower")
	Controller()