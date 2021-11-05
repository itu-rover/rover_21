#!/usr/bin/env python
# -*- coding: UTF8 -*-

import rospy
import numpy as np
import tf2_ros
from tf import TransformBroadcaster
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from position import *

class is_gate(object):
	def __init__(self,left_gate,right_gate):
		rospy.init_node('is_gate')
		self.rate = rospy.Rate(10)

		self.gate= np.nan

		self.goal = PoseStamped()
		self.goal.header.seq = 1
		self.goal.header.stamp = rospy.Time.now()
		self.goal.header.frame_id = "camera_link"


		self.left_gate,self.right_gate = left_gate,right_gate
		self.is_left = False
		self.is_right= False
		self.gate_loc = (0,0,0)
		self.b = TransformBroadcaster()
		self.confused = True

		self.pub_pose = rospy.Publisher('/gate_loc', PoseStamped, queue_size=10)
		self.pub_pose = rospy.Publisher('/gate_angle', PoseStamped, queue_size=10)

		
		self.vehicle_loc= np.array((0,0,0)) 
		rospy.Subscriber('/odometry/filtered',Odometry, self.odom_cb)
		rospy.Subscriber('/visualization_marker', Marker, self.find_gate)
		

	def find_gate(self, data):

		gate = data.id
		print("aaaaaaaaaa" , gate)

		if gate== self.left_gate:
			rospy.loginfo("Found Left AR_Tag")
			self.is_left = True 
			
			self.loc_left = np.array((self.vehicle_loc[0] + data.pose.position.x,
					 			 	  self.vehicle_loc[1] + data.pose.position.y,
					 			 	  self.vehicle_loc[2] + data.pose.position.z))

			self.ori_left = np.array((data.pose.orientation.x,
								 	  data.pose.orientation.y,
								 	  data.pose.orientation.z,
					 			 	  data.pose.orientation.w))

			
		elif gate== self.right_gate:
			rospy.loginfo("Found right AR_Tag")
			self.is_right= True 

			self.loc_right= np.array((self.vehicle_loc[0] + data.pose.position.x,
					 			 	  self.vehicle_loc[1] + data.pose.position.y,
					 			 	  self.vehicle_loc[2] + data.pose.position.z))

			ori_right= np.array((data.pose.orientation.x,
								 data.pose.orientation.y,
								 data.pose.orientation.z,
					 			 data.pose.orientation.w))

		if self.is_left & self.is_right:
			rospy.loginfo("FOUND DA GATE!!!!!\nPublishing gate location")
			self.confused = False
			
			self.gate = ( rotate_by_y((np.add(self.loc_left , self.loc_right)/2),-1*np.pi/2),self.ori_left )


			self.run()

	def odom_cb(self,data):
		self.vehicle_loc[0]=data.pose.pose.position.x
		self.vehicle_loc[1]=data.pose.pose.position.y
		self.vehicle_loc[2]=data.pose.pose.position.z



	def run(self):

		while not rospy.is_shutdown() and not self.confused :
			
			self.goal.pose.position.x = self.gate[0][0]
			self.goal.pose.position.y = self.gate[0][1]
			self.goal.pose.position.z = self.gate[0][2]

			self.goal.pose.orientation.x = self.gate[1][0]
			self.goal.pose.orientation.y = self.gate[1][1]
			self.goal.pose.orientation.z = self.gate[1][2] 
			self.goal.pose.orientation.w = self.gate[1][3]

			self.pub_pose.publish(self.goal)
			self.rate.sleep()

da_gate = is_gate(19,7)

while not rospy.is_shutdown():
	da_gate.run()
	rospy.spin()