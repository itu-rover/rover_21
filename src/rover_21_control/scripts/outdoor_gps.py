#!/usr/bin/env python2
# -- coding: utf-8 --

import rospy
import Queue
import math
import actionlib
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import NavSatFix  
from geometry_msgs.msg import PoseStamped, Point
import geonav_transform.geonav_conversions as gc
reload(gc)


class outdoor_gps():
	def __init__(self):
		rospy.init_node("gps_autonomous")
		self.x=0.0
		self.y=0.0
		self.z=0.0
		self.lat=0.0
		self.lon=0.0
		self.olat=0.0
		self.olon=0.0
		self.target_lat=0.0
		self.target_lon=0.0
		self.target_lat_rad=0.0
		self.target_lon_rad=0.0
		self.target_x=0.0
		self.target_y=0.0
		self.long_q = Queue.Queue()
		self.lat_q = Queue.Queue()
		self.origin_flag = False

		self.p = PoseStamped()
		self.p.header.seq = 1
		self.p.header.stamp = rospy.Time.now()
		self.p.header.frame_id = "odom"
		self.p.pose.position.x = 0.0
		self.p.pose.position.y = 0.0
		self.p.pose.position.z = 0.0
		self.p.pose.orientation.x = 0.0	
		self.p.pose.orientation.y = 0.0	
		self.p.pose.orientation.z = 0.0	
		self.p.pose.orientation.w = 1.0		

		self.m = MoveBaseActionResult()
		self.m.header.seq = 1
		self.m.header.stamp = rospy.Time.now()
		self.m.header.frame_id = ""
		self.m.status.text = ""
		
		rospy.Subscriber("/gps/filtered",NavSatFix,self.gps_cb)	
		rospy.Subscriber("/move_base/result",MoveBaseActionResult,self.move_base_cb)

		self.client = actionlib.SimpleActionClient("move_base",MoveBaseAction)
		self.client.wait_for_server()
		self.goal = MoveBaseGoal()
		self.goal.target_pose.header.frame_id = "map"
		

		self.main()


	def move_base_cb(self,data):
		while(self.lat_q.qsize() > 0 and self.long_q.qsize() > 0 and data.status.text == "Goal Reached"):	
			self.gps_to_xy()
			x = raw_input("Do you want to use coordinates as x:  " + str(self.target_x) + "  y:  " + str(self.target_y) + "? (y/n):")
			if x == "Y" or x == "y":
				print("New target: {},{}".format(self.target_x,self.target_y))
				self.p.pose.position.x = self.target_x
				self.p.pose.position.y = self.target_y
				self.p.header.seq = self.p.header.seq+1
				self.pub.publish(self.p)
			elif x == "N" or x == "n":
				print("Target won't be used")
				continue

	def gps_cb(self, data):
		self.lat = data.latitude
		self.lon = data.longitude
		#Origin chosen for ll/xy transform
		if not self.origin_flag:
			self.olat = self.lat
			self.olon = self.lon
			self.origin_flag = True
	
	def move_base_client(self):

		self.goal.target_pose.header.stamp = rospy.Time.now()
		#self.goal.target_pose.pose.position.x = self.target_x
		#self.goal.target_pose.pose.position.y = self.target_y		
		self.goal.target_pose.pose.position.x = 2.5 #fixed position
		self.goal.target_pose.pose.position.y = 2.5 #fixed

		self.goal.target_pose.pose.orientation.w = 1.0
		self.client.send_goal(self.goal)
		rospy.spin()

	def gps_input(self):
		self.a = int(input("How many coordinates do you want to enter?:"))
		for i in range(self.a):
			print("Please enter coordinates.")
			print("Latitude: ")
			self.lat_q.put(float(raw_input()))
			print("Longitude: ")
			self.long_q.put(float(raw_input()))
 	

	def gps_to_xy(self):
		#For outdoor
		#self.target_y, self.target_x = gc.ll2xy(self.target_lat,self.target_lon,self.olat,self.olon)
		#For Gazebo
		self.target_lat = self.lat_q.get()
		self.target_lon = self.long_q.get()
		self.target_y, self.target_x = gc.ll2xy(self.target_lat,self.target_lon,self.olat,self.olon)
		print('Lat: %.4f, Lon:%.4f >> X: %.2f, Y: %.2f'%(self.target_lat,self.target_lon,self.target_x,self.target_y))

	def main(self):
		self.gps_input()
		self.gps_to_xy()
		print("New target: {},{}".format(self.target_x,self.target_y))
		
		self.result = self.move_base_client()
		if self.result:
			rospy.loginfo("Goal execution done!")

if __name__=="__main__":
	outdoor_gps()
