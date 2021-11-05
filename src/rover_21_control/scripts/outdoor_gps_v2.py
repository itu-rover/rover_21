#!/usr/bin/env python2
# -- coding: utf-8 --

import rospy
import math
import actionlib
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import NavSatFix  
import geonav_transform.geonav_conversions as gc
reload(gc)

class outdoor_gps():
	def __init__(self):
		rospy.init_node("gps_autonomous")
		#parameters to calculate origin gps coordinates
		self.origin_lat = 0.0
		self.origin_lon = 0.0
		self.total_lat = 0
		self.total_lon = 0
		self.gps_cb_counter = 0
		
		self.num_of_gps = 0
		self.lat_list = []
		self.lon_list = []
		#target parameters
		self.target_lat = 0.0
		self.target_lon = 0.0
		self.target_x = 0.0
		self.target_y = 0.0
		self.x_list = []
		self.y_list = []

		self.goal_range = 15.0 #Determined by us, move_base range is 20 meters.
		self.iteration_x = 0
		self.iteration_y = 0
		self.iteration = 0
		self.x_ls = []
		self.y_ls = []
		self.x_flag =True

		self.mb_it = 0 #move_base target iterator

		rospy.Subscriber("/gps/filtered",NavSatFix,self.gps_cb)	
		rospy.sleep(2.0)		
		rospy.Subscriber("/move_base/result",MoveBaseActionResult,self.move_base_client)

		self.client = actionlib.SimpleActionClient("move_base",MoveBaseAction)
		self.client.wait_for_server()
		self.goal = MoveBaseGoal()
		self.goal.target_pose.header.frame_id = "odom"
		
		self.main()

	def gps_cb(self, data):#Callback function for gps data
		self.total_lat = self.total_lat + data.latitude
		self.total_lon = self.total_lon + data.longitude
		self.gps_cb_counter = self.gps_cb_counter+1
		#Origin chosen for ll/xy transform
		if self.gps_cb_counter == 60:
			self.origin_lat = self.total_lat/60
			self.origin_lon = self.total_lon/60

	def gps_input(self):#Input function for gps coordinates
		self.num_of_gps = int(input("How many coordinates do you want to enter?:"))
		for i in range(self.num_of_gps):
			print("Please enter coordinates.")
			print("Latitude: ")
			self.lat_list.append(float(raw_input()))
			print("Longitude: ")
			self.lon_list.append(float(raw_input()))

	def gps_to_xy(self):#GPS to XY convert function
		for i in range(self.num_of_gps):
			self.target_lat = self.lat_list[i]
			self.target_lon = self.lon_list[i]
			if i ==0:
				self.target_x, self.target_y = gc.ll2xy(self.target_lat,self.target_lon,self.origin_lat,self.origin_lon)
			else:
				self.target_x, self.target_y = gc.ll2xy(self.target_lat,self.target_lon,self.lat_list[i-1],self.lon_list[i-1])	
			self.x_list.append(self.target_x)
			self.y_list.append(self.target_y)
	
	def list_main_targets(self):#Target list function
		for i in range(self.num_of_gps):
			print("One of Main Target"+str(i+1)+":"+"  x:"+str(self.x_list[i])+"  y:"+str(self.y_list[i]))

	def path_divide(self):#If target is out of goal_range, this function divides and sends goal to move_base 
		for i in range(self.num_of_gps):
			if self.x_list[i] > self.goal_range or self.y_list[i] > self.goal_range:
				self.iteration_x = int(self.x_list[i]/self.goal_range)
				self.iteration_y = int(self.y_list[i]/self.goal_range)
				if self.iteration_x > self.iteration_y:
					self.iteration = self.iteration_x
					self.x_flag = True
				else:
					self.iteration = self.iteration_y
					self.x_flag = False
				for j in range(self.iteration):
					if self.x_flag == True:
						self.x_ls.append(self.goal_range)
						self.y_ls.append((self.y_l[i]*self.goal_range)/self.x_list[i])
					if self.x_flag == True:
						self.x_ls.append((self.x_list[i]*self.goal_range)/self.y_list[i])
						self.y_ls.append(self.goal_range)
			self.x_ls.append(self.x_list[i])
			self.y_ls.append(self.y_list[i])

	def list_all_targets(self):#All targets list function
		for i in range(len(self.x_ls)):
			print("One of Target"+str(i+1)+":"+"  x:"+str(self.x_ls[i])+"  y:"+str(self.y_ls[i]))


	def move_base_client(self):#move_base communication function
		self.goal.target_pose.header.stamp = rospy.Time.now()
		self.goal.target_pose.pose.position.x = self.x_ls[self.mb_it]
		self.goal.target_pose.pose.position.y = self.y_ls[self.mb_it]
		self.goal.target_pose.pose.orientation.w = 1.0
		self.client.send_goal(self.goal)			
		self.mb_it = self.mb_it +1



	def main(self):#main function
		self.gps_input()
		self.gps_to_xy()
		self.list_main_targets()
		self.list_all_targets()
		self.path_divide()
		self.result = self.move_base_client()
		if self.result:
			rospy.loginfo("Goal Execution done!")

if __name__ == '__main__':
	outdoor_gps()
