#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from math import sqrt


class Normalizer():
	def __init__(self):
		
		rospy.init_node('normalizer')
		rospy.Subscriber('/imu/data_raw',Imu,self.imu_cb)
		self.pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
		rospy.spin()
	
	def imu_cb(self,data):
		if data.orientation.x != 0 and data.orientation.y != 0 and data.orientation.z != 0 and data.orientation.w != 0:
			self.ori_x = data.orientation.x
			self.ori_y = data.orientation.y
			self.ori_z = data.orientation.z			
			self.ori_w = data.orientation.w		
			
			total = self.ori_x**2 + self.ori_y**2 + self.ori_z**2 + self.ori_w**2
			self.ori_x_nor = self.ori_x / sqrt(total)
			self.ori_y_nor = self.ori_y / sqrt(total)
			self.ori_z_nor = self.ori_z / sqrt(total)
			self.ori_w_nor = self.ori_w / sqrt(total)
			
			Imu_msg = Imu()
			Imu_msg.orientation.x = self.ori_x_nor
			Imu_msg.orientation.y = self.ori_y_nor
			Imu_msg.orientation.z = self.ori_z_nor
			Imu_msg.orientation.w = self.ori_w_nor
			self.pub.publish(Imu_msg)
			
		
		
	
if __name__ == '__main__':
	Normalizer()
