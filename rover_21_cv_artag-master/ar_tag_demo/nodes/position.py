#!/usr/bin/env python
# -*- coding: UTF8 -*-

import numpy as np
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker


def quartenion2euler(data):
	arr = euler_from_quaternion(data) 

	position_ori = np.array(arr)

	return position_ori

def quartenion2euler(data):
	arr = quaternion_from_euler(data) 

	position_pos = np.array(arr)

	return position_pos



def rotate_by_y(arr,theta):
	rota_mat = np.array(((np.cos(theta), 0 ,-1* np.sin(theta)),
						 (0            , 1 , 0               ),
						 (np.sin(theta), 0 , np.cos(theta)   )))
	
	return np.matmul(rota_mat,arr)

	
#print rotate_by_y(np.array((1,0,0)),np.pi/2)