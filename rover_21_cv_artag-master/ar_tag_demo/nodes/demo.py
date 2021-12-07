#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf2_ros
from tf import TransformBroadcaster
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def get_odom(data):
    global odom_arr
    odom_arr = np.array((data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z))


def demo(data):
    rospy.Subscriber('/rtabmap/odom',Odometry, get_odom)
    #print odom_arr

    marker_id = data.id
    y = - data.pose.position.x + odom_arr[1]
    z = - data.pose.position.y + odom_arr[2]
    x = data.pose.position.z   + odom_arr[0]
    loc_arr = (x,y,z)
    ori_x = data.pose.orientation.x
    ori_y = data.pose.orientation.y
    ori_z = data.pose.orientation.z
    ori_w = data.pose.orientation.w
    ori_arr=(ori_y,ori_x,ori_w,ori_z)

    ori_arr_euler = euler_from_quaternion(ori_arr)

    #print(loc_arr)
    b.sendTransform(loc_arr,
                    ori_arr,
                    rospy.Time.now(),
                    str(marker_id),
                    'camera')

rospy.init_node("demo_node")
rate = rospy.Rate(10)
b = TransformBroadcaster()
rospy.Subscriber('/visualization_marker', Marker, demo)

rospy.spin() ### surekli olarak sub topicleri kontrol ediyor