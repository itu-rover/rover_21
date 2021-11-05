#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

class TF_Transformer():
    def __init__(self):
        rospy.init_node('tf_transformer')
        rospy.Subscriber('/imu/data',Imu,self.imu_cb)
        rospy.Subscriber('/odometry/filtered',Odometry,self.odom_cb)
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

        self.pose_x = 0
        self.pose_y = 0
        self.pose_z = 0
        
        rospy.spin()
        
    def imu_cb(self,data):
        print('imu geldi')
        try:
            (self.trans,self.rot) = self.listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
        except:
            rospy.logerr('No Transform Between map frame and odom frame')

        self.ori_x = data.orientation.x
        self.ori_y = data.orientation.y
        self.ori_z = data.orientation.z
        self.ori_w = data.orientation.w

        self.br.sendTransform((self.pose_x,self.pose_y,self.pose_z),(self.ori_x,self.ori_y,self.ori_z,self.ori_w),rospy.Time.now(),'base_link','odom')
    
    def odom_cb(self,data):
        print('odom geldi')
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y
        self.pose_z = data.pose.pose.position.z




        

if __name__ == '__main__':
    TF_Transformer()

   


