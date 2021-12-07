#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import math
import serial
import rosparam
from std_msgs.msg import String, Int16
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
import time
from SerialMessageClass import *
import subprocess



serial_obj = SerialMessageClass()
serial_obj.activity_indicator_final = 3 

pub = rospy.Publisher("/rover_serial_topic", String, queue_size=10)

ext_comm_check = 0

class CommCheck():
    def __init__(self):
        self.ext_comm_check = 0
        rospy.init_node('message_creator')
        rospy.Subscriber("joint_states/send",Float64MultiArray,self.jointCallback)
        rospy.Subscriber("/cmd_vel",Twist,self.wheelCallback)
        rospy.Subscriber("/comm_check",Int16,self.extCheckCb)
        rospy.Subscriber('/odometry/wheel',Odometry,self.main)
        self.pub_p = rospy.Publisher('/usta',String,queue_size=10)

        
        rospy.spin()

    def extCheckCb(self,data):
        self.ext_comm_check = rospy.Time.now().to_sec()
        #print(ext_comm_check)


    def jointCallback(self,data):
        
        mode_bit = data.data[-3]

        serial_obj.arm_mode = mode_bit
        serial_obj.activity_indicator = data.data[-2]
        serial_obj.probe_servo = data.data[-1]
        if mode_bit == 8:
            serial_obj.joint_positions = data.data[0:7]
        elif mode_bit == 5:
            serial_obj.joint_velocities = data.data[0:7]

    def wheelCallback(self,data):
        multiplier = 120
        serial_obj.twist_var.linear.x = data.linear.x * multiplier
        serial_obj.twist_var.angular.z = data.angular.z * multiplier
        rospy.loginfo(serial_obj.twist_var.angular.z)

    def main(self,data):

        time = rospy.Time.now().to_sec()
        
        print(str(time-self.ext_comm_check))
        if time - self.ext_comm_check < 0.5:
            
            if serial_obj.arm_mode == 5:
                serial_obj.jointVelocityToString()
                serial_obj.wheelVelToString()
            elif serial_obj.arm_mode == 8:
                serial_obj.jointAngleToString()
                serial_obj.wheelVelToString()
            elif serial_obj.arm_mode == 1:
                serial_obj.wheelVelToString()
                
            final_message = serial_obj.returnFinalMsg()
            rospy.loginfo(final_message)
            pub.publish(final_message)
            self.pub_p.publish(final_message)

        else:
            serial_obj.string_list = ["0000","0000","0000","0000","0000","0000","0000"]
            serial_obj.wheel_velocities = ["0000","0000"]
            final_message = serial_obj.returnFinalMsg()
            #print(final_message)
            final_message = "S"+"0000000010001000100010001000100010000000000000000000"+final_message[53] + "10"+ "F"
            self.pub_p.publish(final_message)
	    pub.publish(final_message)
            
            #self.pub_p.publish(str(final_message))
            
            #rospy.loginfo(final_message)
            #pub.publish(str(final_message))
            #self.pub_p.publish(final_message)
            
                
                
                
            
            
if __name__ == '__main__':
    CommCheck()


