#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import serial
import rosparam
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
import time
from SerialMessageClass import *

serial_obj = SerialMessageClass()

err_counter = 0

pub = rospy.Publisher("/rover_serial_topic", String, queue_size=10)

def jointCallback(data):
    
    mode_bit = data.data[-3]

    serial_obj.arm_node_check = True
    serial_obj.arm_mode = mode_bit
    serial_obj.activity_indicator = data.data[-2]
    serial_obj.probe_servo = data.data[-1]
    if mode_bit == 8:
        serial_obj.joint_positions = data.data[0:7]
    elif mode_bit == 5:
        serial_obj.joint_velocities = data.data[0:7]

def wheelCallback(data):
    multiplier = 120
    serial_obj.twist_var.linear.x = data.linear.x * multiplier
    serial_obj.twist_var.angular.z = data.angular.z * multiplier

def main():
    rospy.init_node('message_creator')
    rospy.Subscriber("joint_states/send",Float64MultiArray,jointCallback)
    rospy.Subscriber("cmd_vel",Twist,wheelCallback)
    #TODO: determine the topic name
    
    rate = rospy.Rate(20)
    
    while not rospy.is_shutdown():
        if serial_obj.arm_mode == 5:
            print("Arm node check: ",serial_obj.arm_node_check)
            if (serial_obj.arm_node_check):
		err_counter = 0
                serial_obj.jointVelocityToString()
            else:
		err_counter += 1
	    if err_counter == 10:
                serial_obj.string_list = ["1000","1000","1000","1000","1000","1000","1000"]
            serial_obj.wheelVelToString()
            serial_obj.arm_node_check = False
        elif serial_obj.arm_mode == 8:
            serial_obj.jointAngleToString()
            serial_obj.wheelVelToString()
            
        final_message = serial_obj.returnFinalMsg()
        rospy.loginfo(final_message)
        pub.publish(final_message)
        rate.sleep()
        
if __name__ == '__main__':
    main()
