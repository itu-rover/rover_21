#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import rosnode
from std_msgs.msg import Float64, Int16
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String

class Arm:

    def __init__(self):
        self.mode = 0           # 0 SLOW, 1 FAST
        self.speed_frac = 100  # 100 SLOW,  150 FAST
        self.actuator_velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self.activity_mode = 1  # 1 IDLE, 2 TELEOPERATION, 3 AUTONOMOUS
        self.servo_action = 0   # 0 CLOSED, 1 OPEN
        self.probe_deploy_flg = 0
        self.probe_pickup_flg = 0
        self.teleop_start_time = 0
	self.last_movement = 0
    
    def checkJoyDeadband(self,data):
        if (abs(data) > 0.15):
            return True
        else: 
            return False

    def forwardKinematics(self,joy_data):
        if joy_data.buttons[0] != 0:
            fingers = joy_data.axes[0]
            self.actuator_velocities[0] = 0
            if (self.checkJoyDeadband(fingers)):
                self.actuator_velocities[6] = fingers*self.speed_frac
            else:
                self.actuator_velocities[6] = 0

        elif joy_data.buttons[1] != 0:
            axis_6 = joy_data.axes[0]
            self.actuator_velocities[0] = 0
            if (self.checkJoyDeadband(axis_6)):
                self.actuator_velocities[5] = -axis_6*self.speed_frac
            else:
                self.actuator_velocities[5] = 0
        
        elif joy_data.buttons[2] != 0:
            axis_5 = joy_data.axes[0]
            self.actuator_velocities[0] = 0
            if (self.checkJoyDeadband(axis_5)):
                self.actuator_velocities[4] = axis_5*self.speed_frac
            else:
                self.actuator_velocities[4] = 0

        elif joy_data.buttons[3] != 0:
            axis_4 = joy_data.axes[1]
            self.actuator_velocities[1] = 0
            if (self.checkJoyDeadband(axis_4)):
                self.actuator_velocities[3] = axis_4*self.speed_frac
            else:
                self.actuator_velocities[3] = 0 

        else:
            axis_1 = joy_data.axes[0]
            axis_2 = joy_data.axes[1]
            axis_3 = joy_data.axes[4]

            if self.checkJoyDeadband(axis_1):
                self.actuator_velocities[0] = axis_1*self.speed_frac/1.75
            else:
                self.actuator_velocities[0] = 0
            
            if self.checkJoyDeadband(axis_2):
                self.actuator_velocities[1] = axis_2*self.speed_frac
            else:
                self.actuator_velocities[1] = 0

            if self.checkJoyDeadband(axis_3):
                self.actuator_velocities[2] = axis_3*self.speed_frac
            else:
                self.actuator_velocities[2] = 0
            self.actuator_velocities[3] = 0
            self.actuator_velocities[4] = 0
            self.actuator_velocities[5] = 0
            self.actuator_velocities[6] = 0
	
            

    def returnActuatorVel(self):
        return_msg = Float64MultiArray(data=(self.actuator_velocities+[5]+[self.activity_mode]+[self.servo_action]))
        return return_msg


