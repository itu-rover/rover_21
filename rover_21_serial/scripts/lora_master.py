#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Writes gps coordinates to lora
#ITU Rover Team

import rospy
import serial
import time
from std_msgs.msg import String



def letsSerial():
    rospy.init_node("serial_rover_lora_master")
    printOnce=True
    while not rospy.is_shutdown():

        ser = serial.Serial(port="/dev/ttyUSB0", baudrate=9600, parity=serial.PARITY_NONE,
                                stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)  # open serial
          
        ser.timeout = 1

          

        while ser.isOpen()  and not rospy.is_shutdown():

            if printOnce == True :
                print("serial1 is open")
                printOnce = False

            receive = ser.readline()

            ser.writelines("49.1234,29.3458"+"\n")
            ser.flushInput()
            ser.flushOutput()
              
            rospy.loginfo(" Writing to 1 : " + "49.1234,29.3458")
                
    rospy.spin()



if __name__ == '__main__':
    try:
        letsSerial()
    except rospy.ROSInterruptException:
        pass