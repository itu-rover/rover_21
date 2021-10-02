#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import serial
import time
from std_msgs.msg import String

serialMsg = " "
serialMsg2 = " "
# get data from pc
def serialCallback(data):
    global serialMsg
    serialMsg = data.data


def serialCallback2(data):
    global serialMsg2
    serialMsg2 = data.data


def letsSerial():
    rospy.init_node("serial_rover_arm")
    serialString1 = "/dev/ttyUSB0"  # 0-1-2-5-6. eksenler için
    serialString2 = "/dev/ttyUSB1"  # 3-4. eksenler için
    rate = rospy.Rate(1)
    rospy.Subscriber(
        "/rover_serial/arm_01256", String, serialCallback
    )  # 0-1-2-5-6. eksenler için
    rospy.Subscriber(
        "/rover_serial/arm_34", String, serialCallback2
    )  # 3-4. eksenler için

    printOnce = True

    while not rospy.is_shutdown():

        ser = serial.Serial(
            port=serialString1,
            baudrate=int(115200),
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )  # open serial
        ser2 = serial.Serial(
            port=serialString2,
            baudrate=int(baudrate2),
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )  # open serial
        ser.timeout = 1
        ser2.timeout = 1

        while ser.isOpen() and not rospy.is_shutdown():

            if printOnce == True:
                print("serial1 is open")
                print(namespace + "sial2 is open")
                printOnce = False

            ser.write(serialMsg2 + "\n")
            ser2.writelines(serialMsg2 + "\n")

            ser.flushInput()
            ser.flushOutput()
            ser2.flushInput()
            ser2.flushOutput()

            print("Writing to 1 : " + str(serialMsg2))
            print("Writing to 2 : " + str(serialMsg2))
            rate.sleep()


rospy.spin()


if __name__ == "__main__":
    try:
        letsSerial()
    except rospy.ROSInterruptException:
        pass
