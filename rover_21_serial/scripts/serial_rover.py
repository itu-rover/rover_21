#!/usr/bin/env python
# -*- coding: utf-8 -*-

# First entered port is always Serial One. Switch ports in case of need.

import rospy
import serial
import time
from std_msgs.msg import String
import rosparam
from serial_rover_config import *

namespace = "[RoverSerial : ] "
serialMsg = ""
serialMsg2 = ""
serialMsg3 = ""

serialString1 = rospy.get_param("RoverSerial/ports/serialString1")
serialString2 = rospy.get_param("RoverSerial/ports/serialString2")
serialString3 = rospy.get_param("RoverSerial/ports/serialString3")

sub_topic1 = rospy.get_param("RoverSerial/sub_topics/sub_topic1")
sub_topic2 = rospy.get_param("RoverSerial/sub_topics/sub_topic2")
sub_topic3 = rospy.get_param("RoverSerial/sub_topics/sub_topic3")

pub_topic1 = rospy.get_param("RoverSerial/pub_topics/pub_topic1")
pub_topic2 = rospy.get_param("RoverSerial/pub_topics/pub_topic2")
pub_topic3 = rospy.get_param("RoverSerial/pub_topics/pub_topic3")

baudrate1 = rospy.get_param("RoverSerial/baudrates/baudrate1")
baudrate2 = rospy.get_param("RoverSerial/baudrates/baudrate2")
baudrate3 = rospy.get_param("RoverSerial/baudrates/baudrate3")

# get data from pc
def serialCallback(data):
    global serialMsg
    serialMsg = data.data


def serialCallback2(data):
    global serialMsg2
    serialMsg2 = data.data


def serialCallback3(data):
    global serialMsg3
    serialMsg3 = data.data


def letsSerial():
    rospy.init_node("serial_rover")

    global namespace
    global serialMsg
    global serialMsg2
    global serialMsg3
    global sub_topic1
    global sub_topic2
    global sub_topic3
    global pub_topic1
    global pub_topic2
    global pub_topic3
    global baudrate1
    global baudrate2
    global baudrate3
    global serialString2
    global serialString3
    global serialString1

    deneme = rospy.get_param("RoverSerial/baudrates/baudrate1")
    rospy.loginfo(str(deneme))

    rospy.Subscriber(sub_topic1, String, serialCallback)
    rospy.Subscriber(sub_topic2, String, serialCallback2)
    rospy.Subscriber(sub_topic3, String, serialCallback3)

    sensor_pub = rospy.Publisher(pub_topic1, String, queue_size=10)
    sensor_pub2 = rospy.Publisher(pub_topic2, String, queue_size=10)
    sensor_pub3 = rospy.Publisher(pub_topic3, String, queue_size=10)

    printOnce = True

    while not rospy.is_shutdown():
        print("")
        print("")
        print(namespace + "Serial port 1: " + serialString1)
        print(namespace + "Serial port 2: " + serialString2)
        print(namespace + "Serial port 3: " + serialString3)
        print("")
        print("")
        print("P : Configure ports")
        print("T : Configure topics")
        print("B : Configure Baudrates")
        print("S : Show Defaults")
        print("Enter : Continue")
        serConfig = raw_input()

        if serConfig == "p" or serConfig == "P":
            print(namespace + "Enter nothing if serial port is not connected.")
            serialString1 = "/dev/ttyUSB" + raw_input("Enter first serial port: ")
            serialString2 = "/dev/ttyUSB" + raw_input("Enter second serial port: ")
            serialString3 = "/dev/ttyUSB" + raw_input("Enter third serial port: ")

        elif serConfig == "T" or serConfig == "t":
            sub_topic1 = raw_input("1. Sub Topic : ")
            sub_topic2 = raw_input("2. Sub Topic : ")
            sub_topic3 = raw_input("3. Sub Topic : ")

            pub_topic1 = raw_input("1. Pub Topic : ")
            pub_topic2 = raw_input("2. Pub Topic : ")
            pub_topic3 = raw_input("3. Sub Topic : ")

        elif serConfig == "B" or serConfig == "b":
            baudrate1 = raw_input("1. Baudrate : ")
            baudrate2 = raw_input("2. Baudrate : ")
            baudrate3 = raw_input("3. Baudrate : ")

        elif serConfig == "S" or serConfig == "s":
            debug = rospy.get_param("RoverSerial/baudrates/baudrate1")
            print(debug)
            print(namespace + "Serial port 1: " + serialString1)
            print(namespace + "Serial port 2: " + serialString2)
            print(namespace + "Serial port 3: " + serialString3)
            print(namespace + "Baudrate 1: " + baudrate1)
            print(namespace + "Baudrate 2: " + baudrate2)
            print(namespace + "Baudrate 3: " + baudrate3)
            print(namespace + "Sub Topic 1: " + sub_topic1)
            print(namespace + "Sub Topic 2: " + sub_topic2)
            print(namespace + "Sub Topic 3: " + sub_topic3)
            print(namespace + "Pub Topic 1: " + pub_topic1)
            print(namespace + "Pub Topic 2: " + pub_topic2)
            print(namespace + "Pub Topic 3: " + pub_topic3)

        else:
            # All ports are open
            if (
                serialString1 != "/dev/ttyUSB"
                and serialString2 != "/dev/ttyUSB"
                and serialString3 != "/dev/ttyUSB"
            ):

                ser = serial.Serial(
                    port=serialString1,
                    baudrate=int(baudrate1),
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
                ser3 = serial.Serial(
                    port=serialString3,
                    baudrate=int(baudrate3),
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                )  # open serial
                ser.timeout = 1
                ser2.timeout = 1
                ser3.timeout = 1

                while (
                    ser.isOpen()
                    and ser2.isOpen()
                    and ser3.isOpen()
                    and not rospy.is_shutdown()
                ):

                    if printOnce == True:
                        print(namespace + "serial1 is open")
                        print(namespace + "serial2 is open")
                        print(namespace + "serial3 is open")
                        printOnce = False

                    receive = ser.readline()
                    receive2 = ser2.readline()
                    receive3 = ser3.readline()

                    ser.writelines(serialMsg + "\n")
                    ser2.writelines(serialMsg2 + "\n")
                    ser3.writelines(serialMsg3 + "\n")

                    sensor_pub.publish(receive)
                    sensor_pub2.publish(receive2)
                    sensor_pub3.publish(receive3)

                    ser.flushInput()
                    ser.flushOutput()
                    ser2.flushInput()
                    ser2.flushOutput()
                    ser3.flushInput()
                    ser3.flushOutput()

                    print(
                        namespace
                        + "Reading from 1 : "
                        + str(receive)
                        + " Writing to 1 : "
                        + str(serialMsg)
                    )
                    print(
                        namespace
                        + "Reading from 2 : "
                        + str(receive2)
                        + "Writing to 2 : "
                        + str(serialMsg2)
                    )
                    print(
                        namespace
                        + "Reading from 3 : "
                        + str(receive)
                        + " Writing to 3 : "
                        + str(serialMsg3)
                    )

            # 2 ports open.
            elif (
                serialString1 != "/dev/ttyUSB"
                and serialString2 != "/dev/ttyUSB"
                and serialString3 == "/dev/ttyUSB"
            ):

                ser = serial.Serial(
                    port=serialString1,
                    baudrate=int(baudrate1),
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

                while ser.isOpen() and ser2.isOpen() and not rospy.is_shutdown():

                    if printOnce == True:
                        print(namespace + "serial1 is open")
                        print(namespace + "serial2 is open")
                        printOnce = False

                    receive = ser.readline()
                    receive2 = ser2.readline()

                    ser.writelines(serialMsg + "\n")
                    ser2.writelines(serialMsg2 + "\n")

                    sensor_pub.publish(receive)
                    sensor_pub2.publish(receive2)

                    ser.flushInput()
                    ser.flushOutput()
                    ser2.flushInput()
                    ser2.flushOutput()

                    print(
                        namespace
                        + "Reading from 1 : "
                        + str(receive)
                        + " Writing to 1 : "
                        + str(serialMsg)
                    )
                    print(
                        namespace
                        + "Reading from 2 : "
                        + str(receive2)
                        + " Writing to 2 : "
                        + str(serialMsg2)
                    )

            # 1 port open.
            elif (
                serialString1 != "/dev/ttyUSB"
                and serialString2 == "/dev/ttyUSB"
                and serialString3 == "/dev/ttyUSB"
            ):

                ser = serial.Serial(
                    port=serialString1,
                    baudrate=int(baudrate1),
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                )  # open serial
                ser.timeout = 1

                while ser.isOpen() and not rospy.is_shutdown():

                    if printOnce == True:
                        print(namespace + "serial1 is open")
                        printOnce = False

                    receive = ser.readline()
                    ser.writelines(serialMsg + "\n")

                    # There is no data that goes to the rover from pc. So there is no other writelines.
                    sensor_pub.publish(receive)

                    ser.flushInput()
                    ser.flushOutput()

                    print(
                        namespace
                        + "Reading from 1: "
                        + str(receive)
                        + "Writing this to 1: "
                        + str(serialMsg)
                    )
            # No open ports.
            else:
                print(namespace + "WTF Then ??")

    rospy.spin()


if __name__ == "__main__":
    try:
        letsSerial()
    except rospy.ROSInterruptException:
        pass
