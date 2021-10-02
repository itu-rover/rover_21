#!/usr/bin/env python

# This code provides to control servo - camera
import rospy
import serial
import time
import io
import rosparam
from std_msgs.msg import String


class Servocamera:
    def __init__(self):
        self.f_letter = "s"  # beginnig of frame
        self.l_letter = "f"  # ending of frame
        # self.rotate = 0
        self.angle = 0  # Onat'in aci degerlerini yazdirmasiyla duzenlencek.
        self.see_artag = 0
        self.control = " "
        # self.first = True
        self.servo_rotating = False
        self.serialMsg = " "
        self.start()

    def start(self):

        self.port = rospy.get_param("ServoCamera/ports/servoport", "/dev/ttyUSB0")
        self.angle_topic = rospy.get_param(
            "ServoCamera/pub_topic_scam/pub_servo_angle", "/servo_angle"
        )
        self.scontrol_topic = rospy.get_param(
            "ServoCamera/sub_topic_scam/sub_control", "/servo_control"
        )  # subscribe from artag_search
        self.baudrate = rospy.get_param("ServoCamera/Baudrate/baudrate", 9600)
        # self.ser = serial.Serial(self.port, self.baudrate, timeout=1) #bytesize i ogren, buna ekle!
        rate = rospy.Rate(30)

        self.openserial()
        self.start_servo()

        angle_pub = rospy.Publisher(self.angle_topic, String, queue_size=10)

        while not rospy.is_shutdown():
            serialMsg = " "
            rospy.Subscriber(self.scontrol_topic, String, self.controlSubscriber)
            print(self.servo_rotating)
            # print(self.serialMsg)
            # time.sleep(5)

            if self.serialMsg == "s10f" and self.servo_rotating is False:
                # print("s10f done")
                self.start_servo()
                print("started")
                self.servo_rotating = True

            if self.serialMsg == "s01f" and self.servo_rotating is True:
                # if self.first == True:
                self.stop_servo()
                print("stopped")
                self.servo_rotating = False
                # self.Reader()

            rospy.sleep(0.2)
        rospy.spin()

    def openserial(self):

        self.ser = serial.Serial(
            self.port, self.baudrate, timeout=1
        )  # bytesize i ogren, buna ekle!
        rospy.loginfo("port is opened.")

    def Reader(self):
        self.angle = (
            self.ser.readline()
        )  # Onat'in aci degerlerini yazdirmasiyla duzenlencek.
        self.ser.flushInput()
        rospy.loginfo(self.angle)
        # angle_pub.publish(self.angle) #Onat'in aci degerlerini yazdirmasiyla duzenlencek.

    def stop_servo(self):
        self.ser.writelines(self.f_letter + "0" + "1" + self.l_letter + "\n")
        self.ser.flushOutput()
        self.first = False

    def start_servo(self):
        self.ser.writelines(self.f_letter + "1" + "0" + self.l_letter + "\n")
        time.sleep(3)

        self.ser.flushOutput()

    def controlSubscriber(self, data):
        self.serialMsg = data.data


if __name__ == "__main__":

    rospy.init_node("serial_servocam")
    try:
        Servocamera()
        print("hello baby!")

    except rospy.ROSInterruptException:
        pass
