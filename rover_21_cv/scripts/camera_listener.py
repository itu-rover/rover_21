#!/usr/bin/env python
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class image_listener:
    def __init__(self):
        self.bridge = CvBridge()
        topic_color = rospy.get_param("/camera/color_topic")
        topic_gray = rospy.get_param("/camera/gray_topic")
        self.sub = rospy.Subscriber(topic_color, Image, self.callback)
        self.image = None

        print("listener begins")
        self.main()

    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.image = cv_image

    def main(self):
        while not rospy.is_shutdown():
            if self.image is not None:
                cv2.imshow("Image window", self.image)
            cv2.waitKey(3)


def main(args):
    rospy.init_node("image_converter", anonymous=True)
    ic = image_listener()


if __name__ == "__main__":
    main(sys.argv)
