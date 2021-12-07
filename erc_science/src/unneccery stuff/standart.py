#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



bridge = CvBridge()

def image_turn(ros_image):
    #print('Image taken!')
    global bridge,socomponents_locations,swcomponents_locations,sacomponents_locations,dicomponents_locations

    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)

    #canny = cv2.Canny(cv_image,100,200)
    cv2.imshow('Camera',cv_image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown('exit...')

def main(args):
    rospy.init_node('Rs_to_CV', anonymous=True)
    rospy.Subscriber('/usb_cam/image_raw',Image,image_turn)


    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Closed')
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
