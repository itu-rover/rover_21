#!/usr/bin/env python
# This is the bearing calculator code for ITU ROVER TEAM
import rospy
from std_msgs.msg import String
import rosparam


pxCoordinateTopic = rospy.get_param(
    "RoverReachImage/CalculateBearing/sub_pxCoordinates", "/px_coordinates"
)
pxCoordinateTopic1 = rospy.get_param(
    "RoverReachImage/calculatedBearing/sub_pxCoordinates1", "/px_coordinates1"
)
bearingPub = rospy.get_param(
    "RoverReachImage/CalculateBearing/pub_bearing", "/bearing_to_ball"
)
camAngleOfView = rospy.get_param("RoverReachImage/CalculateBearing/camAngleOfView", 110)

pxCoordinates = [None] * 2
pxCoordinates1 = [None] * 2
videoWidth = None
videoHeight = None
calculatedBearing = None
noArtag = True


def pxCallback(data):
    global pxCoordinates
    global videoHeight
    global videoWidth
    global noArtag
    if data.data != "-":
        pxCoordinates[0] = float(data.data.split(",")[0])
        pxCoordinates[1] = float(data.data.split(",")[1])
        videoWidth = float(data.data.split(",")[2])
        videoHeight = float(data.data.split(",")[3])
        noArtag = False
    else:
        noArtag = True


def pxCallback1(data):
    global pxCoordinates1
    global videoHeight
    global videoWidth
    global noArtag

    if data.data != "-":
        pxCoordinates1[0] = float(data.data.split(",")[0])
        pxCoordinates1[1] = float(data.data.split(",")[1])
        videoWidth = float(data.data.split(",")[2])
        videoHeight = float(data.data.split(",")[3])
        noArtag = False
    else:
        noArtag = True


def scCallback(data):
    sc = data.data


def calculateBearing(pxWidth, pxHeight, pxWidth1, pxHeight1, videoWidth, videoHeight):
    global calculatedBearing
    global noArtag

    sc_sub = rospy.Subscriber("/stage_counter_topic", String, scCallback)

    if sc_sub == 3:
        if pxWidth != None and videoWidth != None and noBall == False:
            center = videoWidth / 2
            anglePerPixel = camAngleOfView / videoWidth
            diff = center - pxWidth
            if diff > 5:
                print("+" + str(abs(diff * anglePerPixel)))
                calculatedBearing = "+" + str(abs(diff * anglePerPixel))

            elif diff < -5:
                print("-" + str(abs(diff * anglePerPixel)))
                calculatedBearing = "-" + str(abs(diff * anglePerPixel))

            else:
                print("Duz")
                calculatedBearing = "0"

        else:
            calculatedBearing = "-"

    if sc_sub >= 4:
        if (
            pxWidth != None
            and videoWidth != None
            and pxWidth1 != None
            and noArtag == False
        ):
            center = videoWidth / 2
            anglePerPixel = camAngleOfView / videoWidth
            middle_point = (pxWidth + pxWidth1) / 2
            diff = center - middle_point

            if diff > 5:
                print("+" + str(abs(diff * anglePerPixel)))
                calculatedBearing = "+" + str(abs(diff * anglePerPixel))

            elif diff < -5:
                print("-" + str(abs(diff * anglePerPixel)))
                calculatedBearing = "-" + str(abs(diff * anglePerPixel))

            else:
                print("Duz")
                calculatedBearing = "0"

        else:
            calculatedBearing = "-"

        rospy.sleep(0.04)


def printIt(pxWidth, pxHeight, videoWidth, videoHeight):
    print(
        "pxWidth : "
        + str(pxWidth)
        + " pxHeight : "
        + str(pxHeight)
        + "pxWidth1 : "
        + str(pxWidth1)
        + " pxHeight1 : "
        + str(pxHeight1)
        + " videoWidth : "
        + str(videoWidth)
        + " videoHeight : "
        + str(videoHeight)
    )


def main():
    global pxCoordinates
    global pxCoordinates1
    global videoWidth
    global videoHeight
    global calculatedBearing

    while not rospy.is_shutdown():
        calculateBearing(
            pxCoordinates[0],
            pxCoordinates[1],
            pxCoordinates1[0],
            pxCoordinates1[1],
            videoWidth,
            videoHeight,
        )
        if calculatedBearing != None:
            bearingPublisher.publish(calculatedBearing)

    rospy.spin()


if __name__ == "__main__":

    try:
        rospy.init_node("bearing")
        bearingPublisher = rospy.Publisher(bearingPub, String, queue_size=100)
        rospy.Subscriber(pxCoordinateTopic, String, pxCallback)
        rospy.Subscriber(pxCoordinateTopic, String, pxCallback1)
        while not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
