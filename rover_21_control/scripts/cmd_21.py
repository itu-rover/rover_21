#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Empty twist messages
twist_cmd = Twist()
twist_nav = Twist()
twist = Twist()

pub = rospy.Publisher("/rover_serial_topic", String, queue_size=10)

# Reads required info from data published from joystick
# data: Twist message
def callback_cmd(data):
    twist_cmd.linear.x = (
        data.linear.x * 120
    )  # Multiply by 120 to make it sensible for serial comm
    twist_cmd.angular.z = data.angular.z * 120


# Reads required info from data published from autonomous
# data: Twist message
def callback_nav(data):
    twist_nav.linear.x = data.linear.x * 120
    twist_nav.angular.z = data.angular.z * 120


def main():
    rospy.init_node("rover_20_cmd_sub_serial")
    rospy.Subscriber("/cmd_vel", Twist, callback_nav)  # Joystick
    rospy.Subscriber("/joy_teleop/cmd_vel", Twist, callback_cmd)  # Autonomous

    # Initialize messages
    left_wheelString = "000"
    right_wheelString = "000"

    way_left = 0  # 0 for forward, 1 for backward
    way_right = 0  # 0 for forward, 1 for backward
    all_wheels_msg = ""
    # Mesages below are not important since only one of the systems is using at any time
    robotic_arm_msg = "1000656515656552011665320002"  # Robotic arm serial message
    science_msg = "3212321526541653"  # Science system serial message

    left_wheel = 0  # Left wheel velocity
    right_wheel = 0  # Right wheel velocity

    rate = rospy.Rate(10)  # Work with 10Hz

    while not rospy.is_shutdown():

        # Check if any joystick velocity data is received
        if twist_cmd.linear.x != 0.0 or twist_cmd.angular.z != 0.0:
            twist = twist_cmd
        # Else, use autonomous velocity data
        else:
            twist = twist_nav

        if twist.linear.x >= 0:
            # Since positive direction of a rotation is counter-clockwise direction,
            # Wheel velocities should be calculated as given
            left_wheel = twist.linear.x - twist.angular.z
            right_wheel = twist.linear.x + twist.angular.z

            # While publishing Twist with angular velocities, linear velocity may go below
            # minimum motor power threshold. To overcome this, we assign 20 if they are below threshold.
            # If there is an angular velocity and absolute values of wheel velocities are equal,
            # we want to execute a tank rotation
            if twist.angular.z != 0 and abs(left_wheel) == abs(right_wheel):
                if abs(left_wheel) < 20:
                    left_wheel = 20
                if abs(right_wheel) < 20:
                    right_wheel = 20

            # If there is an angular velocity and absolute values of wheel velocities are different,
            # we want to perform rotation and translation
            if twist.angular.z != 0 and abs(left_wheel) != abs(right_wheel):
                if abs(right_wheel) > abs(left_wheel):
                    right_wheel = 80
                    left_wheel = 40
                if abs(right_wheel) < abs(left_wheel):
                    right_wheel = 40
                    left_wheel = 80

        elif twist.linear.x < 0:

            left_wheel = twist.linear.x + twist.angular.z
            right_wheel = twist.linear.x - twist.angular.z

            if twist.angular.z != 0:
                if abs(left_wheel) < 20:
                    left_wheel = 20
                if abs(right_wheel) < 20:
                    right_wheel = 20

        # Determine signs
        if left_wheel < 0:
            way_left = 1
            left_wheel *= -1
        else:
            way_left = 0

        if right_wheel < 0:
            way_right = 1
            right_wheel *= -1
        else:
            way_right = 0

        print("left: " + str(left_wheel) + " right: " + str(right_wheel))

        # Convert values to string messages
        if abs(left_wheel) < 10:
            left_wheelString = "00" + str(int(left_wheel))

        elif abs(left_wheel) < 100:
            left_wheelString = "0" + str(int(left_wheel))

        elif abs(left_wheel) > 100:
            left_wheelString = str(int(left_wheel))

        if abs(right_wheel) < 10:
            right_wheelString = "00" + str(int(right_wheel))

        elif abs(right_wheel) < 100:
            right_wheelString = "0" + str(int(right_wheel))

        elif abs(right_wheel) > 100:
            right_wheelString = str(int(right_wheel))

        # Check if velocities are exceeding upper threshold
        if left_wheel > 200:  # 160
            left_wheelString = "200"

        if right_wheel > 200:  # 160
            right_wheelString = "200"

        # Concatenate string messages and publish
        all_wheels_msg = (
            str(way_left)
            + str(left_wheelString)
            + str(way_right)
            + str(right_wheelString)
        )
        print("S" + all_wheels_msg + robotic_arm_msg + science_msg + "F")
        pub.publish("S" + all_wheels_msg + robotic_arm_msg + science_msg + "F")
        rate.sleep()


if __name__ == "__main__":
    main()
