import rospy
import rosparam
from std_msgs.msg import String
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseStamped, Twist
from math import radians, cos, sin, asin, sqrt, pow, pi, atan2
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf


class Approacher:
    def __init__(self):
        self.bearing = ""
        self.bearing_topic = rospy.get_param()
        self.start()

    def start(self):
        while not rospy.is_shutdown():
            rospy.Subscriber(self.bearing_topic, String, self.bearing_callback)
            if self.bearing != "-":
                self.rotate()
                self.go_forward()

    def bearing_callback(self, msg):
        self.bearing = msg.data

    def rotate(self):
        print("Rotating...")
        # print(angle)
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = "/base_link"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 0
        goal.target_pose.pose.position.y = 0
        goal.target_pose.pose.position.z = 0

        q = tf.transformations.quaternion_from_euler(
            0, 0, (float(self.bearing) * pi / 180)
        )
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()

    def go_forward(self):
        goal = MoveBaseGoal()
        speed = 1
        goal.target_pose.header.frame_id = "/base_link"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = speed
        goal.target_pose.pose.position.y = 0
        goal.target_pose.pose.position.z = 0

        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]


if __name__ == "__main__":
    rospy.init_node("approaching")
    try:
        Approacher()
    except rospy.ROSInterruptException:
        pass
