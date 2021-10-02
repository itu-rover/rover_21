import rospy
from std_msgs.msg import Float64

my_float = 0.0


def callback(data):

    my_float = data.data

    print(my_float)


def listener():

    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber(
        "/rover_arm_j1_joint_position_controller/command", Float64, callback
    )

    rospy.spin()


if __name__ == "__main__":

    listener()
