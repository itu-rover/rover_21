import math
import rospy
from std_msgs.msg import Float64

L1 = 7.0
L2 = 26.5
L3 = 27.3
L4 = 7.0
L5 = 20.0

j1_angle = 0.0
j2_angle = 0.0
j3_angle = 0.0
j4_angle = 0.0
j5_angle = 0.0


class coordinate:
    def __init__(self, new_x, new_y, new_z):

        self.x = new_x
        self.y = new_y
        self.z = new_z


def Dof_6(t0, t1, t2, t3, t4):

    px = (
        L4
        * (
            math.cos(t0) * math.cos(t1) * math.sin(t2)
            + math.cos(t0) * math.cos(t2) * math.sin(t1)
        )
        - L5
        * (
            math.cos(t4)
            * (
                math.sin(t0) * math.sin(t3)
                + math.cos(t3)
                * (
                    math.cos(t0) * math.sin(t1) * math.sin(t2)
                    - math.cos(t0) * math.cos(t1) * math.cos(t2)
                )
            )
            - math.sin(t4)
            * (
                math.cos(t0) * math.cos(t1) * math.sin(t2)
                + math.cos(t0) * math.cos(t2) * math.sin(t1)
            )
        )
        - L2 * math.cos(t0) * math.sin(t1)
        + L3 * math.cos(t0) * math.cos(t1) * math.cos(t2)
        - L3 * math.cos(t0) * math.sin(t1) * math.sin(t2)
    )
    py = (
        L5
        * (
            math.cos(t4)
            * (
                math.cos(t0) * math.sin(t3)
                - math.cos(t3)
                * (
                    math.sin(t0) * math.sin(t1) * math.sin(t2)
                    - math.cos(t1) * math.cos(t2) * math.sin(t0)
                )
            )
            + math.sin(t4)
            * (
                math.cos(t1) * math.sin(t0) * math.sin(t2)
                + math.cos(t2) * math.sin(t0) * math.sin(t1)
            )
        )
        + L4
        * (
            math.cos(t1) * math.sin(t0) * math.sin(t2)
            + math.cos(t2) * math.sin(t0) * math.sin(t1)
        )
        - L2 * math.sin(t0) * math.sin(t1)
        + L3 * math.cos(t1) * math.cos(t2) * math.sin(t0)
        - L3 * math.sin(t0) * math.sin(t1) * math.sin(t2)
    )
    pz = (
        L1
        - L4 * (math.cos(t1) * math.cos(t2) - math.sin(t1) * math.sin(t2))
        + L2 * math.cos(t1)
        - L5
        * (
            math.sin(t4) * (math.cos(t1) * math.cos(t2) - math.sin(t1) * math.sin(t2))
            - math.cos(t3)
            * math.cos(t4)
            * (math.cos(t1) * math.sin(t2) + math.cos(t2) * math.sin(t1))
        )
        + L3 * math.cos(t1) * math.sin(t2)
        + L3 * math.cos(t2) * math.sin(t1)
    )

    return coordinate(px, py, pz)


def Dof_5(t0, t1, t2):

    px = (
        L4
        * (
            math.cos(t0) * math.cos(t1) * math.sin(t2)
            + math.cos(t0) * math.cos(t2) * math.sin(t1)
        )
        - L2 * math.cos(t0) * math.sin(t1)
        + L3 * math.cos(t0) * math.cos(t1) * math.cos(t2)
        - L3 * math.cos(t0) * math.sin(t1) * math.sin(t2)
    )
    py = (
        L4
        * (
            math.cos(t1) * math.sin(t0) * math.sin(t2)
            + math.cos(t2) * math.sin(t0) * math.sin(t1)
        )
        - L2 * math.sin(t0) * math.sin(t1)
        + L3 * math.cos(t1) * math.cos(t2) * math.sin(t0)
        - L3 * math.sin(t0) * math.sin(t1) * math.sin(t2)
    )
    pz = (
        L1
        - L4 * (math.cos(t1) * math.cos(t2) - math.sin(t1) * math.sin(t2))
        + L2 * math.cos(t1)
        + L3 * math.cos(t1) * math.sin(t2)
        + L3 * math.cos(t2) * math.sin(t1)
    )

    return coordinate(px, py, pz)


def Dof_4(t0, t1, t2):

    px = (
        L4
        * (
            math.cos(t0) * math.cos(t1) * math.sin(t2)
            + math.cos(t0) * math.cos(t2) * math.sin(t1)
        )
        - L2 * math.cos(t0) * math.sin(t1)
        + L3 * math.cos(t0) * math.cos(t1) * math.cos(t2)
        - L3 * math.cos(t0) * math.sin(t1) * math.sin(t2)
    )
    py = (
        L4
        * (
            math.cos(t1) * math.sin(t0) * math.sin(t2)
            + math.cos(t2) * math.sin(t0) * math.sin(t1)
        )
        - L2 * math.sin(t0) * math.sin(t1)
        + L3 * math.cos(t1) * math.cos(t2) * math.sin(t0)
        - L3 * math.sin(t0) * math.sin(t1) * math.sin(t2)
    )
    pz = (
        L1
        - L4 * (math.cos(t1) * math.cos(t2) - math.sin(t1) * math.sin(t2))
        + L2 * math.cos(t1)
        + L3 * math.cos(t1) * math.sin(t2)
        + L3 * math.cos(t2) * math.sin(t1)
    )

    return coordinate(px, py, pz)


def Dof_3(t0, t1, t2):

    px = (
        L3 * math.cos(t0) * math.cos(t1) * math.cos(t2)
        - L2 * math.cos(t0) * math.sin(t1)
        - L3 * math.cos(t0) * math.sin(t1) * math.sin(t2)
    )
    py = (
        L3 * math.cos(t1) * math.cos(t2) * math.sin(t0)
        - L2 * math.sin(t0) * math.sin(t1)
        - L3 * math.sin(t0) * math.sin(t1) * math.sin(t2)
    )
    pz = (
        L1
        + L2 * math.cos(t1)
        + L3 * math.cos(t1) * math.sin(t2)
        + L3 * math.cos(t2) * math.sin(t1)
    )

    return coordinate(px, py, pz)


def Dof_2(t0, t1):

    px = -L2 * math.cos(t0) * math.sin(t1)
    py = -L2 * math.sin(t0) * math.sin(t1)
    pz = L1 + L2 * math.cos(t1)

    return coordinate(px, py, pz)


def Dof_1(t0):

    px = 0
    py = 0
    pz = L1

    return coordinate(px, py, pz)


def callback_j1(data):

    j1_angle = data.data


def callback_j2(data):

    j2_angle = data.data


def callback_j3(data):

    j3_angle = data.data


def callback_j4(data):

    j4_angle = data.data


def callback_j5(data):

    j5_angle = data.data


rospy.init_node("joint_pos_pub", anonymous=True)

rospy.Subscriber(
    "/rover_arm_j1_joint_position_controller/command", Float64, callback_j1
)
rospy.Subscriber(
    "/rover_arm_j2_joint_position_controller/command", Float64, callback_j2
)
rospy.Subscriber(
    "/rover_arm_j3_joint_position_controller/command", Float64, callback_j3
)
rospy.Subscriber(
    "/rover_arm_j4_joint_position_controller/command", Float64, callback_j4
)
rospy.Subscriber(
    "/rover_arm_j5_joint_position_controller/command", Float64, callback_j5
)


j1_pos = Dof_1(j1_angle)
j2_pos = Dof_2(j1_angle, j2_angle)
j3_pos = Dof_3(j1_angle, j2_angle, j3_angle)
j4_pos = Dof_4(j1_angle, j2_angle, j3_angle)
j5_pos = Dof_5(j1_angle, j2_angle, j3_angle)
ee_pos = Dof_6(j1_angle, j2_angle, j3_angle, j4_angle, j5_angle)

# print(j1_angle)
# print(j2_angle)
# print(j3_angle)
# print(j4_angle)
# print(j5_angle)

print(j1_pos.x, "  ", j1_pos.y, "  ", j1_pos.z, "  ")
print(j2_pos.x, "  ", j2_pos.y, "  ", j2_pos.z, "  ")
print(j3_pos.x, "  ", j3_pos.y, "  ", j3_pos.z, "  ")
print(j4_pos.x, "  ", j4_pos.y, "  ", j4_pos.z, "  ")
print(j5_pos.x, "  ", j5_pos.y, "  ", j5_pos.z, "  ")
print(ee_pos.x, "  ", ee_pos.y, "  ", ee_pos.z, "  ")
