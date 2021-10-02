// Algoritmanın çalışma mantığı için: https://drive.google.com/drive/folders/10Jx3R2WmaSRwj1mYTVzVz-0sIhBbL4JL

#include <sensor_msgs/Joy.h>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "definitions.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

using namespace std;

bool new_message = false;
joy_message joy_msg;

double REACH_1 = 53.8;

void joystick_callback(const sensor_msgs::Joy::ConstPtr& msg) {
  new_message = true;
  // cout << msg->axes[0] << endl;
  joy_msg.set_axes(msg->axes[0], msg->axes[1], msg->axes[2], msg->axes[3], msg->axes[4], msg->axes[5]);
  joy_msg.set_buttons(msg->buttons[0], msg->buttons[1], msg->buttons[2], msg->buttons[3], msg->buttons[4],
                      msg->buttons[5], msg->buttons[6], msg->buttons[7], msg->buttons[8], msg->buttons[9],
                      msg->buttons[10], msg->buttons[11]);
}

int main(int argc, char** argv) {
  vector<coordinate> my_joints_1;

  my_joints_1.push_back(coordinate(0.0, 0.0, 7.0));
  my_joints_1.push_back(coordinate(0.0, 0.0, 33.5));
  my_joints_1.push_back(coordinate(27.3, 0.0, 33.5));

  int i;
  double r, lambda;

  double link_lengths_1[2];

  link_lengths_1[0] = 26.5;
  link_lengths_1[1] = 27.3;

  double joint_angles[6];

  joint_angles[0] = 0.0;
  joint_angles[1] = 0.0;
  joint_angles[2] = 0.0;
  joint_angles[3] = 0.0;
  joint_angles[4] = 0.0;
  joint_angles[5] = 0.0;

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::NodeHandle joint1_handle;
  ros::NodeHandle joint2_handle;
  ros::NodeHandle joint3_handle;
  ros::NodeHandle joint4_handle;
  ros::NodeHandle joint5_handle;
  ros::NodeHandle joint6_handle;
  ros::NodeHandle joy_handle;

  std_msgs::String angles;
  std_msgs::Float64 joint1_last;
  std_msgs::Float64 joint2_last;
  std_msgs::Float64 joint3_last;
  std_msgs::Float64 joint4_last;
  std_msgs::Float64 joint5_last;
  std_msgs::Float64 joint6_last;

  stringstream msg_ss;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher joint1_pub =
      joint1_handle.advertise<std_msgs::Float64>("/rover_arm_j1_joint_position_controller/command", 1000);
  ros::Publisher joint2_pub =
      joint2_handle.advertise<std_msgs::Float64>("/rover_arm_j2_joint_position_controller/command", 1000);
  ros::Publisher joint3_pub =
      joint3_handle.advertise<std_msgs::Float64>("/rover_arm_j3_joint_position_controller/command", 1000);
  ros::Publisher joint4_pub =
      joint4_handle.advertise<std_msgs::Float64>("/rover_arm_j4_joint_position_controller/command", 1000);
  ros::Publisher joint5_pub =
      joint5_handle.advertise<std_msgs::Float64>("/rover_arm_j5_joint_position_controller/command", 1000);
  ros::Publisher joint6_pub =
      joint6_handle.advertise<std_msgs::Float64>("/rover_arm_j6_joint_position_controller/command", 1000);
  ros::Subscriber joy_sub = joy_handle.subscribe("/joy", 1, joystick_callback);
  ros::Rate loop_rate(100);

  int count = 0;

  bool first_run = true;
  bool second_part = false;
  bool first_stage = false;
  bool part_switch = false;

  double X1;
  double Y1;
  double Z1;
  double joint_4_part;
  double joint_5_part;
  double joint_6_part;

  double joint_1_angle = 0.0;
  double joint_2_angle = 0.0;
  double joint_3_angle = 0.0;
  double joint_4_angle = 0.0;
  double joint_5_angle = 0.0;

  while (ros::ok()) {
    ros::spinOnce();

    if (first_run) {
      X1 = 0.0;
      Y1 = 0.0;
      Z1 = 0.0;
      joint_4_part = 0.0;
      joint_5_part = 0.0;
      joint_6_part = 0.0;

      joint_1_angle = 0.0;

      first_run = false;
    }

    if ((joy_msg.get_button(7) == 1) && (!first_stage))
      first_stage = true;  // A switch mechanism for switching between parts

    if ((first_stage) && (joy_msg.get_button(7) == 0)) {
      if (second_part)
        second_part = false;
      else
        second_part = true;

      first_stage = false;
    }

    if (!second_part) {
      cout << "FIRST PART" << endl;

      X1 += joy_msg.get_axis(3) * 0.04;
      Y1 += joy_msg.get_axis(0) * 0.04;
      Z1 += joy_msg.get_axis(1) * 0.04;

      coordinate new_end_point = coordinate(27.3 + X1, 0.0 + Y1, 33.5 + Z1);

      cout << endl
           << "Target: X = " << new_end_point.get_x() << "  Y = " << new_end_point.get_y()
           << "  Z = " << new_end_point.get_z() << endl
           << endl;

      double joint_1_angle_dif = (-1.0) * joint_1_angle;

      coordinate reference_coord = coordinate(4, 0, 0);
      coordinate proj_joint_1 = coordinate(new_end_point.get_x(), new_end_point.get_y(), 0);
      joint_1_angle =
          angle_of_vectors(proj_joint_1, reference_coord);  // Projection of endpoint on xy and reference cord
      if (proj_joint_1.get_y() < 0) joint_1_angle = joint_1_angle * (-1);

      joint_1_angle_dif += joint_1_angle;

      rotate_on_xy(my_joints_1[0], joint_1_angle_dif);
      rotate_on_xy(my_joints_1[1], joint_1_angle_dif);
      rotate_on_xy(my_joints_1[2], joint_1_angle_dif);

      FABRIK_algorithm(my_joints_1, link_lengths_1, new_end_point, REACH_1);

      joint_2_angle = find_angle(my_joints_1[0], coordinate(0.0, 0.0, 0.0), my_joints_1[1], joint_1_angle);
      joint_3_angle = find_angle(my_joints_1[1], my_joints_1[0], my_joints_1[2], joint_1_angle);

      joint_angles[0] = joint_1_angle;  // Calculated angles minus zero pos angles
      joint_angles[1] = 180.0 - joint_2_angle;
      joint_angles[2] = 90.0 - joint_3_angle;

      joint1_last.data = (joint_angles[0] * PI) / 180;
      joint2_last.data = (joint_angles[1] * PI) / 180;
      joint3_last.data = (joint_angles[2] * PI) / 180;
      joint4_last.data = joint_angles[3];
      joint5_last.data = joint_angles[4];
      joint6_last.data = joint_angles[5];

      msg_ss << joint_angles[0] << "  " << joint_angles[1] << "  " << joint_angles[2] << "  " << joint_angles[3] << "  "
             << joint_angles[4] << "  " << joint_angles[5] << endl;
      angles.data = msg_ss.str();

      cout << "Joint angles:  " << joint_angles[0] << "  " << joint_angles[1] << "  " << joint_angles[2] << "  "
           << joint_angles[3] << "  " << joint_angles[4] << "  " << joint_angles[5] << endl;

      chatter_pub.publish(angles);

      if (check_angles(4, joint_angles)) {
        joint1_pub.publish(joint1_last);
        joint2_pub.publish(joint2_last);
        joint3_pub.publish(joint3_last);
        joint4_pub.publish(joint4_last);
        joint5_pub.publish(joint5_last);
        joint6_pub.publish(joint6_last);
      }
    }

    else {
      cout << "SECOND PART" << endl;

      joint_4_part += joy_msg.get_axis(0) * 0.0009;
      joint_5_part += (-1) * joy_msg.get_axis(1) * 0.0009;
      joint_6_part += (-1) * joy_msg.get_axis(2) * 0.0009;

      // cout << joint_6_part << endl;

      joint_angles[3] = joint_4_part;
      joint_angles[4] = joint_5_part;
      joint_angles[5] = joint_6_part;

      joint1_last.data = (joint_angles[0] * PI) / 180;
      joint2_last.data = (joint_angles[1] * PI) / 180;
      joint3_last.data = (joint_angles[2] * PI) / 180;
      joint4_last.data = joint_angles[3];
      joint5_last.data = joint_angles[4];
      joint6_last.data = joint_angles[5];

      // cout << joint6_last.data << endl;

      msg_ss << joint_angles[0] << "  " << joint_angles[1] << "  " << joint_angles[2] << "  " << joint_angles[3] << "  "
             << joint_angles[4] << "  " << joint_angles[5] << endl;
      angles.data = msg_ss.str();

      cout << "Joint angles:  " << joint_angles[0] << "  " << joint_angles[1] << "  " << joint_angles[2] << "  "
           << joint_angles[3] << "  " << joint_angles[4] << "  " << joint_angles[5] << endl;

      chatter_pub.publish(angles);

      if (check_angles(4, joint_angles)) {
        joint1_pub.publish(joint1_last);
        joint2_pub.publish(joint2_last);
        joint3_pub.publish(joint3_last);
        joint4_pub.publish(joint4_last);
        joint5_pub.publish(joint5_last);
        joint6_pub.publish(joint6_last);
      }
    }

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
