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

void joystick_callback(const sensor_msgs::Joy::ConstPtr& msg) {
  new_message = true;
  // cout << msg->axes[0] << endl;
  joy_msg.set_axes(msg->axes[0], msg->axes[1], msg->axes[2], msg->axes[3], msg->axes[4], msg->axes[5]);
  joy_msg.set_buttons(msg->buttons[0], msg->buttons[1], msg->buttons[2], msg->buttons[3], msg->buttons[4],
                      msg->buttons[5], msg->buttons[6], msg->buttons[7], msg->buttons[8], msg->buttons[9],
                      msg->buttons[10], msg->buttons[11]);
}

int main(int argc, char** argv) {
  vector<coordinate> my_joints;

  my_joints.push_back(coordinate(0.0, 0.0, 14.0));
  my_joints.push_back(coordinate(0.0, 0.0, 64.0));
  my_joints.push_back(coordinate(42.0, 0.0, 53.0));
  my_joints.push_back(coordinate(69.0, 0.0, 53.0));

  int i;
  double r, lambda;

  double link_lengths[3];

  link_lengths[0] = 50.0;     // link_2_3_length
  link_lengths[1] = 43.4165;  // link_3_5_length
  link_lengths[2] = 27.0;     // link_5_end_length

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

  double joint_angles[6];

  joint_angles[0] = 0.0;
  joint_angles[1] = 0.0;
  joint_angles[2] = 0.0;
  joint_angles[3] = 0.0;
  joint_angles[4] = 0.0;
  joint_angles[5] = 0.0;

  while (ros::ok()) {
    ros::spinOnce();

    joint_angles[0] += joy_msg.get_axis(0) * 0.06;  // Calculated angles minus zero pos angles
    joint_angles[1] += joy_msg.get_axis(1) * 0.06;
    joint_angles[2] += joy_msg.get_axis(3) * 0.06;

    if (joy_msg.get_button(0)) joint_angles[3] += joy_msg.get_button(0) * 0.06;
    if (joy_msg.get_button(1)) joint_angles[3] -= joy_msg.get_button(1) * 0.06;

    if (joy_msg.get_button(2)) joint_angles[4] += joy_msg.get_button(2) * 0.06;
    if (joy_msg.get_button(3)) joint_angles[4] -= joy_msg.get_button(3) * 0.06;

    if (joy_msg.get_button(7)) joint_angles[5] += joy_msg.get_button(7) * 0.06;
    if (joy_msg.get_button(6)) joint_angles[5] -= joy_msg.get_button(6) * 0.06;

    msg_ss << joint_angles[0] << "  " << joint_angles[1] << "  " << joint_angles[2] << "  " << joint_angles[3] << "  "
           << joint_angles[4] << endl;
    angles.data = msg_ss.str();

    cout << "Joint angles:  " << joint_angles[0] << "  " << joint_angles[1] << "  " << joint_angles[2] << "  "
         << joint_angles[3] << "  " << joint_angles[4] << endl;

    joint1_last.data = (joint_angles[0] * PI) / 180;
    joint2_last.data = (joint_angles[1] * PI) / 180;
    joint3_last.data = (joint_angles[2] * PI) / 180;
    joint4_last.data = (joint_angles[3] * PI) / 180;
    joint5_last.data = (joint_angles[4] * PI) / 180;
    joint6_last.data = (joint_angles[5] * PI) / 180;

    chatter_pub.publish(angles);
    joint1_pub.publish(joint1_last);
    joint2_pub.publish(joint2_last);
    joint3_pub.publish(joint3_last);
    joint4_pub.publish(joint4_last);
    joint5_pub.publish(joint5_last);
    joint6_pub.publish(joint6_last);

    loop_rate.sleep();
  }

  return 0;
}
