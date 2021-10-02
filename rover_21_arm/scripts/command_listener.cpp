#include <iostream>
#include <sstream>
#include <string>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

using namespace std;

void joystick_callback(const std_msgs::Float64::ConstPtr& msg) { cout << msg->data << endl; }

int main(int argc, char** argv) {
  ros::init(argc, argv, "joints_pos_publisher");
  ros::NodeHandle n;

  std_msgs::Float64 j1_angle;
  ros::Subscriber j1_sub = n.subscribe("/rover_arm_j1_joint_position_controller/command", 1, joystick_callback);

  ros::Rate loop_rate(100);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
