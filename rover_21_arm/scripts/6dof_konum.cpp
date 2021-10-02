#include <cmath>
#include <iostream>
#include <sstream>
#include <string>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

using namespace std;

double j1_angle = 0.0;
double j2_angle = -30.0;
double j3_angle = 0.0;
double j4_angle = 0.0;
double j5_angle = 0.0;

double L1 = 7.0;
double L2 = 26.5;
double L3 = 27.3;
double L4 = 7.0;
double L5 = 20.0;

string conv_to_string(double my_double) {
  ostringstream os;
  os << my_double;
  return os.str();
}

class coordinate {
 public:
  double x;
  double y;
  double z;
  coordinate(double, double, double);
};

coordinate::coordinate(double new_x, double new_y, double new_z) {
  x = new_x;
  y = new_y;
  z = new_z;
}

void j1_callback(const std_msgs::Float64::ConstPtr& msg) { j1_angle = (-1) * msg->data; }

void j2_callback(const std_msgs::Float64::ConstPtr& msg) { j2_angle = (-1) * msg->data; }

void j3_callback(const std_msgs::Float64::ConstPtr& msg) { j3_angle = (-1) * msg->data; }

void j4_callback(const std_msgs::Float64::ConstPtr& msg) { j4_angle = (-1) * msg->data; }

void j5_callback(const std_msgs::Float64::ConstPtr& msg) { j5_angle = (-1) * msg->data; }

coordinate Dof_6(double t0, double t1, double t2, double t3, double t4) {
  double px =
      L4 * (cos(t0) * cos(t1) * sin(t2) + cos(t0) * cos(t2) * sin(t1)) -
      L5 * (cos(t4) * (sin(t0) * sin(t3) + cos(t3) * (cos(t0) * sin(t1) * sin(t2) - cos(t0) * cos(t1) * cos(t2))) -
            sin(t4) * (cos(t0) * cos(t1) * sin(t2) + cos(t0) * cos(t2) * sin(t1))) -
      L2 * cos(t0) * sin(t1) + L3 * cos(t0) * cos(t1) * cos(t2) - L3 * cos(t0) * sin(t1) * sin(t2);
  double py =
      L5 * (cos(t4) * (cos(t0) * sin(t3) - cos(t3) * (sin(t0) * sin(t1) * sin(t2) - cos(t1) * cos(t2) * sin(t0))) +
            sin(t4) * (cos(t1) * sin(t0) * sin(t2) + cos(t2) * sin(t0) * sin(t1))) +
      L4 * (cos(t1) * sin(t0) * sin(t2) + cos(t2) * sin(t0) * sin(t1)) - L2 * sin(t0) * sin(t1) +
      L3 * cos(t1) * cos(t2) * sin(t0) - L3 * sin(t0) * sin(t1) * sin(t2);
  double pz = L1 - L4 * (cos(t1) * cos(t2) - sin(t1) * sin(t2)) + L2 * cos(t1) -
              L5 * (sin(t4) * (cos(t1) * cos(t2) - sin(t1) * sin(t2)) -
                    cos(t3) * cos(t4) * (cos(t1) * sin(t2) + cos(t2) * sin(t1))) +
              L3 * cos(t1) * sin(t2) + L3 * cos(t2) * sin(t1);

  return coordinate(px, py, pz);
}

coordinate Dof_5(double t0, double t1, double t2) {
  double px = L4 * (cos(t0) * cos(t1) * sin(t2) + cos(t0) * cos(t2) * sin(t1)) - L2 * cos(t0) * sin(t1) +
              L3 * cos(t0) * cos(t1) * cos(t2) - L3 * cos(t0) * sin(t1) * sin(t2);
  double py = L4 * (cos(t1) * sin(t0) * sin(t2) + cos(t2) * sin(t0) * sin(t1)) - L2 * sin(t0) * sin(t1) +
              L3 * cos(t1) * cos(t2) * sin(t0) - L3 * sin(t0) * sin(t1) * sin(t2);
  double pz = L1 - L4 * (cos(t1) * cos(t2) - sin(t1) * sin(t2)) + L2 * cos(t1) + L3 * cos(t1) * sin(t2) +
              L3 * cos(t2) * sin(t1);

  return coordinate(px, py, pz);
}

coordinate Dof_4(double t0, double t1, double t2) {
  double px = L3 * cos(t0) * cos(t1) * cos(t2) - L2 * cos(t0) * sin(t1) - L3 * cos(t0) * sin(t1) * sin(t2);
  double py = L3 * cos(t1) * cos(t2) * sin(t0) - L2 * sin(t0) * sin(t1) - L3 * sin(t0) * sin(t1) * sin(t2);
  double pz = L1 + L2 * cos(t1) + L3 * cos(t1) * sin(t2) + L3 * cos(t2) * sin(t1);

  return coordinate(px, py, pz);
}

coordinate Dof_3(double t0, double t1) {
  double px = (-1) * L2 * cos(t0) * sin(t1);
  double py = (-1) * L2 * sin(t0) * sin(t1);
  double pz = L1 + L2 * cos(t1);

  return coordinate(px, py, pz);
}

coordinate Dof_2(double t0) {
  double px = 0.0;
  double py = 0.0;
  double pz = L1;

  return coordinate(px, py, pz);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "joints_pos_publisher");

  ros::NodeHandle pos_publisher;
  ros::NodeHandle j1_handle;
  ros::NodeHandle j2_handle;
  ros::NodeHandle j3_handle;
  ros::NodeHandle j4_handle;
  ros::NodeHandle j5_handle;

  ros::Subscriber j1_sub = j1_handle.subscribe("/rover_arm_j1_joint_position_controller/command", 1, j1_callback);
  ros::Subscriber j2_sub = j2_handle.subscribe("/rover_arm_j2_joint_position_controller/command", 1, j2_callback);
  ros::Subscriber j3_sub = j3_handle.subscribe("/rover_arm_j3_joint_position_controller/command", 1, j3_callback);
  ros::Subscriber j4_sub = j4_handle.subscribe("/rover_arm_j4_joint_position_controller/command", 1, j4_callback);
  ros::Subscriber j5_sub = j5_handle.subscribe("/rover_arm_j5_joint_position_controller/command", 1, j5_callback);
  ros::Publisher pos_pub = pos_publisher.advertise<std_msgs::String>("ui_joint_pos", 100);

  ros::Rate loop_rate(1);

  coordinate j2_pos = coordinate(0.0, 0.0, 0.0);
  coordinate j3_pos = coordinate(0.0, 0.0, 0.0);
  coordinate j4_pos = coordinate(0.0, 0.0, 0.0);
  coordinate j5_pos = coordinate(0.0, 0.0, 0.0);
  coordinate ee_pos = coordinate(0.0, 0.0, 0.0);

  string my_message;

  while (ros::ok()) {
    std_msgs::String msg_last;
    my_message = "";

    j2_pos = Dof_2(j1_angle);
    j3_pos = Dof_3(j1_angle, j2_angle);
    j4_pos = Dof_4(j1_angle, j2_angle, j3_angle);
    j5_pos = Dof_5(j1_angle, j2_angle, j3_angle);
    ee_pos = Dof_6(j1_angle, j2_angle, j3_angle, j4_angle, j5_angle);

    cout << "Joint 2: " << j2_pos.x << "  " << j2_pos.y << "  " << j2_pos.z << endl;
    cout << "Joint 3: " << j3_pos.x << "  " << j3_pos.y << "  " << j3_pos.z << endl;
    cout << "Joint 4: " << j4_pos.x << "  " << j4_pos.y << "  " << j4_pos.z << endl;
    cout << "Joint 5: " << j5_pos.x << "  " << j5_pos.y << "  " << j5_pos.z << endl;
    cout << "EE: " << ee_pos.x << "  " << ee_pos.y << "  " << ee_pos.z << endl;

    my_message += conv_to_string(j2_pos.x) + " " + conv_to_string(j2_pos.z) + " ";
    my_message += conv_to_string(j3_pos.x) + " " + conv_to_string(j3_pos.z) + " ";
    my_message += conv_to_string(j4_pos.x) + " " + conv_to_string(j4_pos.z) + " ";
    my_message += conv_to_string(j5_pos.x) + " " + conv_to_string(j5_pos.z) + " ";
    my_message += conv_to_string(ee_pos.x) + " " + conv_to_string(ee_pos.z);

    cout << endl << my_message << endl;

    msg_last.data = my_message;
    pos_pub.publish(msg_last);

    ros::spinOnce();

    loop_rate.sleep();
  }
}
