#ifndef MOVEIT_FUNCTIONS
#define MOVEIT_FUNCTIONS


#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_msgs/Float64MultiArray.h>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>


namespace CustomMoveItFuncs{

void updateJointStates(moveit::planning_interface::MoveGroupInterface &group,moveit::planning_interface::MoveGroupInterface::Plan &myPlan,std::vector<double> encoder_list);
	
void executeCartesianPath(std::vector<float> deltaXYZ,moveit::planning_interface::MoveGroupInterface &group, std::vector<geometry_msgs::Pose> &waypoints, geometry_msgs::Pose &target_pose);

void executeRPYGoal(std::vector<float> deltaRPY, moveit::planning_interface::MoveGroupInterface &group,const robot_state::JointModelGroup* joint_model_group);

void executeJointGoal(std::vector<double> deltaJoints, moveit::planning_interface::MoveGroupInterface &group,const robot_state::JointModelGroup* joint_model_group,double &finger_pos);

std_msgs::Float64MultiArray returnJointAngles(moveit::planning_interface::MoveGroupInterface &group, int mode, double finger_pos,int activity_indicator, int probe_servo);

}

#endif