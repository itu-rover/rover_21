#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf/transform_datatypes.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <unistd.h>


#include "autonomous_functions.h"

#define Z_UPPER 0.625
#define Z_LOWER -0.07
#define MODULUS_MAX 2


bool AutoFuncs::checkGoalBoundaries(std::vector<double> &goalXYZ){
	if (goalXYZ[2] > Z_UPPER){
		goalXYZ[2] = Z_UPPER;
	} 
	else if (goalXYZ[2] < Z_LOWER){
		goalXYZ[2] = Z_LOWER;
	}

	double modulus = sqrt(pow(goalXYZ[0],2)+pow(goalXYZ[1],2)+pow(goalXYZ[2],2));
	if (modulus > MODULUS_MAX){
		ROS_INFO("\n\n\nGoal is out of limits\n\n\n");
		return false;
	}
	else
		return true;
	
}

void AutoFuncs::approachCartesianGoal(std::vector<double> &goalXYZ,
	moveit::planning_interface::MoveGroupInterface &group,
	geometry_msgs::Pose &target_pose,
	std::string elementType){

	ROS_INFO("\nCAUTION: AUTONOMOUSLY APPROACHING!\n");
	sleep(5);
	

	double current_x = (double) target_pose.position.x;
	double current_y = (double) target_pose.position.y;
	
	if ((goalXYZ[0] > current_x) && goalXYZ[0] - current_x > 0.3){
		goalXYZ[0] = goalXYZ[0] - 0.3;
		// arbitrarily given x offset is 0.3, might be changed
	}

	if (goalXYZ[1] > 0 && goalXYZ[1] > current_y)
		goalXYZ[1] = goalXYZ[1] - 0.08;			// arbitrarily given
	else if (goalXYZ[1] < 0 && goalXYZ[1] < current_y)
		goalXYZ[1] = goalXYZ[1] + 0.08;

	target_pose.position.x = goalXYZ[0];
	target_pose.position.y = goalXYZ[1];
	target_pose.position.z = goalXYZ[2];

	ROS_INFO("XYZ Goal: %f %f %f",goalXYZ[0],goalXYZ[1],goalXYZ[2]);


	group.setPoseTarget(target_pose);
	group.setGoalTolerance(0.02);
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

	if (success){
		ROS_INFO("\nHere we go babe ;)\n");
		group.move();
	}
	else
		ROS_INFO("\n i just can't\n");

}

void AutoFuncs::alignYZ(std::vector<double> &goalXYZ,
	moveit::planning_interface::MoveGroupInterface &group,
	geometry_msgs::Pose &target_pose){
	ROS_INFO("\n\nAligning Y and Z\n");

	target_pose.position.y = goalXYZ[1];
	target_pose.position.z = goalXYZ[2];
	ROS_INFO("Align goal: x: %f y: %f z: %f",goalXYZ[0],goalXYZ[1],goalXYZ[2]);
	sleep(1);

	group.setPoseTarget(target_pose);
	group.setGoalTolerance(0.01);
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

	if (success){
		ROS_INFO("\nHere we go babe ;)\n");
		group.move();
	}
	else
		ROS_INFO("\n i just can't\n");

}

void AutoFuncs::approachOnX(double goalX,
	moveit::planning_interface::MoveGroupInterface &group,
	geometry_msgs::Pose &target_pose, int mode){
	ROS_INFO("\n\nApproaching on X: %f \n",goalX);
	sleep(1);
	goalX -= 0.2;
	
	double current_x = target_pose.position.x;
	double increment_x;

	if (mode == 1){
		increment_x = (goalX-current_x)/3;
		target_pose.position.x += increment_x;
	}
	else if (mode == 2){
		increment_x = (goalX-current_x)/2;
		target_pose.position.x += increment_x;
	} 
	else if (mode == 3){
		target_pose.position.x = goalX;
	}

	group.setPoseTarget(target_pose);
	group.setGoalTolerance(0.01);
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

	if (success){
		ROS_INFO("\nHere we go babe ;)\n");
		group.move();
	}
	else
		ROS_INFO("\n i just can't\n");

}

void AutoFuncs::rotateSwitch(moveit::planning_interface::MoveGroupInterface &group,const robot_state::JointModelGroup* joint_model_group, int direction, double radians){
	moveit::core::RobotStatePtr current_state = group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    if (direction == 0)
    	radians = -radians;
    joint_group_positions[5] += radians;
    group.setJointValueTarget(joint_group_positions);
    ROS_INFO("\n\nEnd effector is turning around %f\n\n",radians);
    group.move();

}

void AutoFuncs::actuateFingers(double &finger_pos, double radians, int action){

	if (action == 1)
		radians *= -1;
	finger_pos += radians;
	ROS_INFO("Actuating fingers %f", radians);

}

std::vector<double> AutoFuncs::rotatePoint(std::vector<double> position, std::vector<double> orientation){
    std::vector<double> new_coordinates{0.0,0.0,0.0};

    Eigen::Vector3f local_positions(position[0],position[1],position[2]);

    Eigen::Matrix3f rotation_mat = Eigen::Quaternionf(orientation[3], orientation[0], orientation[1], orientation[2]).toRotationMatrix();
    // W X Y Z

    Eigen::Vector3f global_positions = rotation_mat*local_positions;

    
    new_coordinates[0] = global_positions[0];
    new_coordinates[1] = global_positions[1];
    new_coordinates[2] = global_positions[2];
    
    return new_coordinates;
}

std::vector<double> AutoFuncs::updateCameraPosition(std::vector<double> link_5_pos, std::vector<double> link_5_ori, std::vector<double> rel_camera_pos){

    std::vector<double> new_camera_pos = link_5_pos;
    std::vector<double> camera_offset = AutoFuncs::rotatePoint(rel_camera_pos,link_5_ori);
    new_camera_pos[0] += camera_offset[0];
    new_camera_pos[1] += camera_offset[1];
    new_camera_pos[2] += camera_offset[2];

    return new_camera_pos;
}

std::vector<double> AutoFuncs::updateCameraOrientation(std::vector<double> link_5_ori,std::vector<double> rel_camera_ori){
	std::vector<double> new_camera_ori{0.0,0.0,0.0,0.0};

	Eigen::Quaterniond link6_quaternion(link_5_ori[3],link_5_ori[0],link_5_ori[1],link_5_ori[2]);
	Eigen::Quaterniond camera_relative_quaternion(rel_camera_ori[3],rel_camera_ori[0],rel_camera_ori[1],rel_camera_ori[2]);

	Eigen::Quaterniond global_cam_ori = link6_quaternion*camera_relative_quaternion;

	new_camera_ori[0] = global_cam_ori.x();
	new_camera_ori[1] = global_cam_ori.y();
	new_camera_ori[2] = global_cam_ori.z();
	new_camera_ori[3] = global_cam_ori.w();

	return new_camera_ori;
	// W,X,Y,Z
}

std::vector<double> AutoFuncs::transformGoal(std::vector<double> camera_orientation, std::vector<double> camera_pos,std::vector<double> rel_cam_XYZ){

	std::vector<double> goal_global = camera_pos;
	std::vector<double> goal_offset = AutoFuncs::rotatePoint(rel_cam_XYZ,camera_orientation);
	goal_global[0] += goal_offset[0];
	goal_global[1] += goal_offset[1];
	goal_global[2] += goal_offset[2];

	return goal_global;
}

