#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <moveit_msgs/DisplayRobotState.h>
#include <Eigen/Dense>

#include "moveit_functions.h"

#define PI 3.141592654


void CustomMoveItFuncs::updateJointStates(moveit::planning_interface::MoveGroupInterface &group,moveit::planning_interface::MoveGroupInterface::Plan &myPlan,std::vector<double> encoder_list){

    bool success;
    group.setJointValueTarget(encoder_list);
    success = (group.plan(myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
    group.move();
    }
    
void CustomMoveItFuncs::executeCartesianPath(std::vector<float> deltaXYZ,moveit::planning_interface::MoveGroupInterface &group, std::vector<geometry_msgs::Pose> &waypoints,geometry_msgs::Pose &target_pose){
        target_pose.position.x += deltaXYZ[0];
        target_pose.position.y += deltaXYZ[1];
        target_pose.position.z += deltaXYZ[2];

        waypoints.push_back(target_pose);

        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        if(fraction == 1){
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            group.execute(plan);
        } 
}

void CustomMoveItFuncs::executeRPYGoal(std::vector<float> deltaRPY, moveit::planning_interface::MoveGroupInterface &group,const robot_state::JointModelGroup* joint_model_group){

    moveit::core::RobotStatePtr current_state = group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[5] += deltaRPY[0]*3;
    joint_group_positions[3] -= deltaRPY[1]*3;
    joint_group_positions[4] -= deltaRPY[2]*3;  // radians 
    group.setJointValueTarget(joint_group_positions);
    group.move();

}

void CustomMoveItFuncs::executeJointGoal(std::vector<double> deltaJoints, moveit::planning_interface::MoveGroupInterface &group,const robot_state::JointModelGroup* joint_model_group, double &finger_pos){
    moveit::core::RobotStatePtr current_state = group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    for (int i=0; i<joint_group_positions.size(); i++){
        joint_group_positions[i] += deltaJoints[i]/750;
    }

    finger_pos += deltaJoints[6]/4500;
    // fingers

    group.setJointValueTarget(joint_group_positions);
    group.move();
}

std_msgs::Float64MultiArray CustomMoveItFuncs::returnJointAngles(moveit::planning_interface::MoveGroupInterface &group, int mode, double finger_pos, int activity_indicator, int probe_servo){

    std::vector<double> joint_values = group.getCurrentJointValues();
    //ROS_INFO("%f|%f|%f|%f|%f|%f",joint_values[0],joint_values[1],joint_values[2],joint_values[3],joint_values[4],joint_values[5]);
    std_msgs::Float64MultiArray array_of_angles;
    for (int i = 0; i < 6; i++)
    {
        array_of_angles.data.push_back(joint_values[i]);
        //ROS_INFO("%f %f",i,joint_values[i]);
    }
    //loop_rate.sleep();


    array_of_angles.data.push_back(finger_pos);
    array_of_angles.data.push_back(mode);
    array_of_angles.data.push_back(activity_indicator);    // Activity indicator LED 
    array_of_angles.data.push_back(probe_servo);           //probe servo

    // 8 -> mode bit

    return array_of_angles;
}


