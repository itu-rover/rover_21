#ifndef AUTONOMOUS_FUNCTIONS
#define AUTONOMOUS_FUNCTIONS

#include <string>
#include <vector>

namespace AutoFuncs{

bool checkGoalBoundaries(std::vector<double> &goalXYZ);
/* MoveIt cannot execute the motion plan if the goal is out of the reachable region.
The goal of this function is to check given coordinates and if the points are not in
the region, return a feasible point. */

void approachCartesianGoal(std::vector<double> &goalXYZ,
	moveit::planning_interface::MoveGroupInterface &group,
	geometry_msgs::Pose &target_pose,
	std::string elementType);
/* The robotic arm will approach to the given goal. A safety offset must be determined
elementType: Switch, dimmer, probe etc. Don't know what to do now, gonna figure it out */

void alignYZ(std::vector<double> &goalXYZ,
	moveit::planning_interface::MoveGroupInterface &group,
	geometry_msgs::Pose &target_pose);

void approachOnX(double goalX,
	moveit::planning_interface::MoveGroupInterface &group,
	geometry_msgs::Pose &target_pose, int mode);
/* mode 1: go 1/3 of error, mode 2: go 1/2 of error, mode 3: go directly */

void rotateSwitch(moveit::planning_interface::MoveGroupInterface &group,const robot_state::JointModelGroup* joint_model_group, int direction, double radians);
/* Rotates the sixth axis in given direction: 1 clockwise 0 counter-clockwise */

void actuateFingers(double &finger_pos, double radians, int action);
/* Opens or closes the fingers, action indicates direction. 1 opens 0 closes */

std::vector<double> rotatePoint(std::vector<double> position, std::vector<double> orientation);
/* Calculates the projectile position of a point in a child frame on a parent frame.
It uses quaternion to construct rotation matrix */

std::vector<double> updateCameraPosition(std::vector<double> link_5_pos, std::vector<double> link_5_ori, std::vector<double> rel_camera_pos);
/* Updates the camera position. Returns the camera position according to the global frame */

std::vector<double> updateCameraOrientation(std::vector<double> link_5_ori,std::vector<double> rel_camera_ori);
/* Updates the camera orientation. Returns the camera orientation according to the global frame */

std::vector<double> transformGoal(std::vector<double> camera_orientation, std::vector<double> camera_pos,std::vector<double> rel_cam_XYZ);
/* Transforms the relative position of the detected object to the global position. 
Returns global position */ 

}


#endif