#ifndef JOY_FUNCTIONS
#define JOY_FUNCTIONS

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>

namespace JoyFuncs{

bool checkJoyDeadband(float data);

std::vector<float> xyzControl(float x, float y, float z, float frac);

std::vector<float> rpyControl(float r, float p, float y, float frac);

std::vector<std::string> split(std::string strToSplit, char delimeter);

std::vector<double> updateFromFile();

bool checkJoyNullData(const sensor_msgs::Joy::ConstPtr& joy);

/*
std::vector<double> updateFromNode();
*/
//TODO: This function will de defined
// This function should evaluate the data which is read from encoders. A subscriber will be defined 

std::vector<double> forwardKinematicsFunc(const sensor_msgs::Joy::ConstPtr& joy, int frac);

std_msgs::Float64MultiArray returnActuatorVel(const std::vector<double> &actuatorVelocities, int activity_mode, int probe_servo);

}



#endif
