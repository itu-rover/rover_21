#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include "joy_functions.h"

bool JoyFuncs::checkJoyDeadband(float data){
    if (abs(data) > 0.15){
        return true;
    }
    else 
        return false;
}

std::vector<float> JoyFuncs::xyzControl(float x, float y, float z, float frac){
    std::vector <float> deltaXYZ{0.0,0.0,0.0};

    if (checkJoyDeadband(x))
        deltaXYZ[0] = x*frac;
    else
        deltaXYZ[0] = 0;

    if (checkJoyDeadband(y))
        deltaXYZ[1] = y*frac;
    else
        deltaXYZ[1] = 0;

    if (checkJoyDeadband(z))
        deltaXYZ[2] = z*frac;
    else
        deltaXYZ[2] = 0;

    return deltaXYZ;
}

std::vector<float> JoyFuncs::rpyControl(float r, float p, float y, float frac){
    std::vector <float> deltaRPY{0.0,0.0,0.0};

    if (checkJoyDeadband(r))
        deltaRPY[0] = r*frac;
    else
        deltaRPY[0] = 0;

    if (checkJoyDeadband(p))
        deltaRPY[1] = p*frac;
    else
        deltaRPY[1] = 0;

    if (checkJoyDeadband(y))
        deltaRPY[2] = y*frac;
    else
        deltaRPY[2] = 0;

    return deltaRPY;
}

std::vector<std::string> JoyFuncs::split(std::string strToSplit, char delimeter)
{
    // a function to split data coming from the database. got it from a tutorial 
    std::stringstream ss(strToSplit);
    std::string item;
    std::vector<std::string> splittedStrings;
    while (std::getline(ss, item, delimeter))
    {
       splittedStrings.push_back(item);
    }
    return splittedStrings;
}


std::vector<double> JoyFuncs::updateFromFile(){
    //bu calisacak mi emin degilim
    std::vector<double> encoder_list{0.0,0.0,0.0,0.0,0.0,0.0};
    std::ifstream file("encoder_db.txt");
    std::string str;
    std::getline(file, str);
    std::vector<std::string> splitted = split(str,',');
    for (int i=0;i<splitted.size();i++)
    {
        encoder_list[i] = -std::stof(splitted[i]);
        std::cout << i << std::endl;
        std::cout << encoder_list[i] << std::endl;
    }
    return encoder_list;
}

/*
std::vector<double> JoyFuncs::updateFromNode(std::vector<double> encoder_list){
    
    for (int i=0;i<encoder_list.size();i++)
    {
        encoder_list[i] = std::stof(splitted[i]);
        std::cout << i << std::endl;
        std::cout << encoder_list[i] << std::endl;
    }
    return encoder_list;
}
*/

bool JoyFuncs::checkJoyNullData(const sensor_msgs::Joy::ConstPtr& joy){
    if (joy == NULL)
        return false;
    else{
        //ROS_INFO("TRUE\n\n");
        return true;
    }
}

std::vector<double> JoyFuncs::forwardKinematicsFunc(const sensor_msgs::Joy::ConstPtr& joy, int frac){

    //int frac = 150;
    std::vector<double> velocity_vector{0.0,0.0,0.0,0.0,0.0,0.0,0.0};

    //std::cout << "FK Joy " << std::endl;

    if (checkJoyNullData(joy)){

        ROS_INFO("GIRDI\n\n");

        if (joy->buttons[0] != 0){
        // fingers
        float fingers = joy->axes[0];

        if (checkJoyDeadband(fingers))
            velocity_vector[6] = fingers*frac;
        else
            velocity_vector[6] = 0;
    } 

    else if (joy->buttons[1] != 0){
        // axis 6
        float axis_6 = joy->axes[0];
        if (checkJoyDeadband(axis_6))
            velocity_vector[5] = -axis_6*frac;
        else
            velocity_vector[5] = 0;
    }

    else if (joy->buttons[2] != 0){
        // axis 5
        float axis_5 = joy->axes[0];
        if (checkJoyDeadband(axis_5))
            velocity_vector[4] = axis_5*frac;
        else
            velocity_vector[4] = 0;
    }

    else if (joy->buttons[3] != 0){
        // axis 4
        float axis_4 = joy->axes[1];
        if (checkJoyDeadband(axis_4))
            velocity_vector[3] = axis_4*frac;
        else
            velocity_vector[3] = 0;
    }

    else {

        float axis_1, axis_2, axis_3;
        axis_1 = joy->axes[0];
        axis_2 = joy->axes[1];
        axis_3 = joy->axes[4];

        if (checkJoyDeadband(axis_1))
            velocity_vector[0] = axis_1*frac/1.5;
        else
            velocity_vector[0] = 0;

        if (checkJoyDeadband(axis_2))
            velocity_vector[1] = axis_2*frac;
        else
            velocity_vector[1] = 0;

        if (checkJoyDeadband(axis_3))
            velocity_vector[2] = axis_3*frac;
        else
            velocity_vector[2] = 0;
    }
    }

    else{
        ROS_INFO("GIRMEDI\n\n");
    }


    return velocity_vector;
    

    
}

std_msgs::Float64MultiArray JoyFuncs::returnActuatorVel(const std::vector<double> &actuatorVelocities, int activity_mode, int probe_servo){
    
    std_msgs::Float64MultiArray act_vel;
    for (int i = 0; i < 7; i++)
    {
        act_vel.data.push_back(actuatorVelocities[i]);
        //std::cout << "push back: " << actuatorVelocities[i] << std::endl;
    } 

    act_vel.data.push_back(5);
    act_vel.data.push_back(activity_mode);
    act_vel.data.push_back(probe_servo);
    // arbitrarly chosen mode bit for serial message constructor
    return act_vel;
}