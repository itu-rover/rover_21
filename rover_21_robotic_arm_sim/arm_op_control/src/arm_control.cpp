#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64MultiArray.h>
/////////////
#include <std_msgs/String.h>
//////////////////
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <unistd.h>

///////////////
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
///////////////

#include "joy_functions.h"
#include "moveit_functions.h"
#include "autonomous_functions.h"
#include <unistd.h>

static const std::string PLANNING_GROUP = "manipulator";

std::vector<float> deltaXYZ{0.0,0.0,0.0};
std::vector<float> deltaRPY{0.0,0.0,0.0};
std::vector<double> actuatorVelocities{0.0,0.0,0.0,0.0,0.0,0.0,0.0};
// global variables to control end effector position and orientation

std_msgs::Float64MultiArray array_of_angles;
std_msgs::Float64MultiArray velocity_of_motors;
// global arrays to publish other nodes

std::vector<double> encoder{0.0,0.0,0.0,0.0,0.0,0.0};
bool first_run = true;
bool update_joints = false;
// global variable to store last encoder data

std::vector<double> rel_camera_pos{0, 0.1, 0.21};
std::vector<double> rel_camera_ori{-0.2334454, 0.0, 0.0, 0.9723699};
// X Y Z W
// camera calibration parameters, must be upgraded (relative ones)

std::vector<double> camera_pos{0.0,0.0,0.0};
std::vector<double> camera_orientation{0.0,0.0,0.0,0.0};
// X Y Z W
// camera position and orientation according to the global origin

std::vector<double> link_6_pos{0.0,0.0,0.0};
std::vector<double> link_6_ori{0.0,0.0,0.0,0.0};

std::vector<double> link_5_pos{0.0,0.0,0.0};
std::vector<double> link_5_ori{0.0,0.0,0.0,0.0};

std::vector<double> goalXYZ{0.0,0.0,0.0};
std::vector<double> rel_cam_XYZ{0.0,0.0,0.0};
bool go_autonomous = false;
bool go_auto_check2 = false;
bool go_auto_check3 = false;
// global variables to perform autonomous approach & manipulation


// for fingers position control
double finger_pose = 0.0;

int mode = 3; 
int switch_btn_prev = 0;
float frac = 0.002;
int activity_mode = 1; // 1 IDLE, 2 TELEOPERATION, 3 AUTONOMOUS
int servo_action = 0;   // 0 closed, 1 open
// global variables to achieve a mode switch algorithm, they are being used in joyCallback function

int probe_deploy_flg = 0;
int probe_pickup_flg = 0;



void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

    if (joy->buttons[0] && joy->buttons[2] && joy->buttons[4]){
        probe_deploy_flg = 1;
    }

    if (joy->buttons[1] && joy->buttons[3] && joy->buttons[4]){
        probe_pickup_flg = 1;
    }

    if ((!joy->buttons[4]) && (!joy->buttons[5]))
    {

    activity_mode = 2;
    int switch_btn_current = joy->buttons[7];
    int autonomous_appr = joy->buttons[4];

    if (joy->buttons[6] == 1){
            mode = 3;
        }
    


    if (switch_btn_current == 1 && switch_btn_prev == 0){

        mode += 1;
        if (mode >= 6)
            mode = 0;
        if (mode == 0){
            frac = 0.002;
            ROS_INFO("Control mode: JOINT SPACE CONTROL: POSITION\n");
            update_joints = true;
        }
        else if (mode == 1){
            frac = 0.01;
            ROS_INFO("Control mode: CARTESIAN SPACE SLOW \n");
        }
        else if (mode == 2){
            frac = 0.03;
            ROS_INFO("Control mode: CARTESIAN SPACE: FAST /tikkatli olun\n");
        } 
        else if (mode == 3){
            ROS_INFO("Control mode: JOINT SPACE CONTROL: SPEEDY GONZALES\n");
        }
        else if (mode == 4){
            ROS_INFO("Control mode: JOINT SPACE CONTROL: SPEED / SLOW MODE\n");
        }
        else if (mode == 5 && !autonomous_appr){
            mode += 1;
        } 
        else if (mode == 5 && autonomous_appr){
            update_joints = true;
            ROS_INFO("Autonomous mode!\n");
        }
    }
    switch_btn_prev = switch_btn_current;

    if (mode == 0){
        actuatorVelocities = JoyFuncs::forwardKinematicsFunc(joy,150);
    }

    else if (mode == 1 || mode == 2){
        float x = joy->axes[1];
        float y = joy->axes[0];
        float z = joy->axes[7];
        float roll = joy->axes[6];
        float pitch = joy->axes[4];
        float yaw = joy->axes[3];

        deltaXYZ = JoyFuncs::xyzControl(x,y,z,frac);
        deltaRPY = JoyFuncs::rpyControl(roll,pitch,yaw,frac);
        // cartesian point position and euler angles orientation control
    } 

    else if (mode == 3){
        actuatorVelocities = JoyFuncs::forwardKinematicsFunc(joy,150);
        servo_action ^= joy->buttons[5];
    }

    else if (mode == 4){
        actuatorVelocities = JoyFuncs::forwardKinematicsFunc(joy,100);
        servo_action ^= joy->buttons[5];
    }

    else if (mode == 5){
        go_autonomous = joy->buttons[0];
        if (go_autonomous)
            ROS_INFO("\n\n\n GIDIYOM BEN \n\n\n");
        }
    }
}

void encCallback(const std_msgs::Float64MultiArray array){
    for (int i=0; i < encoder.size(); i++){
        encoder[i] = array.data[i];
    } 
    encoder[2] = -encoder[2];
}

void autoCallback(const std_msgs::Float64MultiArray array){
    for (int i=0; i<3; i++){
        rel_cam_XYZ[i] = array.data[i];
    }
    go_auto_check3 = array.data[3];
    //ROS_INFO("Incoming depth camera data\n");
}

int main(int argc, char **argv)
{




    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle nh;
    ros::Rate loop_rate(150);
    ros::Subscriber joy_sub = nh.subscribe("joy",1,joyCallback);
    ros::Subscriber enc_sub = nh.subscribe("joint_states/get",1,encCallback);
    ros::Subscriber auto_sub = nh.subscribe("auto_arm_topic",1,autoCallback);
    //ros::Subscriber tf_sub = nh.subscribe("/tf",10,tfCallback);

    ros::Publisher joint_values_pub = nh.advertise<std_msgs::Float64MultiArray>("joint_states/send", 1000);
    ros::Publisher deploy_check_pub = nh.advertise<std_msgs::String>("probe/deploy", 1000);
    ros::Publisher pickup_check_pub = nh.advertise<std_msgs::String>("probe/pickup", 1000);
    // ROS Procedure

    
    ros::AsyncSpinner spinner(1); 
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    //bool success;

////////////////////////////////////////////////
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
///////////////////////////////////////////////

    //// LED INDICATOR -> 5 seconds of delay //////
    activity_mode = 1; // Green
    array_of_angles = CustomMoveItFuncs::returnJointAngles(move_group,8,finger_pose,activity_mode,servo_action);
    joint_values_pub.publish(array_of_angles);
    usleep(5*1000000);
    ///////////////////////////////////////////////


    ROS_INFO("GO ROBOT \n");
    // for first run

    while (ros::ok())
    {

    ros::spinOnce();

    if (probe_deploy_flg){
        std_msgs::String msg;
        std::stringstream ss;
        ss << "F";
        msg.data = ss.str();    
        deploy_check_pub.publish(msg);
        probe_deploy_flg = 0;
    }

    if (probe_pickup_flg){
        std_msgs::String msg;
        std::stringstream ss;
        ss << "F";
        msg.data = ss.str();    
        pickup_check_pub.publish(msg);
        probe_pickup_flg = 0;
    }

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose current_pose;
    current_pose = move_group.getCurrentPose().pose;

    waypoints.push_back(current_pose);
    // MoveIt procedure

    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("base_link", "link_5",
                               ros::Time(0));
      link_5_pos[0] = transformStamped.transform.translation.x;
      link_5_pos[1] = transformStamped.transform.translation.y;
      link_5_pos[2] = transformStamped.transform.translation.z;

      link_5_ori[0] = transformStamped.transform.rotation.x;
      link_5_ori[1] = transformStamped.transform.rotation.y;
      link_5_ori[2] = transformStamped.transform.rotation.z;
      link_5_ori[3] = transformStamped.transform.rotation.w;

    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }


////////////////////////////// UPDATING /////////////////////////
// Here, the goal is to update joint states according to the encoder data.
// this part should be executed only on first run or after mode is changed from Forward Kine. to something else

    activity_mode = 2;

    if (first_run || update_joints){
        if (first_run) {
            encoder = JoyFuncs::updateFromFile();
            first_run = false;
        }
        else if (update_joints){
            //encoder = JoyFuncs::updateFromNode();
            update_joints = false;
        }
        CustomMoveItFuncs::updateJointStates(move_group,my_plan,encoder);
    }
////////////////////////////////////////////////////////////////

    if (mode == 0){

        /// TELEOPERATION / YELLOW 
        activity_mode = 2;


        ROS_INFO("JOINT SPACE MODE: POSITION");
        CustomMoveItFuncs::executeJointGoal(actuatorVelocities,move_group,joint_model_group,finger_pose);
        array_of_angles = CustomMoveItFuncs::returnJointAngles(move_group,8,finger_pose,activity_mode,servo_action);
        joint_values_pub.publish(array_of_angles);
    }

    else if (mode == 1 || mode == 2){
        
       

        /// TELEOPERATION / YELLOW 
        activity_mode = 2;

        ROS_INFO("CARTESIAN SPACE MODE: %d\nCurrent position: %f %f %f \n",
            mode,current_pose.position.x,current_pose.position.y,current_pose.position.z);

        geometry_msgs::Pose target_pose = current_pose;

        if(deltaXYZ[0] != 0 || deltaXYZ[1] != 0 || deltaXYZ[2] != 0) 
        {
            CustomMoveItFuncs::executeCartesianPath(deltaXYZ,move_group,waypoints,target_pose);
        }

        else if (deltaRPY[0] != 0 || deltaRPY[1] != 0 || deltaRPY[2] != 0)
        {
            CustomMoveItFuncs::executeRPYGoal(deltaRPY,move_group,joint_model_group);
        }

        
        array_of_angles = CustomMoveItFuncs::returnJointAngles(move_group,8,finger_pose,activity_mode,servo_action);

        // the second argument is arbitrarily given as serial mode, will be changed afterwards

        joint_values_pub.publish(array_of_angles);
        // Publishing joint states to the serial message constructor

    }

    else if (mode == 3 || mode == 4){
    

        /// TELEOPERATION / YELLOW 
            activity_mode = 2;

            if (mode == 3)
                ROS_INFO("JOINT SPACE MODE: SPEEDY GONZALES");
            else 
                ROS_INFO("JOINT SPACE MODE: SLOW MODE ");
            
            velocity_of_motors = JoyFuncs::returnActuatorVel(actuatorVelocities,activity_mode,servo_action);
            joint_values_pub.publish(velocity_of_motors);
        
        
        

    }

    else if (mode == 5){

        /// AUTONOMOUS / RED 
        activity_mode = 3;


        ROS_INFO("AUTONOMOUS______: x: %f y: %f z: %f",current_pose.position.x,current_pose.position.y,current_pose.position.z);
        geometry_msgs::Pose target_pose = current_pose;

///////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////      WIP     ///////////////////////////////////////


        //AutoFuncs::updateLink6Pose(link_6_pos,link_6_ori,current_pose);
        camera_pos = AutoFuncs::updateCameraPosition(link_5_pos,link_5_ori,rel_camera_pos);
        camera_orientation = AutoFuncs::updateCameraOrientation(link_5_ori,rel_camera_ori);

        //goalXYZ = AutoFuncs::transformGoal(camera_orientation,camera_pos,rel_cam_XYZ);
        goalXYZ = rel_cam_XYZ;

         ROS_INFO("rel_cam_XYZ x: %f  y: %f z: %f\n goalXYZ x: %f y: %f z: %f",
            rel_cam_XYZ[0],rel_cam_XYZ[1],rel_cam_XYZ[2],goalXYZ[0],goalXYZ[1],goalXYZ[2]);
        

        /*ROS_INFO("EEF Pos: %f %f %f \n",
            current_pose.position.x,current_pose.position.y,current_pose.position.z);
        */

        
        //ROS_INFO("\n \nCamera pose: %f %f %f,\n Camera ori %f %f %f %f \n link 5 pos: %f %f %f \n link5 ori: %f %f %f %f",
        //camera_pos[0],camera_pos[1],camera_pos[2],camera_orientation[0],camera_orientation[1],camera_orientation[2],camera_orientation[3],link_5_pos[0],link_5_pos[1],link_5_pos[2],link_5_ori[0],link_5_ori[1],link_5_ori[2],link_5_ori[3]);
        

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////        
        //array_of_angles = CustomMoveItFuncs::returnJointAngles(move_group,8,finger_pose);
        //joint_values_pub.publish(array_of_angles);
        
        array_of_angles = CustomMoveItFuncs::returnJointAngles(move_group,8,finger_pose,activity_mode,servo_action);
        joint_values_pub.publish(array_of_angles);

        if (go_autonomous){

            go_auto_check2 = AutoFuncs::checkGoalBoundaries(goalXYZ);  
            if (go_auto_check2 && go_auto_check3){

                //TODO: 5 seconds of time delay

                //AutoFuncs::approachCartesianGoal(goalXYZ,move_group,target_pose,"button");

                
                AutoFuncs::alignYZ(goalXYZ,move_group,target_pose);
                array_of_angles = CustomMoveItFuncs::returnJointAngles(move_group,8,finger_pose,3,servo_action);
                joint_values_pub.publish(array_of_angles);

                AutoFuncs::approachOnX(goalXYZ[0],move_group,target_pose,1);
                array_of_angles = CustomMoveItFuncs::returnJointAngles(move_group,8,finger_pose,3,servo_action);
                joint_values_pub.publish(array_of_angles);

                AutoFuncs::alignYZ(goalXYZ,move_group,target_pose);
                array_of_angles = CustomMoveItFuncs::returnJointAngles(move_group,8,finger_pose,3,servo_action);
                joint_values_pub.publish(array_of_angles);

                AutoFuncs::approachOnX(goalXYZ[0],move_group,target_pose,3);
                array_of_angles = CustomMoveItFuncs::returnJointAngles(move_group,8,finger_pose,3,servo_action);
                joint_values_pub.publish(array_of_angles);

                AutoFuncs::rotateSwitch(move_group,joint_model_group,0,0.4);

                go_autonomous = false;
                
                sleep(1);
                

        }

        }

    }
    loop_rate.sleep();
    update_joints = false;
    go_autonomous = false;
    
    }  
    return 0;
}