/*
    Date: 19.02.2021
    Author: Berke Algül
*/

#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>

#define PI 3.14159265

void pathCallback(const nav_msgs::Path::ConstPtr&);
void odomCallback(const nav_msgs::Odometry::ConstPtr&);

nav_msgs::Path::ConstPtr path;
float x, y, yaw;

float ANGULAR_SPEED_TRESH = PI / 6;
float K_P = 5.0;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_following");
    ros::NodeHandle nh;

    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "goal", 10);
    ros::Subscriber path_sub = nh.subscribe("/path", 10, pathCallback);
    ros::Subscriber odom_sub = nh.subscribe("/odometry/filtered", 10, odomCallback);
    
    while(ros::ok())
    {
        ros::spinOnce();

        float pX, pY;

        if(!path)
            continue;

        pX = path->poses[0].pose.position.x;
        pY = path->poses[0].pose.position.y;

        float target_yaw = (float)atan2(pY, pX);
        float yaw_error = target_yaw - yaw;
        yaw_error = atan2(sin(yaw_error), cos(yaw_error));

        geometry_msgs::Twist twist;

        if(abs(yaw_error) > ANGULAR_SPEED_TRESH)
            twist.linear.x = 0;
        else
            twist.linear.x = 1.5;
        
        twist.angular.z = yaw_error*K_P;
        twist_pub.publish(twist);


        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "goal";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0;
        tf2::Quaternion q;
        q.setRPY(0 , 0, target_yaw ); 
        marker.pose.orientation.x = q[0];
        marker.pose.orientation.y = q[1];
        marker.pose.orientation.z = q[2];
        marker.pose.orientation.w = q[3];
        marker.scale.x = 1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    }

    return 0;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
    //ROS_INFO("ODOM SUBBED");
    // covert quaternion to euler and extract yaw
    tf::Quaternion q(
        odom->pose.pose.orientation.x,
        odom->pose.pose.orientation.y,
        odom->pose.pose.orientation.z,
        odom->pose.pose.orientation.w);
    tf::Matrix3x3 m(q); // q -> matrix
    double r, p, y_;
    m.getRPY(r, p, y_); // matrx -> euler
    
    yaw = (float)y_;
    x = odom->pose.pose.position.x;
    y = odom->pose.pose.position.y;
}

void pathCallback(const nav_msgs::Path::ConstPtr &path_)
{
    //ROS_INFO("Path subbed");
    path = path_;
}