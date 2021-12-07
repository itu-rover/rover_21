#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp> // import OpenCV library.
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "SobelandDisplay.hpp" // import Sobel display header


int main(int argc, char **argv)
{   
    ros::init(argc, argv, "Erc_Science_Filter_Cam");
    ros::NodeHandle nh;


    image_transport::ImageTransport it_left(nh);
	image_transport::Subscriber camera_sub_left = it_left.subscribe("/c922_left/image_raw", 1, camera_bridge_check_left);
    //image_transport::Subscriber camera_sub_right = it_left.subscribe("/c922_left/image_raw", 1, camera_bridge_check_right);
    
    std::cout << "Cam Ready..." << std::endl;
    std::cout << "Video Capturing..." << std::endl;
    std::cout << std::endl;

    while (ros::ok())
    {
        if (new_frame_left)
        {
            sobel_and_display_left(frames_left);
            //int x = cv::waitKey(1);

            char c = (char)cv::waitKey(25); // When ESC key pressed program closed
            if(c == 27)
            {
                break;
            }
            new_frame_left = false;
        }
        
        if (new_frame_right)
        {
            sobel_and_display_right(frames_right);
            //int x = cv::waitKey(1);

            char c = (char)cv::waitKey(25); // When ESC key pressed program closed
            if(c == 27)
            {
                break;
            }
            new_frame_right = false;
        }
        
        ros::spinOnce();
    }
        
        return 0;
}
