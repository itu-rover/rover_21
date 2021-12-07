#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp> // import OpenCV library.
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "SobelandDisplay.hpp" // import Sobel display header


int main(int argc, char **argv)
{   
    ros::init(argc, argv, "Erc_Science_Filter_RealSense");
    ros::NodeHandle nh;
    
    image_transport::ImageTransport it_real(nh);
    image_transport::Subscriber camera_sub_real = it_real.subscribe("/camera/color/image_raw", 1, camera_bridge_check_real);
    
    
    std::cout << "Realsense Ready..." << std::endl;
    std::cout << "Video Capturing..." << std::endl;
    std::cout << std::endl;

    while (true)
    {
        if (new_frame_real)
        {
            
            sobel_and_display(frames_real);
            //sobel_and_display(frames_right);

            char c = (char)cv::waitKey(25); // When ESC key pressed program closed
            if(c == 27)
            {
                break;
            }
            new_frame_real = false;
        }
        
        ros::spinOnce();
    }
        
        return 0;
}
