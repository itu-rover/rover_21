#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>

class CameraPub {
 public:
  image_transport::Publisher color_pub, gray_pub;

  bool gray;
  std::string name;
  cv::VideoCapture cap;

  CameraPub();
  void main();
};

CameraPub::CameraPub() {
  ros::NodeHandle nh("~");
  bool success = true;

  // private params
  success = nh.getParam("gray", gray);
  success = nh.getParam("name", name);

  // publisher topics
  std::string color_topic, gray_topic;
  success = nh.getParam("/camera/color_topic", color_topic);
  success = nh.getParam("/camera/gray_topic", gray_topic);

  int cap_id;
  nh.getParam("cap_id", cap_id);

  if (!success) {
    ROS_ERROR("Something went wrong with params. Terminating!!!");
    ros::shutdown();
  }

  cap.open(cap_id);

  if (!cap.isOpened()) {
    ROS_ERROR("Camera is not opened!!!. Terminating...");
    ros::shutdown();
  }

  image_transport::ImageTransport it(nh);

  color_pub = it.advertise(color_topic, 1);
  gray_pub = it.advertise(gray_topic, 1);

  ROS_INFO("Initialization completed. we shall begin publishing");
  ROS_INFO_STREAM("Convert to gray: " << gray);

  // creating camera info msg

  // we shall begin publishing
  main();
}

void CameraPub::main() {
  ros::Rate loop_rate(30);

  while (ros::ok() && cap.isOpened()) {
    cv::Mat image;
    cap.read(image);

    if (gray) {
      cv::Mat gray_image;
      cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray_image).toImageMsg();
      gray_pub.publish(msg);
    }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    color_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "camera_node");
  CameraPub();
  return 0;
}
