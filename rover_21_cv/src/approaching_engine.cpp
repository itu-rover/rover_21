#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "ar_tracker/ar_tracker.hpp"

// Berke Algül 25.9.2020

class ApproachingEngine {
 public:
  tf::TransformBroadcaster br;
  ArTracker::OneTimeTracker tracker;

  tf::Quaternion Rcam;

  void camera_cb(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat frame = cv_ptr->image;

    int tags = tracker.run_frame(frame);

    if (tags > 0) {
      ROS_INFO_STREAM("FOUND ARTAG: " << tags);
    }

    // TODO: add launch arg for imshow
    int x = waitKey(1);
    imshow("frame", frame);
  }

  ApproachingEngine() {
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    std::string camera_topic;
    nh.getParam("/camera/color_topic", camera_topic);
    image_transport::Subscriber camera_sub = it.subscribe(camera_topic, 1, &ApproachingEngine::camera_cb, this);

    // TODO: change initial angles angles x front y right z up
    Rcam.setRPY(PI / 2, 0, PI);
    ros::spin();
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "approaching_engine");
  ApproachingEngine ae;
  return 0;
}
