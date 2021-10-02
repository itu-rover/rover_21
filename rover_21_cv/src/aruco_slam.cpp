#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "ar_tracker/tracker.h"

bool new_frame = false;
cv::Mat frame;

void camera_cb(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  frame = cv_ptr->image;
  new_frame = true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "aruco_slam");
  ros::NodeHandle n;

  std::string camera_topic;
  n.getParam("/camera/color_topic", camera_topic);

  image_transport::ImageTransport it(n);
  image_transport::Subscriber camera_sub = it.subscribe(camera_topic, 1, camera_cb);

  Mat_<double> mtx(3, 3);
  Mat_<double> dist(1, 5);
  dist << 0.020436355102596344, -0.11407839179793304, 0.004229887050454093, -0.01709654130034178, 0.13991605472148272;
  mtx << 627.2839475395182, 0.0, 295.0153571445745, 0.0, 630.6046803340988, 237.10098847214766, 0.0, 0.0, 1.0;

  ArTracker::parameters *params = ArTracker::create_parameters();

  params->dist = dist;
  params->mtx = mtx;

  ArTracker::Tracker tracker(params);

  while (ros::ok()) {
    if (new_frame) {
      tracker.run_frame(frame);
      tracker.broadcast_tree();
      int x = waitKey(1);
      imshow("frame", frame);
      new_frame = false;
    }

    ros::spinOnce();
  }

  return 0;
}
