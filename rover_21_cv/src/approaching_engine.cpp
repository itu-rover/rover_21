#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "ar_tracker/ar_tracker.hpp"

//Berke Algül 25.10.2020

class ApproachingEngine
{
public:
	tf::TransformBroadcaster br;
	ArTracker::OneTimeTracker tracker;

	tf::Quaternion Rcam;

	void camera_cb(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    	cv::Mat frame = cv_ptr->image;
    	
    	int tags = tracker.run_frame(frame);

    	for(int i = 0; i < tags; i++)
    	{
    		int id = tracker.ids[i];
    		tf::Vector3 T = tracker.Tvec[i];
    		tf::Quaternion R = tracker.Rvec[i];

    		R = R * Rcam;
    		T = tf::Matrix3x3(Rcam) * T;
    		
    		ROS_INFO_STREAM(id<< " -> "<< R[0] << R[1] << R[2] << R[3]);

    		broadcast_tf(id, T, R);
    	}


		int x = waitKey(1);
		imshow("frame", frame);
	}

	ApproachingEngine()
	{
		ros::NodeHandle nh;
		image_transport::ImageTransport it(nh);
		std::string camera_topic;
		nh.getParam("/camera/color_topic", camera_topic);
		image_transport::Subscriber camera_sub = it.subscribe(camera_topic, 1, &ApproachingEngine::camera_cb, this);

		//TODO: change initial angles angles x front y right z up
		Rcam.setRPY(PI/2, 0, PI);
		ros::spin();
	}

	void broadcast_tf(int id, tf::Vector3 T, tf::Quaternion R)
	{
		tf::Transform transform;
		transform.setOrigin(T);
		transform.setRotation(R);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), FIXED_FRAME, std::to_string(id)));
	}
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "approaching_engine");
	ApproachingEngine ae;
	return 0;
}