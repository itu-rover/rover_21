#include <vector>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>


class CameraPub
{
public:
  	image_transport::Publisher color_pub, gray_pub;
  	ros::Publisher info_pub;
<<<<<<< HEAD

  	sensor_msgs::CameraInfo info; //we have fixed camera parameters
=======
>>>>>>> d57da7d6ca9079783623660e902043048eed3e61

	bool gray; 
	std::string name;
	cv::VideoCapture cap;
	
	CameraPub(ros::NodeHandle);
	void main();
	void pub_image();
	void pub_info();
<<<<<<< HEAD
	void initialize_info_stuff(ros::NodeHandle);
	void initialize_image_stuff(ros::NodeHandle);
=======
>>>>>>> d57da7d6ca9079783623660e902043048eed3e61
};

CameraPub::CameraPub(ros::NodeHandle nh)
{
	initialize_info_stuff(nh);
	initialize_image_stuff(nh);

	ROS_INFO("Initialization completed. we shall begin publishing");
	ROS_INFO_STREAM("Convert to gray: " << gray);

	//we shall begin publishing
	main();
}

void CameraPub::initialize_info_stuff(ros::NodeHandle nh)
{
	bool success = true;

	//creating camera info msg
	std::string distortion_model, info_topic, camera, param_address_header;
	std::vector<float> D;
	float fx, fy, cx, cy;
	int w, h;

	success = nh.getParam("camera", camera);

	param_address_header = "/"+camera+"/";	

	ROS_INFO_STREAM(param_address_header);

	success = nh.getParam("/camera/info_topic", info_topic);
	success = nh.getParam(param_address_header+"distortion_model", distortion_model);
	success = nh.getParam(param_address_header+"D", D);
	success = nh.getParam(param_address_header+"width", w);
	success = nh.getParam(param_address_header+"height", h);
	success = nh.getParam(param_address_header+"fx", fx);
	success = nh.getParam(param_address_header+"fy", fy);
	success = nh.getParam(param_address_header+"cx", cx);
	success = nh.getParam(param_address_header+"cy", cy);

	
	if(!success)
	{
		ROS_ERROR("Something went wrong with params. Terminating!!!");
		ros::shutdown();
	}

	info.K[0] = fx;
	info.K[1] = 0;
	info.K[2] = cx;
	info.K[3] = 0;
	info.K[4] = fy;
	info.K[5] = cy;
	info.K[6] = 0;
	info.K[7] = 0;
	info.K[8] = 1;


	for(int i = 0, n = D.size(); i < n; i++)
		info.D.push_back(D[i]);

	info.width = w;
	info.height = h;
	info.distortion_model = distortion_model;

	info_pub = nh.advertise<sensor_msgs::CameraInfo>(info_topic, 1);
}

void CameraPub::initialize_image_stuff(ros::NodeHandle nh)
{
	bool success = true;

	//creating camera info msg
	


	//creating image publisher

	// private params
	success = nh.getParam("gray", gray);

	// publisher topics
	std::string color_topic, gray_topic;
	success = nh.getParam("/camera/color_topic", color_topic);
	success = nh.getParam("/camera/gray_topic", gray_topic);

	int cap_id;
	nh.getParam("cap_id", cap_id);

	if(!success)
	{
		ROS_ERROR("Something went wrong with params. Terminating!!!");
		ros::shutdown();
	}

	cap.open(cap_id);

	if(!cap.isOpened())
	{
		ROS_ERROR("Camera is not opened!!!. Terminating...");
		ros::shutdown();
	}

	image_transport::ImageTransport it(nh);

	color_pub = it.advertise(color_topic, 1);
	gray_pub = it.advertise(gray_topic, 1);
<<<<<<< HEAD
=======

	ROS_INFO("Initialization completed. we shall begin publishing");
	ROS_INFO_STREAM("Convert to gray: " << gray);

	//we shall begin publishing
	main();
>>>>>>> d57da7d6ca9079783623660e902043048eed3e61
}

void CameraPub::main()
{
	ros::Rate loop_rate(30);

	while(ros::ok() && cap.isOpened())
	{
		pub_image();
		pub_info();
		
    	ros::spinOnce();
    	loop_rate.sleep();
	}
}

void CameraPub::pub_image()
{
	cv::Mat image;
	cap.read(image);

	if(gray)
	{	
		cv::Mat gray_image;
		cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray_image).toImageMsg();
		gray_pub.publish(msg);
	}

	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
	color_pub.publish(msg);
}

void CameraPub::pub_info()
{
<<<<<<< HEAD
	info_pub.publish(info);
=======

>>>>>>> d57da7d6ca9079783623660e902043048eed3e61
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "camera_node");
	ros::NodeHandle nh("~");
	CameraPub cp(nh);
	return 0;
}
