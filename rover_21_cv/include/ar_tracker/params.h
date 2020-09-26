#ifndef PARAMS_H
#define PARAMS_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace cv;
using namespace aruco;

namespace ArTracker
{

/*
	parameter struct for all adjustable params of 
	the system

	- aruco_params: parameters for aruco detection that made by aruco libary
	- aruco_dictionary: it contains all marker codes that we want to detect
	it is built-in data type in aruco libary just like aruco_parameters

	You can look to https://docs.opencv.org/master/d4/d17/namespacecv_1_1aruco.html 
	for all references of aruco

	mtx: 3x3 camera matrix
	dist: cameras distortion coefficents
	marker_size: artag size in meters. (URC config is 20 cm)

	default values of dist and mtx are made for logitech c922
	hence I recommend to change values for different camera 
	sensors 
*/
struct parameters
{
	Ptr<DetectorParameters> aruco_params;
	Ptr<Dictionary> aruco_dictionary;
	float marker_size;
	Mat dist;
	Mat mtx;
};

/*
	create parameters of the system
	and initialize default values
	you can change values of parameters 
	after initializiation
*/
parameters *create_parameters()
{
	parameters *p = new parameters;

	p->marker_size = 0.2;
	p->aruco_params = DetectorParameters::create();
	p->aruco_dictionary = getPredefinedDictionary(DICT_5X5_250);
	
	Mat_<double> mtx(3,3);
	Mat_<double> dist(1,5);
	dist << 0.020436355102596344, -0.11407839179793304, 0.004229887050454093, -0.01709654130034178, 0.13991605472148272;
	mtx << 627.2839475395182, 0.0, 295.0153571445745,
		   0.0, 630.6046803340988, 237.10098847214766,
		   0.0, 0.0, 1.0;

	p->dist = dist;
	p->mtx = mtx;

	return p;
}  	

}

#endif