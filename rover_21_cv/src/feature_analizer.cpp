#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

#define cornerTresh 32

using namespace cv;


int main()
{
	VideoCapture cap;
	cap.open("/home/basestation/rover20_ws/src/input.mp4");

	if(!cap.isOpened())
	{
		std::cout << "Video Acilamadi... Kapatiliyor\n";
		return 1;
	}

	Mat frame, gray;

	while(1)
	{
		if(!cap.read(frame))
		{
			std::cout << "Video bitti veya sorun cikti \n";
			break;
		}

		std::vector<KeyPoint> features;

		cvtColor(frame, gray, COLOR_BGR2GRAY);

		FAST(gray, features, cornerTresh, true);

		drawKeypoints(frame, features, Scalar(0,0,255));

		imshow("frame", frame);

		if(waitKey(10) == 27) break;

	}

	return 0;
}