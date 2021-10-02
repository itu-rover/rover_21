#include <ros/ros.h>

#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;
using namespace aruco;

Ptr<Dictionary> GetDictionary();

Mat convertToMat(int data[5][5]);

void addBit(Ptr<Dictionary> targetDic, int bitArr[5][5]);

int main(int argc, char **argv) {
  ros::init(argc, argv, "aruco_cpp");

  Mat frame;
  vector<vector<Point2f>> corners;
  vector<int> ids;
  VideoCapture cap;
  Ptr<DetectorParameters> params = DetectorParameters::create();
  Ptr<Dictionary> dictionary = GetDictionary();  // getPredefinedDictionary(DICT_5X5_250);
  params->markerBorderBits = 2;

  cap.open(0);

  if (!cap.isOpened()) {
    cout << "Camera is not open!! Shutting down!!\n";
    return 1;
  }

  while (ros::ok()) {
    cap.read(frame);

    detectMarkers(frame, dictionary, corners, ids, params);

    if (ids.size() > 0) drawDetectedMarkers(frame, corners);
    // cout << ids << '\n';
    imshow("frame", frame);

    ros::spinOnce();
    if (waitKey(10) == 27)  // 27 == esc
      break;
  }

  return 0;
}

Ptr<Dictionary> GetDictionary() {
  Dictionary dic;
  Ptr<Dictionary> Dic = makePtr<Dictionary>(dic);

  Dic->markerSize = 5;
  Dic->maxCorrectionBits = 3;

  int data[5][5] = {{1, 1, 0, 1, 1}, {1, 1, 0, 1, 1}, {1, 0, 1, 0, 1}, {0, 1, 1, 1, 0}, {0, 1, 1, 1, 0}};
  addBit(Dic, data);

  return Dic;
}

void addBit(Ptr<Dictionary> targetDic, int bitArr[5][5]) {
  Mat bit = convertToMat(bitArr);
  Mat markerCoded = Dictionary::getByteListFromBits(bit);
  targetDic->bytesList.push_back(markerCoded);
}

Mat convertToMat(int data[5][5]) {
  Mat M(5, 5, CV_8UC1);

  for (int j = 0; j < 5; j++) {
    for (int i = 0; i < 5; i++) {
      M.at<uchar>(j, i) = data[j][i];
    }
  }

  return M;
}
