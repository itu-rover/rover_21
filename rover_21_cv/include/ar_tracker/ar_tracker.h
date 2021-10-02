#ifndef AR_TRACKER_H
#define AR_TRACKER_H

/*
        READ ME:
        Dear coder friend i strongly recommend you to read following section for
        your better understanding in rest of the code

        Rotations represented as tf::Quaternion (x,y,z,w) and will be showed as "R"
        Translation vector represented as tf::Vector3 and will be showed as "T"

        I also recommend you to take a look a bit to math of transformations in 3d, quaternions, vectors
        and some linear algebra

        naming notations:
        w - world (worlds orijin in 0 and its rotation is I)
        m - marker (artag)
        c - camera

        R_cm means markers rotation respect to the camera
        T_wm means markers translation respect to the world
        R_wc means cameras rotation respect to the world
        and ect you got the thing

        This systems uses its own parameter system.
        You will need to initialize params first before using it
        This system also uses binary tree for tracking markers faster
        These datatypes detailed in their respective header file
*/

#include <ros/ros.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <tuple>
#include <vector>

#include "params.h"
#include "slam_tree.h"

using namespace cv;
using namespace aruco;

namespace ArTracker {

// define section
#define PI 3.14159265358979323846
#define FIXED_FRAME "artracker_world"

/*
        Main funtion to run the system.
        inputs:
        - cap: videocapturer object for reading frames from camera
        (make sure cap is opened)
        - p: parameters for system

        code runs the algorithm and publishes tf. tf's fixed
        frame can be seen in "define" section you can view it
        in rviz
*/
void run(VideoCapture cap, parameters* p);

/*
        it is second main function. Instead of
        run() function this will only process
        the given frame and updates tree object
        which stores every object in world
        (look "slam_tree.h" for details of "tree")

        inputs:
        - frame: image from camera to be proceed
        - p: main parameters
        - tree: tree that will be updated
        - iter: system iter counter
*/
void run_frame(Mat frame, parameters* p, slam_tree* tree);

/*
        not used yet. will be detailed later
*/
void relocalize_marker(slam_obj* marker, slam_obj* camera, tf::Quaternion R_cm, tf::Vector3 T_cm);

/*
        function that publishes our tf
        inputs:
        - br: object that publihes tf
        - obj: slam obj that contains our tf elements
*/
void broadcast_tf(tf::TransformBroadcaster br, slam_obj* obj);

/*
        updates cameras tf based on multiple mesaurements
        inputs:
        - camera: camera that will be updated
        - qv: cameras mesaured rotations (std::vector)
        - tv: cameras mesaured translations (std::vector)
        - tresh: minimum limit of update. if amount of change between
        current and new camera transform is in bellow limit we will
        ignore new transform
*/
void update_camera_transform(slam_obj* camera, std::vector<tf::Quaternion> qv, std::vector<tf::Vector3> tv,
                             float tresh);

/*
        estimate cameras new transform with "known" marker
        inputs:
        - camere: camera object being estimated
        - marker: known marker object
        - R_cm: markers rotation relative to camera
        - T_cm: markers translation relative to camere
        outputs:
        Tuple<R, T>
        - R: cameras estimated rotation
        - T: cameras estimated trasnlation
*/
std::tuple<tf::Quaternion, tf::Vector3> estimate_cam_transform(tf::Quaternion R_cm, tf::Vector3 T_cm, slam_obj* camera,
                                                               slam_obj* marker);

/*
        when we found unknown marker we must create marker
        object and initialize its values based on estimated
        values
        inputs:
        - R_cm: markers rotation relative to camera
        - T_cm: narkers translation relative to camera
        - id:
        - camera: camera object that found the marker
        - marker: marker that found by camera
        outputs:
        - marker object

        NOTE: camera coordinate system is z: front y: down x:right
                        world coordinate system should be: x: front y: right z: up
*/
slam_obj* create_marker_obj(tf::Quaternion R_cm, tf::Vector3 T_cm, int id, slam_obj* camera);

/*
        initialize camera object with default params
        you can change those parameters in inside of
        function
        outputs:
        - camera object
*/
slam_obj* create_camera(float raw = PI / 2, float pitch = PI, float yaw = 0);

void run(VideoCapture cap, parameters* p) {
  // opencv stuff
  Mat frame;

  // ros stuff
  tf::TransformBroadcaster br;

  // tree stuff
  slam_tree tree;
  tree.add(create_camera());

  while (ros::ok()) {
    int x = waitKey(1);
    ros::spinOnce();
    tree.traverse(br, broadcast_tf);
    cap.read(frame);

    // main function that proceeds only once
    run_frame(frame, p, &tree);

    imshow("frame", frame);
  }
}

void run_frame(Mat frame, parameters* p, slam_tree* tree) {
  // aruco stoff
  std::vector<std::vector<Point2f>> corners;
  std::vector<int> ids;
  std::vector<Vec3d> rvecs, tvecs;

  detectMarkers(frame, p->aruco_dictionary, corners, ids, p->aruco_params);

  if (ids.size() == 0) return;

  slam_obj* cam = tree->search_id(-1);

  estimatePoseSingleMarkers(corners, 0.2, p->mtx, p->dist, rvecs, tvecs);
  drawDetectedMarkers(frame, corners);

  std::vector<tf::Quaternion> cam_orientations;
  std::vector<tf::Vector3> cam_locations;

  for (int i = 0, n = ids.size(); i < n; i++) {
    int id = ids[i];
    Vec3d r = rvecs[i];
    Vec3d t = tvecs[i];

    drawAxis(frame, p->mtx, p->dist, r, t, 0.1);

    tf::Vector3 T_cm;
    tf::Quaternion R_cm;
    T_cm = tf::Vector3(t[0], t[1], t[2]);
    R_cm.setRPY(r[0], r[1], r[2]);

    slam_obj* marker = tree->search_id(id);

    // If marker is not exist in map(slam system) we will add it
    // otherwise estimate cam transform with it
    // Note: Markers are assumed to be static
    if (marker == NULL) {
      marker = create_marker_obj(R_cm, T_cm, id, cam);
      tree->add(marker);
    } else {
      tf::Quaternion R;
      tf::Vector3 T;

      std::tie(R, T) = estimate_cam_transform(R_cm, T_cm, cam, marker);

      cam_orientations.push_back(R);
      cam_locations.push_back(T);
    }
  }

  // Update camera
  update_camera_transform(cam, cam_orientations, cam_locations, p->transform_confidence_tresh);
}

void relocalize_marker(slam_obj* marker, slam_obj* camera, tf::Quaternion R_cm, tf::Vector3 T_cm) {
  tf::Quaternion R_wc = camera->R;
  tf::Quaternion R_wm = R_wc * R_cm;

  tf::Vector3 T_wc = camera->T;
  tf::Vector3 T_wm = T_wc + (tf::Matrix3x3(R_wc) * T_cm);

  marker->T = T_wm;
  marker->R = R_wm;
}

slam_obj* create_marker_obj(tf::Quaternion R_cm, tf::Vector3 T_cm, int id, slam_obj* camera) {
  tf::Quaternion R_wc = camera->R;
  tf::Quaternion R_wm = R_wc * R_cm;

  tf::Vector3 T_wc = camera->T;
  tf::Vector3 T_wm = T_wc + (tf::Matrix3x3(R_wc) * T_cm);

  slam_obj* marker = new slam_obj;

  marker->id = id;
  // marker->relocalization_cooldown_counter = RELOCALIZATION_COOLDOWN;
  marker->name = "id=" + std::to_string(id);
  marker->R = R_wm;
  marker->T = T_wm;
  marker->left = NULL;
  marker->right = NULL;

  return marker;
}

std::tuple<tf::Quaternion, tf::Vector3> estimate_cam_transform(tf::Quaternion R_cm, tf::Vector3 T_cm, slam_obj* camera,
                                                               slam_obj* marker) {
  tf::Vector3 T_wm = marker->T, T_wc = camera->T;
  tf::Quaternion R_wm = marker->R, R_wc = camera->R;

  T_wc = T_wm - (tf::Matrix3x3(R_wc) * T_cm);
  R_wc = R_wm * R_cm.inverse();

  return std::make_tuple(R_wc, T_wc);
}

slam_obj* create_camera(float raw, float pitch, float yaw) {
  slam_obj* camera = new slam_obj;
  // we assumed that all markers have positive id.
  // hence with -1 id, camera can be in tree while
  // can be distinguished from markers
  camera->id = -1;
  camera->name = "c922";
  // the initial rotation and position of the camera can be changed
  // but not recommended
  camera->R.setRPY(raw, pitch, yaw);
  camera->T = tf::Vector3(0, 0, 0);
  camera->left = NULL;
  camera->right = NULL;
  // Wont be used with camera but lets initialize
  // for not getting error
  // camera->relocalization_cooldown_counter = RELOCALIZATION_COOLDOWN;

  return camera;
}

void broadcast_tf(tf::TransformBroadcaster br, slam_obj* obj) {
  tf::Transform transform;
  transform.setOrigin(obj->T);
  transform.setRotation(obj->R);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), FIXED_FRAME, obj->name));
}

void update_camera_transform(slam_obj* camera, std::vector<tf::Quaternion> qv, std::vector<tf::Vector3> tv,
                             float tresh) {
  tf::Quaternion R = camera->R;
  tf::Vector3 T = camera->T;

  int n = qv.size();

  if (n == 0)
    return;

  else if (n == 1) {
    camera->R = qv[0];
    camera->T = tv[0];
    return;
  }

  float minDot = R.dot(qv[0]);
  float minDis = T.distance(tv[0]);

  tf::Quaternion qMin = qv[0];
  tf::Vector3 tMin = tv[0];

  for (auto q : qv) {
    float dot = R.dot(q);

    if (dot < minDot) {
      minDot = dot;
      qMin = q;
    }
  }

  for (auto t : tv) {
    float dis = T.distance(t);

    if (dis < minDis) {
      minDis = dis;
      tMin = t;
    }
  }

  float dot = qMin.dot(camera->R);
  float dis = tMin.distance(camera->T);

  if (dot < tresh && dis < tresh) {
    camera->R = qMin;
    camera->T = tMin;
  }
}

}  // namespace ArTracker

#endif
