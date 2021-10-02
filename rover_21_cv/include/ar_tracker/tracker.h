#ifndef TRACKER_H
#define TRACKER_H

#include "ar_tracker.h"

namespace ArTracker {

/*
        Interface class between libary and user
        it is created for make usage easier
*/
class Tracker {
 public:
  Tracker();
  Tracker(parameters* p);

  void run(VideoCapture cap);
  void run_frame(Mat frame, bool global_tracking = true);
  void broadcast_tree();
  bool getArtagTransform(int, tf::Vector3, tf::Quaternion);

  /*
          returns pointer of tree
  */
  slam_tree* get_tree();

 private:
  tf::TransformBroadcaster br;
  parameters* params;
  slam_tree tree;
};

/*
Tracker::Tracker()
{
        parameters *p = create_parameters();
        Tracker(p);
}
*/

Tracker::Tracker(parameters* p) {
  params = p;
  tree.add(create_camera());
}

void Tracker::run(VideoCapture cap) { ArTracker::run(cap, params); }

/*
 global traking: if it is true we will store marker locations
                                 track even there are not in sight
                                 to accomplish this we will reset the tree before
                                 running the system
*/
void Tracker::run_frame(Mat frame, bool global_tracking) {
  if (!global_tracking) {
    // tree.reset(); //this gives seg error
  }

  ArTracker::run_frame(frame, params, &tree);
  broadcast_tree();
}

void Tracker::broadcast_tree() { tree.traverse(br, ArTracker::broadcast_tf); }

slam_tree* Tracker::get_tree() { return &tree; }

bool Tracker::getArtagTransform(int id, tf::Vector3 T, tf::Quaternion R) {
  slam_obj* artag = tree.search_id(id);

  if (artag == NULL) return false;

  T = artag->T;
  R = artag->R;
  return true;
}

}  // namespace ArTracker

#endif
