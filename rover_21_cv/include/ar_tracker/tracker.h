#include "ar_tracker.h"

namespace ArTracker
{

/*
	Interface class between libary and user
	it is created for make usage easier
*/
class Tracker
{
public:

Tracker();
Tracker(parameters *p);

void run(VideoCapture cap);
void run_frame(Mat frame);
void broadcast_tree();

/*
	returns pointer of tree
*/
slam_tree *get_tree();

private:

int iter;
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

Tracker::Tracker(parameters* p)
{
	params = p;
	iter = 0;
	tree.add(create_camera());
}

void Tracker::run(VideoCapture cap)
{	
	ArTracker::run(cap, params);
}

void Tracker::run_frame(Mat frame)
{
	ArTracker::run_frame(frame, params, &tree, iter);
}

void Tracker::broadcast_tree()
{
	tree.traverse(br, ArTracker::broadcast_tf);
}

slam_tree* Tracker::get_tree()
{
	return &tree;
}

}
