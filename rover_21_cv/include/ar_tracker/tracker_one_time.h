#ifndef TRACKER_ONE_TIME_H
#define TRACKER_ONE_TIME_H

#include "ar_tracker.h"

namespace ArTracker
{

class OneTimeTracker
{
public:
	parameters* p;

	//they store last findings of tracking.
	std::vector<int> ids;
	std::vector<tf::Vector3> Tvec;
	std::vector<tf::Quaternion> Rvec;
	std::vector<tf::Vector3> RPYvec; //vector version of rotation optional for now

	OneTimeTracker();
	void reset_vecs();
	bool get_artag_transform(int, tf::Vector3, tf::Quaternion);
	int run_frame(Mat);

};


OneTimeTracker::OneTimeTracker()
{
	p = create_parameters();
}

void OneTimeTracker::reset_vecs()
{
	RPYvec.clear();
	ids.clear();
	Tvec.clear();
	Rvec.clear();
}

/*
	fills vector and quaternin if we find 
	artag with id. Also return true if desired
	id is exists
*/
bool OneTimeTracker::get_artag_transform(int id, tf::Vector3 T, tf::Quaternion R)
{
	return true;
}

/*
	run aruco for one time. No global tracking is here 
	simple use for approaching(for now)

	input:
		-frame: input bgr image

	output:
		-int: number of tags we have found

*/
int OneTimeTracker::run_frame(Mat frame)
{
	reset_vecs();

	//aruco stuff
	std::vector<std::vector<Point2f>> corners;
	std::vector<Vec3d> rvecs, tvecs;

	detectMarkers(frame, p->aruco_dictionary, corners, ids ,p->aruco_params);

	if(ids.size() != 0)
	{
		estimatePoseSingleMarkers(corners, p->marker_size, p->mtx, p->dist, rvecs, tvecs);
		drawDetectedMarkers(frame, corners);

		for (int i = 0, n = ids.size(); i < n; i++)
		{
			Vec3d r = rvecs[i];
			Vec3d t = tvecs[i];

			drawAxis(frame, p->mtx, p->dist, r, t, 0.1);

			tf::Vector3 T = tf::Vector3(t[0],t[1],t[2]);
			tf::Vector3 RPY = tf::Vector3(r[0], r[1], r[2]);
			tf::Quaternion R;
			R.setRPY(r[0], r[1], r[2]);

			Tvec.push_back(T);
			RPYvec.push_back(RPY);
			Rvec.push_back(R);
		}
	}

	return ids.size();
}

}

#endif