
#ifndef FRAME_H
#define FRAME_H

#include "myslam/common_include.h"

class Frame
{
public:
	typedef std::shared_ptr<Frame> Ptr;
	unsigned long id_; // id of this frame
	double time_stamp_; // when it is recorded
	SE3 T_c_w_; // pose
	Camera::Ptr camera_; // pinhole rgb-d camera model
	Mat color_, depth_; //color and depth image

public:		//data members
	Frame(); //default constructor
	Frame( long id, double time_stamp_ =0, SE3 T_c_w_=SE3(), Camera::Ptr camera= nullptr, 
	       Mat color =Mat(), Mat depth = Mat()  ) ;  // see how the initial value are defined by different variable names! (with/ without understroke)
	~Frame(); //decostructor
	
	//factory function
	static Frame::Ptr createFrame();
	
	//find the depth in depth map
	double findDepth ( const cv::KeyPoint& kp);
	
	// get camera centre
	Vector3d getCamCeneter() const;
	
	// check if a point is in this frame
	
	bool isInFrame( const Vector3d& pt_ world);
	
	
};

	
public:		

class MapPoint // MapPoint is landmark, estimate world pose, 
//use feature points from prev frame to match landmarks to esti pose
// so we need to store its descriptor
// also record # of times a ppoint being Observed and being Matched
{
	typedef shared_ptr<MapPoint> Ptr;
	unsigned long id_; //id
	vector3d pos_; //world pose
	Vector3d norm_; //normal of viewing direction?
	Mat descriptor_; // descriptor for matching
	int observed_times_;//.being ovserved by deature matching algo
	int correct_times_; //being an inlinder in ppose estimation
	
	MapPoint();
	MapPoint( long id, Vector3d position, Vector3d norm);
	
	//factoring function
	static MapPoint::Ptr createMapPoint();
	
};
