#ifndef FRAME_H
#define FRAME_H

#include "myslam/common_include.h"


// store: id, time_stamp, pose, color&depth imgs
// fnc: createFrame, find depth for pointm find camera optical centre, check if one point is in range

class Frame {
public:
	typedef std::shared_ptr<Frame> Ptr;
	unsigned long id_; // id of this frame
	double time_stamp_; // when it is recorded
	SE3 T_c_w_; // pose
	Camera::Ptr camera_; // pinhole rgb-d camera model
	Mat color_, depth_; //color and depth image

public:        //data members
	Frame();

	Frame(long id, double time_stamp_ = 0, Sophus::SE3 T_c_w_ = SE3(), Camera::Ptr camera = nullptr,
		  Mat color = Mat(),
		  Mat depth = Mat());  // see how the initial value are defined by different variable names! (with/ without understroke)
	~Frame();

	//factory function
	static Frame::Ptr createFrame();

	//find the depth in depth map
	double findDepth(const cv::KeyPoint &kp);

	// get camera centre
	Vector3d getCamCenter() const;

	// check if a point is in this frame

	bool isInFrame(const Vector3d &pt_ world);
};





