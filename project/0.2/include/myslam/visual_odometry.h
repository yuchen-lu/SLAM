#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "myslam/map.h"
#include "myslam/common_include.h"


#include <opencv2/features2d/features2d.hpp>


namespace myslam
{

    class VisualOdometry

// about vo class:
// VO itself has several status: setting 1st frame; successfullt tracked; lost
// can see it as Finite State Machine(FSM)
// in this application, only three simply: inilitilization; normal; lost
// we add middle variables' definition in the class, so that we can omit complex params pass;
// i.e. since variables are defined in class, can be accessed anywhere in the function
    {
    public:
        typedef shared_ptr<VisualOdometry> Ptr;
        enum VOState
        {
            INITIALIZING = 1,
            OK = 0,
            LOST
        };

        VOState state_; // current VO status

        Map::Ptr map_; //map with all frames and map points
        Frame::Ptr ref_; //reference frame
        Frame::Ptr curr_; // current frame
        cv::Ptr<cv::ORB> orb_; // orb detctor and computer
        vector<cv::Point3f> pts_3d_ref_; //3d points in ref frame
        vector<cv::KeyPoint> keypoints_curr_; // KeyPoints in current frame
        Mat descriptors_curr_; //descriptors in current frame
        Mat descriptors_ref_; //descriptors in ref frame
        vector<cv::DMatch> feature_matches_;

        SE3 T_c_r_estimated_; // estimated pose of current frame to ref frame
        int num_inliers_; // number of inlier features in icp
        int num_lost_; // number of lost times

        //params
        int num_of_features_;
        double scale_factor_; // in image pyramid
        int level_pyramid_;
        float match_ratio_;  // ratio for selecting good matches
        int max_num_lost_; // max num of continuous lost times
        int min_inliers_;

        double key_frame_min_rot; //min rotation of two key-frames
        double key_frame_min_trans;

    public:
        VisualOdometry();

        ~VisualOdometry();

        bool addFrame(Frame::Ptr frame); // add a new frame

    protected:
        // inner operation
        void extractKeyPoints();

        void computeDescriptors();

        void featureMatching();

        void poseEstimationPnP();

        void setRef3DPoints();

        void addKeyFrame();

        bool checkEstimatedPose();

        bool checkKeyFrame();

    };

}


#endif