#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

// define the commonly included file to avoid a long include list


//eigen
#include<eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include<
using Eigen::Vector2d;
using Eigen::Vector3d;

//for sophus
#include<sophus/se3.h>
#include <sophus/so3.h>
using Sophus::SE3;
using Sophus::SO3;


// for opencv
#include <opencv2/core/core.hpp>
using cv::Mat;


//std
#include<vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>

using namespace std;
#endif



