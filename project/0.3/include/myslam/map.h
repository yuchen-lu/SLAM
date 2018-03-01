
#ifndef MAP_H
#define MAP_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/mappoint.h"


namespace myslam
{

class Map // manage all landmarks and add new landmarks
// delete  bad landmarks
// vo matching process only deals with Map class

{
public:
    typedef shared_ptr<Map> Ptr;
    unordered_map<unsigned long, MapPoint::Ptr> map_points_; // all landmarks
    unordered_map<unsigned long, Frame::Ptr> keyframes_; // all key-frames

    Map() {}

    void insertKeyFrame(Frame::Ptr frame);

    void insertMapPoint(MapPoint::Ptr map_point);
};
	
}


#endif