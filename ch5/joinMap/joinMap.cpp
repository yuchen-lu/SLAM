

// overview: known five rgb-d imgs and camera intrinstics and extrinsics
// using imgs and camera intrins ----> any pixel location in camera coords
// using extrins(camera pose), find pixel in world coords
// list all pixels' world coords, create sth like a map




#include<iostream>
#include<fstream>

#include <opencv2/core/core.hpp>
#include<eigen3/Eigen/Geometry>
#include<opencv2/highgui/highgui.hpp>
#include<boost/format.hpp> // for formating strings
#include<pcl-1.7/pcl/point_types.h>
#include<pcl-1.7/pcl/io/pcd_io.h>
#include<pcl-1.7/pcl/visualization/pcl_visualizer.h>


using namespace std;


int main( int argc, char** argv)
{
  
  // --------highlight: see how to call vector and its type!
  
  vector<cv::Mat> colorImgs, depthImgs; // color and depthImgs
  vector<Eigen::Isometry3d,Eigen::aligned_allocator<Eigen::Isometry3d>> poses; // camera poses
  
  ifstream fin("/home/yuchen/SLAMbook/ch5/joinMap/build/pose.txt");
  if(!fin)
  {
    cerr<<"please run this file in the folder with pose.txt"<<endl;
    return 1;
  }
  
  
  
  //----highlight: how to process multiple imgs
  for ( int i=0; i<5; i++)
  {
    
    boost::format fmt( "./%s/%d.%s");// image file format
    
    
    colorImgs.push_back( cv::imread( (fmt%"color"%(i+1)%"png").str() ));
    depthImgs.push_back( cv::imread( (fmt%"depth"%(i+1)%"pgm").str(), -1 )); // use -1 to read original image
    
 // transform pose from Quaternion and translation vector to isometry matrix   
     double data[7]={0};
     for ( auto& d:data )
	fin>>d;
     Eigen::Quaterniond q( data[6], data[3], data[4], data[5]);
     Eigen::Isometry3d T(q);
     T.pretranslate( Eigen::Vector3d( data[0], data[1], data[2]));
     poses.push_back( T);

   
  }
    //compute point cloud 
    //camera intrin
    double cx = 325.5;
    double cy = 253.5;
    double fx =518.0;
    double fy =519.0;
    double depthScale = 1000.0;
    
    cout<<"transforming image to point cloud zzZZZ.."<<endl;
    
    //define pcl format: XYZRGB here
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    
    //create a new PointCloud
    PointCloud::Ptr pointCloud (new PointCloud);
    for ( int i=0; i<5; i++)
    {
      cout<<"transforming image zzzZZZ.."<<i+1<<endl;
      cv::Mat color = colorImgs[i];
      cv::Mat depth = depthImgs[i];
      Eigen::Isometry3d T =poses[i];
      for ( int v=0; v<color.rows; v++)
	for ( int u=0; u<color.cols; u++)
	{
	  unsigned int d = depth.ptr<unsigned short> (v)[u];// depth value
	  if ( d==0) continue ; // if 0, didnt detect
	
	
// calculate positions in camera coods of pixels with (u,v) and depth d	
// then transform to world coods using extrins(pose)

	Eigen::Vector3d point;
	  point[2] = double(d)/depthScale; 
	  point[0] = (u-cx)*point[2]/fx; // x coods in camera
	  point[1] = (v-cy)*point[2]/fy; // y coods in camera
 // why??
	  Eigen::Vector3d pointWorld = T*point;
	  
	  PointT p;
	  p.x = pointWorld[0];
	  p.y = pointWorld[1];
	  p.z = pointWorld[2];
	  p.b = color.data[v*color.step+u*color.channels()];
	  p.g = color.data[v*color.step+u*color.channels()+1];
	  p.r = color.data[v*color.step+u*color.channels()+2];
	  pointCloud->points.push_back(p);
	 
	  
	  
	}
    }
      
      
      
    
    pointCloud->is_dense = false;
    cout<< "point cloud has" <<pointCloud->size() <<" points"<<endl;
    pcl::io::savePCDFileBinary("map.pcd",*pointCloud);
    return 0;

  
}