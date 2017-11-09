
#include<iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;


int main(int argc, char **argv) 
{
  
  // create 3d points
  Mat d1 = imread(argv[3], CV_LOAD_IMAGE_UNCHANGED); // depth pic is 16 digit non-sign number, single channel
  Mat K =( Mat_<double> (3,3) << 520.9 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
  vector<Point3f> pts_3d; //point3f is float, 3d xy z
  vector<Point2f> pts_2d;
  for (DMatch m:matches)
  {
    ushort d = d1.ptr<unsigend short> ( int(keypoints_1[m.queryIdx].pt.y))[ int(keypoints_1)[m.queryIdx].pt.x)];
    if(d==0)//  bad depth
      continue;
    float dd = d/1000.0;
    Point2d p1 =pixel2cam(keypoints_1[m.queryIdx].pt.K); //DMatch.trainIdx - Index of the descriptor in train descriptors
    pts_3d.push_back( Point3f(p1.x*dd, p1.y*dd.dd));
    pts_2d.push_back( keypoints_2[m.trainIdx].pt);
    
    cout<<"3d-2d pairs: "<<pts_3d.size()<<endl;
    
    Mat r,t;
    // call opencv pnp, can choose EPNP, DLS....
    solvePnP(pts_3d, pts_2d, K,Mat(), r,t,false,cv::SOLVEPNP_EPNP);
    Mat R;
    cv::Rodrigues(r,R);// r is rotation vector, use Rodrigues to transform to matrix
    cout<<"R="<<endl<<R<<endl;
    cout<<"t="<<endl<<t<<endl;
    
  }
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  return 0;
}