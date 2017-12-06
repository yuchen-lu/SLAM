#include<iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
using namespace std;
using namespace cv;

//  using pose to get features points' 3d position using triangu
// calling out opencv triangulation



void triangulation (
  const vector<KeyPoint>& KeyPoint_1,
  const vector<KeyPoint>& KeyPoint_2,
  const std::vector<Dmatch>& matches,
  const Mat& R, const Mat&t,
  vector<Point3d>& points
  
  
);

void triangulation(const vector< KeyPoint >& KeyPoint_1, const vector< KeyPoint >& KeyPoint_2, const std::vector< std::allocator >& matches, const Mat& R, const Mat& t, vector< Point3d >& points)
{

    Mat T1 = (Mat_<double>(3,4) <<
    1,0,0,0,
    0,1,0,0,
    0,0,1,0);
    
    Mat T2 =(Mat_<double> (3,4)<<
      R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),t.at<double>(0,0),
      R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
      R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0)
	    );
  
  Mat K = (Mat_<double>(3,3)<<520.9,0,325.1,0,521.0,249.7,0,0,1);

  vector<Point2d> pts_1, pts_2;
  
  for(Dmatch m:matches)
  {
    // transfer pixel coods to camera coods
    pts_1.push_back( pixel2cam(KeyPoint_1[m.queryIdx].pt, K));
    pts_2.push_back( pixel2cam(KeyPoint_2[m.trainIdx].pt, K));
    
    
  }
    
    Mat pts_4d;
    cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);
    
    // transfer to non-homo coods
    
    for (int i=0; i<pts_4d.cols; i++)
    {
      Mat x = pts_4d.col(i);
      x /= x.at<float>(3,0); // unitlize    //x=x/....
      Point3d p(
	x.at<float>(0,0),x.at<float>(1,0),x.at<float>(2,0)
	);
      points.push_back(p);
    }
  
    
int main(int argc, char** argv)
{
  //...
  //trangulize..
  vector<Point3d> points;
  triangulation(KeyPoint_1,KeyPoint_2, matches, R,t,points);
  
  // -- verify projection relation between triangulized and feature points
  Mat K =(Mat_<double>(3,3)<< 520.9, 0, 325.1, 0, 521.0, 249.7, 0,0,1);
  for (int i=0; i<matches.size();i++)
  {
    points2d pt1_cam = pixel2cam(KeyPoint_1 [matches[i].queryIdx].pt,K);
    Point2d pt1_cam_3d(
      points[i].x/points[i].z, points[i].y/points[i].z
      
      
    );
    
    cout<<"point in the first camera frame:"<<pt1_cam<<endl;
    cout<<"point projected from 3d ""<<pt1_cam_3d"<<", d ="<<points[i].z+t;
    pt2_trans /=pt2_trans.at<double>(2,0);
    cout<<"point in the second camera frame"<<pt2_cam<<endl;
    cout<<"point reprojected from second frame"<<pt2_trans.t()<<endl;
    cout<<endl;
    
    
  }
  
  
}
    
    
}

  





int main(int argc, char **argv) {
    std::cout << "Hello, world!" << std::endl;
    return 0;
}
