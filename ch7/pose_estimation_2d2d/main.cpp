#include<iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

// summary: use matched feature pts to compute E,F and H
// 	    then decompose e to R,t


  void pose_estimation_2d2d(
    std::vector<KeyPoint> keypoints_1,
    std::vector<KeyPoint> keypoints_2,
    std::vector<DMatch> matches,
    Mat& R, Mat& t)
  
{  

  // camera intrinsics, TUM Freiburg2
  Mat K = (Mat_<double>(3,3)<<520.9, 325.1,0,521.0,249.7,0,0,1);

  // transform matched feature pts to the form of vector<Point2f>
  vector<Point2f> points1;
  vector<Point2f> points2;

  for (int i=0; i<(int)matches.size();i++)
  {
    points1.push_back(keypoints_1[matches[i].queryIdx].pt);
    points2.push_back(keypoints_2[matches[i].trainIdx].pt);
  }
  
  // compute fundamentental matrix
  Mat fundamental_matrix;
  fundamental_matrix = findFundamentalMat(points1, points2, CV_FM_8POINT);
  cout<<"funamental_matrix is "<<endl<<fundamental_matrix<<endl;
  
  // compute essential matrix
  Point2d principal_point(325.1, 249.7); // optical centre, TUM dataset calibration vallue
  int focal_length =521; // focal , tum dataset calibration
  Mat essential_matrix;
  essential_matrix = findEssentialMat(points1,points2,focal_length,principal_point,RANSAC);
  cout<<"essential matrix is "<<endl<<essential_matrix<<endl;
  
  // compute homography matrix
  Mat homography_matrix;
  homography_matrix = findHomography(points1,points2,RANSAC,3,noArray(),2000,0.99);
  cout<<"homography matrix is"<<endl<<homography_matrix<<endl;
  
  
  
  // recover rotation and translation
  recoverPose(essential_matrix,points1,points2,R,t,focal_length,principal_point);
  cout<<"R is "<<R<<endl;
  cout<<"T is "<<t<<endl;
  
}



// above is a function, needs to be called in main cpp

int main(int argc, char**argv)
{
  if (argc!=3)
  {
    cout<<"usage:feature_extration img1 img2"<<endl;
    return 1;
    
  }
  
  //read imgs
  Mat img_1 =imread(argv[1],CV_LOAD_IMAGE_COLOR);
  Mat img_2 =imread(argv[2],CV_LOAD_IMAGE_COLOR);
  std::vector<KeyPoint> keypoints_1, keypoints_2;
  vector<DMatch> matches;
  find_feature_matches(img_1,img_2,keypoints_1,keypoints_2,matches);
  cout<<"found total number of "<<matches.size()<<"matched feature points"
  
  Mat R,t;
  pose_estimation_2d2d(keypoints_1,keypoints_2,matches,R,t);
  
  // check E=t^R*scale
  Mat t-x = (Mat_<double>(3,3))<<
    0,-t.at<double>(2,0),0,-t.at<double>(0,0),
    -t.at<double>(1,0),t.at<double>(0,0),0);
   
  cout<<"t^R="<<endl<<t_x*R<<end;
  
  // check epipolar constraints
  Mat K =(Mat_<double> (3,3)<<520.9,0,325.1,0,521.0,349.7,0,0,1);
  for (DMatch ,; matches)
  {
    Point2d pt1 = pixel2cam(keypoints_1[ m.queryIdx].pt,K);
    Mat y1 = (Mat_<double>(3,1)<<pt1.x,pt1.y,1);
    Point2d pt2 = pixel2cam(keypoints_2[m.trainIdx].pt,K);
    Mat y2 = (Mat_<double>(3,1))<<pt2.x,pt2.y,1);
    Mar d = y2.t()*t_x*R*y1;
    cout<<"epipolar constraints="<<d<<endl;
  }
  
  return 0;
    
    
    
    
}
   







