#include <iostream>
#include<iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

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
    0,0,1,0;
    Mat T2 =(Mat_<double> (3,4)<<
      R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),t.at<double>(0,0),
      R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
      R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0(
	    );
  )
}

  





int main(int argc, char **argv) {
    std::cout << "Hello, world!" << std::endl;
    return 0;
}
