#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

/*The arguments must be between quotes:

"/your folder and path/Your file"

or

"enter your parameter here"

*/

using namespace std;
using namespace cv;

int main ( int argc, char**argv)
{
  if (argc != 3)
  {
    
    cout<<"usage: feature_extraction img1 img2"<<endl;
    return 1;
    
    
  }
  
  //read img
  Mat img_1 = imread( argv[1], CV_LOAD_IMAGE_COLOR);
  Mat img_2 = imread( argv[2], CV_LOAD_IMAGE_COLOR);
  
  // initilization
  std::vector<KeyPoint> keypoints_1, keypoints_2;
  Mat descriptors_1, descriptors_2;
  Ptr<ORB> orb = ORB::create(500,1.2f,8,31,0,2,ORB::HARRIS_SCORE,31,20);
  
  // step 1: detect oriented fast corner position
  orb->detect(img_1,keypoints_1);
  orb->detect(img_2,keypoints_2);
  
  //step 2: compute descriptors
  orb->compute(img_1,keypoints_1,descriptors_1);
  orb->compute(img_2,keypoints_2,descriptors_2);
  
  Mat outimg1;
  drawKeypoints(img_1, keypoints_1, outimg1, Scalar::all(-1),DrawMatchesFlags::DEFAULT);
  imshow("feature points ORB", outimg1);
  
  // step 3: associate descriptors for two imgs, use brief Hamming dist
  vector<DMatch> matches;
  BFMatcher matcher (NORM_HAMMING);
  matcher.match(descriptors_1,descriptors_2,matches);
  
  // step 4: filter match points
  double min_dist = 10000, max_dist =0;
  
  // find min and max disctance for all matches
  // that is distance for two sets of points that are most similar and least similar 
  for (int i=0; i<descriptors_1.rows;i++)
  {
    double dist = matches[i].distance;
    if(dist <min_dist) min_dist =dist;
    if(dist >max_dist) max_dist = dist;

  }
  
  printf("--max dist: %f \n",max_dist);
  printf("--min dist: %f \n",min_dist);
  
  // when distance between descriptors > two times min, then error
  // but sometimes min is very small, so we set a bottom limit by experience
  
  std::vector<DMatch> good_matches;
  for( int i=0; i<descriptors_1.rows;i++)
  {
    if (matches[i].distance <= max(2*min_dist,30.0));  // again, by experience, dis < 2*min hamming dis
    
    {
      good_matches.push_back(matches[i]);
      
    }
    
    
  }
  
  // step 5: plot matching result
  Mat img_match;
  Mat img_goodmatch;
  drawMatches(img_1,keypoints_1,img_2,keypoints_2,matches,img_match);
  drawMatches(img_1,keypoints_1,img_2,keypoints_2,good_matches,img_goodmatch);
  imshow("all matches points", img_match);
  imshow("optimized matched points",img_goodmatch);
  waitKey(0);

  return 0;
  
  
}