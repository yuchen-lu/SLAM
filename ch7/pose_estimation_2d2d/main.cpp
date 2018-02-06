#include<iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>



using namespace std;
using namespace cv;

// summary: use 2D-2D matched feature pts to compute E,F and ; then decompose e to R,t

// feature points from before
void find_feature_matches (
  const Mat& img_1, const Mat& img_2,
  std::vector<KeyPoint>& keypoints_1_,
  std::vector<KeyPoint>& keypoints_2_,
  std::vector< DMatch >& matches )

//    //-- 初始化
//    Mat descriptors_1, descriptors_2;
//    // used in OpenCV3
//    Ptr<FeatureDetector> detector = ORB::create();
//    Ptr<DescriptorExtractor> descriptor = ORB::create();
//    // use this if you are in OpenCV2
//    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
//    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
//    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
//    //-- 第一步:检测 Oriented FAST 角点位置
//    detector->detect ( img_1,keypoints_1 );
//    detector->detect ( img_2,keypoints_2 );
//
//    //-- 第二步:根据角点位置计算 BRIEF 描述子
//    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
//    descriptor->compute ( img_2, keypoints_2, descriptors_2 );
//
//    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
//    vector<DMatch> match;
//    //BFMatcher matcher ( NORM_HAMMING );
//    matcher->match ( descriptors_1, descriptors_2, match );
//
//    //-- 第四步:匹配点对筛选
//    double min_dist=10000, max_dist=0;
//
//    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
//    for ( int i = 0; i < descriptors_1.rows; i++ )
//    {
//        double dist = match[i].distance;
//        if ( dist < min_dist ) min_dist = dist;
//        if ( dist > max_dist ) max_dist = dist;
//    }
//
//    printf ( "-- Max dist : %f \n", max_dist );
//    printf ( "-- Min dist : %f \n", min_dist );
//
//    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
//    for ( int i = 0; i < descriptors_1.rows; i++ )
//    {
//        if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
//        {
//            matches.push_back ( match[i] );
//        }
//    }
//}
{

    // initilization

    Mat descriptors_1, descriptors_2;  // descriptors are mat!
    Ptr<ORB> orb = ORB::create(500, 1.2f, 8, 31, 0, 2, ORB::HARRIS_SCORE, 31, 20); //DEFAULT

    // step 1: detect oriented fast corner position
    orb->detect(img_1, keypoints_1_);
    orb->detect(img_2, keypoints_2_);

    //step 2: compute descriptors
    orb->compute(img_1, keypoints_1_, descriptors_1);
    orb->compute(img_2, keypoints_2_, descriptors_2);

    // step 3: associate descriptors for two imgs, use brief Hamming dist
    vector<DMatch> matches_m;
    BFMatcher matcher(NORM_HAMMING); //brute-force descriptor matcher
    matcher.match(descriptors_1, descriptors_2, matches_m);

    // step 4: filter match points
    double min_dist = 10000, max_dist = 0;

    // find min and max disctance for all matches
    // that is distance for two sets of points that are most similar and least similar
    for (int i = 0; i < descriptors_1.rows; i++) {
        double dist = matches_m[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

    printf("--max dist: %f \n", max_dist);
    printf("--min dist: %f \n", min_dist);

    // when distance between descriptors > two times min, then error
    // but sometimes min is very small, so we set a bottom limit by experience

    //std::vector<DMatch> good_matches;
    for (int i = 0; i < descriptors_1.rows; i++) {
        if (matches_m[i].distance <= max(2 * min_dist, 30.0));  // again, by experience, dis < 2*min hamming dis

        {
            matches.push_back(matches_m[i]);
        }

    }

}


//********end of find_feature_matches ****output :R,t

void pose_estimation_2d2d(
std::vector<KeyPoint> keypoints_1,
std::vector<KeyPoint> keypoints_2,
std::vector<DMatch> matches,
Mat& R, Mat& t)
{
  // camera intrinsics, TUM Freiburg2
  
  //--------highlight: how to give value to cv Mat
  Mat K = (Mat_<double> (3,3) <<520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);


  // transform matched feature pts to the form of vector<Point2f>
  vector<Point2f> points1;
  vector<Point2f> points2;
  for (int i=0; i<(int)matches.size();i++)
  {
    points1.push_back(keypoints_1[matches[i].queryIdx].pt); // query descriptor index.xycoods
    points2.push_back(keypoints_2[matches[i].trainIdx].pt);
  }
  
  // compute fundamentental matrix

  Mat fundamental_matrix;
  fundamental_matrix = findFundamentalMat(points1, points2, CV_FM_8POINT);  // 8 point method
  cout<<"funamental_matrix is "<<endl<<fundamental_matrix<<endl;
  
  // compute essential matrix E
  Point2d principal_point(325.1, 249.7); // optical centre, TUM dataset calibration value
    double focal_length = 521; // focal , tum dataset calibration
  Mat essential_matrix = findEssentialMat(points1,points2,focal_length,principal_point,RANSAC);
  cout<<"essential matrix is "<<endl<<essential_matrix<<endl;
  
  // compute homography matrix H
  Mat homography_matrix;
  homography_matrix = findHomography(points1,points2,RANSAC,3);
  cout<<"homography matrix is"<<endl<<homography_matrix<<endl;
  
  
  
  // recover rotation and translation
  recoverPose(essential_matrix,points1,points2,R,t,focal_length,principal_point);
  cout<<"R is "<<R<<endl;
  cout<<"T is "<<t<<endl;
  
}
//********end of pose_estimation_2d2d ****output :R,t


// pixel coords to normalized z=1 camera coods
// ??????
Point2d pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2d( (p.x - K.at<double> (0,2))/K.at<double>(0,0),    (p.y - K.at<double> (1,2))/K.at<double>(1,1) );
}



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
  find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
  cout<<"found total number of "<<matches.size()<<"matched feature points";

    //   estimate R,t
  Mat R,t;
  pose_estimation_2d2d(keypoints_1,keypoints_2,matches,R,t);
  
  // confirm  E=t^R*scale

  Mat t_hat = (Mat_<double>(3,3)<<
    0,                    -t.at<double>(2,0),           t.at<double>(1,0),
    t.at<double>(2,0),    0,                            -t.at<double>(0,0),
    -t.at<double>(1,0),   t.at<double>(0,0),            0);
   
  cout<<"t^R="<<endl<<t_hat*R<<endl;
  
  // check epipolar constraints
  Mat K =(Mat_<double> (3,3)<<520.9,    0,  325.1,  0,  521.0,  249.7,  0,  0,  1);
  for ( DMatch m: matches )
  {
    Point2d pt1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
    Mat y1 = (Mat_<double>(3,1)<<pt1.x, pt1.y,  1);
    Point2d pt2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
    Mat y2 = (Mat_<double>(3,1)<<pt2.x, pt2.y,   1);
    Mat d = y2.t() * t_hat * R * y1;
    cout<<"epipolar constraints="<<d<<endl;
  }
  
  return 0;

}
   





