#include<iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
using namespace std;
using namespace cv;

//  using pose to get features points' 3d position using triangu
// calling out opencv triangulation
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
    int focal_length = 521; // focal , tum dataset calibration
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


Point2d pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2d( (p.x - K.at<double> (0,2))/K.at<double>(0,0),    (p.y - K.at<double> (1,2))/K.at<double>(1,1) );
}

void find_feature_matches (
        const Mat& img_1, const Mat& img_2,
        std::vector<KeyPoint>& keypoints_1_,
        std::vector<KeyPoint>& keypoints_2_,
        std::vector< DMatch >& matches )
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
//***********end of find feature matches********



void triangulation (
  const vector<KeyPoint>& KeyPoint_1,
  const vector<KeyPoint>& KeyPoint_2,
  const std::vector<DMatch>& matches,
  const Mat& R, const Mat&t,
  vector<Point3d>& points)
{

    Mat T1 = (Mat_<double>(3, 4) <<
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0);  //projMatr1 3x4 projection matrix of the first camera

    Mat T2 = (Mat_<double>(3, 4) <<
            R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0)
    ); //projMatr2 3x4 projection matrix of the second camera.

    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    vector<Point2d> pts_1, pts_2;  //2xN array of feature points in the first & second image

    for (DMatch m:matches) {
        // transfer pixel coods to camera coods
        pts_1.push_back(pixel2cam(KeyPoint_1[m.queryIdx].pt, K));
        pts_2.push_back(pixel2cam(KeyPoint_2[m.trainIdx].pt, K));
    }

    Mat pts_4d;
    cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);


    // transfer to non-homo coods
    for (int i = 0; i < pts_4d.cols; i++) {
        Mat x = pts_4d.col(i);
        x /= x.at<float>(3, 0); // normalization
        Point3d p(  x.at<float>(0, 0), x.at<float>(1, 0), x.at<float>(2, 0) );
        points.push_back(p);
    }

}


// **********end of function triangulation***********



int main(int argc, char** argv)
{
    if (argc != 3)
    {
      cout<<"usage: triangulation img1 img2"<<endl;
      return 1;
    }

    Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[1], CV_LOAD_IMAGE_COLOR);

    vector<KeyPoint>KeyPoint_1, KeyPoint_2;
    vector<DMatch> matches;
    find_feature_matches(img_1, img_2, KeyPoint_1, KeyPoint_2, matches);
    cout<<"total number of "<<matches.size()<<"groups of features found! not bad huh?"<<endl;

    // estimate R, t
    Mat R,t;
    pose_estimation_2d2d(KeyPoint_1, KeyPoint_2, matches, R,t);

    // triangulation
    vector<Point3d> points;
    triangulation(KeyPoint_1,KeyPoint_2, matches, R,t,points);

    // -- verify reprojection relation between triangulized and feature points
    Mat K =(Mat_<double>(3,3)<< 520.9, 0, 325.1, 0, 521.0, 249.7, 0,0,1);

    for (int i=0; i<matches.size();i++)
    {
    Point2d pt1_cam = pixel2cam(KeyPoint_1 [matches[i].queryIdx].pt,K);
    Point2d pt1_cam_3d(
      points[i].x/points[i].z,
      points[i].y/points[i].z
    );

    cout<<"point in the first camera frame:"<<pt1_cam<<endl;
    cout<<"point projected from 3d ""<<pt1_cam_3d"<<", d ="<<points[i].z+t;

    // second picture
    Point2f pt2_cam = pixel2cam(KeyPoint_2[ matches[i].trainIdx].pt, K);
    Mat pt2_trans = R*( Mat_<double>(3,1) <<points[i].x, points[i].y, points[i].z)+t;
    pt2_trans /=pt2_trans.at<double>(2,0);
    cout<<"point in the second camera frame"<<pt2_cam<<endl;
    cout<<"point reprojected from second frame"<<pt2_trans.t()<<endl;
    cout<<endl;


    }
    return 0;
}
    
    




