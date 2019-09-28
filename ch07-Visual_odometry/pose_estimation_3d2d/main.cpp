#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include<g2o/solvers/dense/linear_solver_dense.h>

#include <g2o/types/sba/types_six_dof_expmap.h>
#include <chrono>


using namespace std;
using namespace cv;

void find_feature_matches (
        const Mat& img_1, const Mat& img_2,
        std::vector<KeyPoint>& keypoints_1_,
        std::vector<KeyPoint>& keypoints_2_,
        std::vector< DMatch >& matches )
{

    // initilization
    Mat descriptors_1, descriptors_2;  // descriptors are mat!
    //Ptr<ORB> orb = ORB::create(); //DEFAULT
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create ("BruteForce-Hamming");


    // step 1: detect oriented fast corner position
    //    orb->detect(img_1, keypoints_1_);
    //    orb->detect(img_2, keypoints_2_);
    detector->detect(img_1, keypoints_1_);
    detector->detect(img_2, keypoints_2_);




    //step 2: compute descriptors
    //    orb->compute(img_1, keypoints_1_, descriptors_1);
    //    orb->compute(img_2, keypoints_2_, descriptors_2);
    descriptor->compute (img_1, keypoints_1_, descriptors_1);
    descriptor->compute (img_2, keypoints_2_, descriptors_2);




    // step 3: associate descriptors for two imgs, use brief Hamming dist
    //    vector<DMatch> matches_m;
    //    BFMatcher matcher(NORM_HAMMING); //brute-force descriptor matcher
    //    matcher.match(descriptors_1, descriptors_2, matches_m);
    vector<DMatch> match;
    matcher->match( descriptors_1, descriptors_2, match);



    // step 4: filter match points
    double min_dist = 10000, max_dist = 0;

    // find min and max disctance for all matches
    // that is distance for two sets of points that are most similar and least similar
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
      //  double dist = matches_m[i].distance;
        double dist = match[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

    printf("--max dist: %f \n", max_dist);
    printf("--min dist: %f \n", min_dist);

    // when distance between descriptors > two times min, then error
    // but sometimes min is very small, so we set a bottom limit by experience

    //std::vector<DMatch> good_matches;
    for ( int i = 0; i < descriptors_1.rows; i++)
    {
        //if (matches_m[i].distance <= max(2 * min_dist, 30.0));  // again, by experience, dis < 2*min hamming dis
        if ( match[i].distance <= max(2 * min_dist, 30.0) )
        {
            matches.push_back(match[i]);
        }

    }

}


//********end of find_feature_matches ****output :R,t



// pixel coods to camera gui yi coods
Point2d pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2d
            (
                    ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
                    ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
            );
}
//********end of pixelt2cam ****




void bundleAdjustment(
  const vector<Point3f> points_3d,
  const vector<Point2f> points_2d,
  const Mat& K,
  Mat& R, Mat& t)
{
    // *****initilize g2o
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block; // pose is 6d, landmark is 3d
    // linear fnc solver setup
    //    unique_ptr<Block::LinearSolverType> linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>();
    unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverCSparse<Block::PoseMatrixType>());

   // Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>();
    // block solver
   // Block* solver_ptr = new Block (linearSolver);
   // unique_ptr<Block> solver_ptr ( new Block ( linearSolver));
    unique_ptr<Block> solver_ptr ( new Block ( std::move(linearSolver)));     // 矩阵块求解器
   // g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::move(solver_ptr));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    //********end of initilization

    //vertex
        //***vertex for camera pose
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    Eigen::Matrix3d R_mat;
    R_mat<<
        R.at<double> ( 0,0 ), R.at<double> ( 0,1 ), R.at<double> ( 0,2 ),
        R.at<double> ( 1,0 ), R.at<double> ( 1,1 ), R.at<double> ( 1,2 ),
        R.at<double> ( 2,0 ), R.at<double> ( 2,1 ), R.at<double> ( 2,2 );
    pose->setId ( 0 );
    pose->setEstimate( g2o::SE3Quat(
                            R_mat,
                            Eigen::Vector3d(t.at<double> (0,0), t.at<double>(1,0), t.at<double>(2,0))
                       ));
    optimizer.addVertex ( pose );

        //****vertex for landmark
    int index =1;
    for ( const Point3f p:points_3d) // landmarks
    {
        g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
        point->setId(index++);
        point->setEstimate(Eigen::Vector3d(p.x, p.y, p.z));
        point->setMarginalized( true); // g2o has to setup marg, see later in ch10
        optimizer.addVertex(point);
    }


    // parameter: camera intrinsics
    g2o::CameraParameters* cameraK = new g2o::CameraParameters(
            K.at<double> (0,0), Eigen::Vector2d(K.at<double>(0,2), K.at<double>(1,2))
            , 0);
    cameraK->setId(0);
    optimizer.addParameter(cameraK);



    // edges
    index = 1;
    for (const Point2f p:points_2d)
    {
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId ( index );
        edge->setVertex (0, dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(index)));
        edge->setVertex (1, pose);
        edge->setMeasurement (Eigen::Vector2d( p.x, p.y));
        edge->setParameterId ( 0,0 );
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
        index++;
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose( true);
    optimizer.initializeOptimization();
    optimizer.optimize (100);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout<<"optimization costs time:"<<time_used.count()<<" seconds, how about that??"<<endl;

    cout<<endl<<"after optimzation:"<<endl;
    cout<<"T = "<<endl<<Eigen::Isometry3d( pose->estimate() ).matrix() <<endl;


}







int main(int argc, char **argv) 
{

  if ( argc!= 5 )
  {
    cout<<"usg: pose_estimation 3d2d img1 img2 depth1 depth2"<<endl;
    return 1;
  }


  // read img
  Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
  Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);


  // define key pts and distance match
  vector<KeyPoint> keypoints_1, keypoints_2;
  vector<DMatch> matches;

  find_feature_matches (img_1, img_2, keypoints_1, keypoints_2, matches);
  cout<<"I found total number of "<<matches.size()<<"    set of matching points"<<endl;


  // create 3d points
  Mat d1 = imread ( argv[3], CV_LOAD_IMAGE_UNCHANGED ); // depth pic is 16 bit unsigned num, single channel
  Mat  K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
  vector<Point3f> pts_3d;
  vector<Point2f> pts_2d;
  for (DMatch m:matches)
  {
      ushort d = d1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(
              keypoints_1[m.queryIdx].pt.x)]; //Mat::ptr
      // mat.ptr<type>(y)[x]  **not sure
      if (d == 0)//  bad depth
          continue;
      float dd = d / 5000.0;
      Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt,
                             K); //keypoint(in cv) 2d coords(pixel frame): keypoints[m.queryIdx].pt
      pts_3d.push_back(Point3f(p1.x * dd, p1.y * dd, dd));  // 2d points to 3d, adding depth
      // why need * dd? when normalized?
      pts_2d.push_back(keypoints_2[m.trainIdx].pt); // keypoints_2 2d coods in pixel frame
  }
    cout<<"3d-2d pairs: "<<pts_3d.size()<<endl;

   //********end of creating 3d points

    Mat r, t;
    // call opencv pnp, can choose EPNP, DLS....
    // solvePnP: input 3dcoods, 2dcoods, camera intrinsics K;
    // output r,t
    solvePnP(pts_3d, pts_2d, K, Mat(), r,t,false);
    Mat R;
    cv::Rodrigues(r,R);// r is rotation vector, use Rodrigues to transform to matrix R

    cout<<"just seeing r ="<<endl<<r<<endl;
    cout<<"R="<<endl<<R<<endl;
    cout<<"t="<<endl<<t<<endl;

    cout<<"calling bundle adjustment...."<<endl;

    bundleAdjustment( pts_3d, pts_2d, K, R, t);




}
  




