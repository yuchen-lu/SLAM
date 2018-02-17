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



void pose_estimation_3d3d
        ( const vector<Point3f>& pts1,
          const vector<Point3f>& pts2,
          Mat& R, Mat& t)
{
    //***********step1: compute two centre of mass for two sets of pts, then remove them for every point
    Point3f p1, p2; // center of mass
    // p1 = 1/n * Sumof(pi)
    // p2 = 1/n * Sumof(pi')
    int N = pts1.size();
    for ( int i=0; i<N; i++ )
    {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 /= N;
    p2 /= N;

    // qi = pi -p
    vector<Point3f> q1(N), q2(N); // remove the center

    // why have to sepecify dim?? q1(N) not q1??
    for (int i=0; i<N; i++)
    {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    //***********step2: find R
    // compute W = sum of q1*q2'^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for ( int i=0; i<N; i++)
    {
        W += Eigen::Vector3d (q1[i].x, q1[i].y, q1[i].z) *
                Eigen::Vector3d( q2[i].x, q2[i].y, q2[i].z ).transpose();
    }
    cout<<"W = : "<<W<<endl<<endl;

    // do a SVD decompo on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd (W, Eigen::ComputeFullU|Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    cout<<"U="<<U<<endl;
    cout<<"V="<<V<<endl;

    // step 3: compute t
    Eigen::Matrix3d R_ = U* (V.transpose() );
    Eigen::Vector3d t_ = Eigen::Vector3d( p1.x, p1.y, p1.z ) - R_ * Eigen::Vector3d( p2.x, p2.y, p2.z);

    // step 4: convert R, t to cv::Mat
    R = ( Mat_<double>(3,3)<<
        R_ ( 0,0 ), R_ ( 0,1 ), R_ ( 0,2 ),
        R_ ( 1,0 ), R_ ( 1,1 ), R_ ( 1,2 ),
        R_ ( 2,0 ), R_ ( 2,1 ), R_ ( 2,2 )
        );

    t = ( Mat_<double>(3,1)<<
        t_( 0,0 ),
        t_( 1,0 ),
        t_( 2,0 )
        );

    // ********important note: here R,t is from 2nd frame to 1st frame, need to do inverse()!!!!


}


//*******end of 3d3d



//void bundleAdjustment(
//        const vector<Point3f> pts1,
//        const vector<Point3f> pts2,
//        const Mat& K,
//        Mat& R, Mat& t)
//{
//    // *****initilize g2o
//    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block; // pose is 6d, landmark is 3d
//    // linear fnc solver setup
//    //    unique_ptr<Block::LinearSolverType> linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>();
//    unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverCSparse<Block::PoseMatrixType>());
//
//    // Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>();
//    // block solver
//    // Block* solver_ptr = new Block (linearSolver);
//    // unique_ptr<Block> solver_ptr ( new Block ( linearSolver));
//    unique_ptr<Block> solver_ptr ( new Block ( std::move(linearSolver)));     // 矩阵块求解器
//    // g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
//    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::move(solver_ptr));
//
//    g2o::SparseOptimizer optimizer;
//    optimizer.setAlgorithm(solver);
//
//    //********end of initilization
//
//    //vertex
//    //***vertex for camera pose
//    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
//    Eigen::Matrix3d R_mat;
//    R_mat<<
//         R.at<double> ( 0,0 ), R.at<double> ( 0,1 ), R.at<double> ( 0,2 ),
//            R.at<double> ( 1,0 ), R.at<double> ( 1,1 ), R.at<double> ( 1,2 ),
//            R.at<double> ( 2,0 ), R.at<double> ( 2,1 ), R.at<double> ( 2,2 );
//    pose->setId ( 0 );
//    pose->setEstimate( g2o::SE3Quat(
//            R_mat,
//            Eigen::Vector3d(t.at<double> (0,0), t.at<double>(1,0), t.at<double>(2,0))
//    ));
//    optimizer.addVertex ( pose );
//
//    //****vertex for landmark
//    int index =1;
//    for ( const Point3f p:pts1) // landmarks
//    {
//        g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
//        point->setId(index++);
//        point->setEstimate(Eigen::Vector3d(p.x, p.y, p.z));
//        point->setMarginalized( true); // g2o has to setup marg, see later in ch10
//        optimizer.addVertex(point);
//    }
//
//
//    // parameter: camera intrinsics
//    g2o::CameraParameters* cameraK = new g2o::CameraParameters(
//            K.at<double> (0,0), Eigen::Vector2d(K.at<double>(0,2), K.at<double>(1,2))
//            , 0);
//    cameraK->setId(0);
//    optimizer.addParameter(cameraK);
//
//
//
//    // edges
//    index = 1;
//    for (const Point3f p:pts2)
//    {
//        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
//        edge->setId ( index );
//        edge->setVertex (0, dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(index)));
//        edge->setVertex (1, pose);
//        edge->setMeasurement (Eigen::Vector3d( p.x, p.y, p.z));
//        edge->setParameterId ( 0,0 );
//        edge->setInformation(Eigen::Matrix3d::Identity());
//        optimizer.addEdge(edge);
//        index++;
//    }
//
//    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
//    optimizer.setVerbose( true);
//    optimizer.initializeOptimization();
//    optimizer.optimize (100);
//    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
//    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
//    cout<<"optimization costs time:"<<time_used.count()<<" seconds, how about that??"<<endl;
//
//    cout<<endl<<"after optimzation:"<<endl;
//    cout<<"T = "<<endl<<Eigen::Isometry3d( pose->estimate() ).matrix() <<endl;
//
//
//}

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





int main( int argc, char** argv)
{
    // read img
    Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);


    // define key pts and distance match
    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;

    find_feature_matches (img_1, img_2, keypoints_1, keypoints_2, matches);
    cout<<"I found total number of "<<matches.size()<<"    set of matching points"<<endl;


    // create 3d points
    Mat depth1 = imread ( argv[3], CV_LOAD_IMAGE_UNCHANGED ); // depth pic is 16 bit unsigned num, single channel
    Mat depth2 = imread ( argv[4], CV_LOAD_IMAGE_UNCHANGED ); // depth pic is 16 bit unsigned num, single channel

    Mat  K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    vector<Point3f> pts1, pts2;

    for (DMatch m:matches)
    {
        ushort d1 = depth1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(
                keypoints_1[m.queryIdx].pt.x)]; //Mat::ptr

        ushort d2 = depth2.ptr<unsigned short>(int(keypoints_2[m.trainIdx].pt.y))[int(
                keypoints_1[m.trainIdx].pt.x)]; //Mat::ptr
        // mat.ptr<type>(y)[x]  **not sure
        if (d1 == 0 || d2 ==0)//  bad depth
            continue;
        float dd1 = d1 / 5000.0;
        float dd2 = d2 / 5000.0;

        Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt,
                               K); //keypoint(in cv) 2d coords(pixel frame): keypoints[m.queryIdx].pt
        Point2d p2 = pixel2cam(keypoints_2[m.trainIdx].pt,
                               K); //keypoint(in cv) 2d coords(pixel frame): keypoints[m.queryIdx].pt

        pts1.push_back(Point3f(p1.x * dd1, p1.y * dd1, dd1));  // 2d points to 3d, adding depth
        // why need * dd? when normalized?
        pts2.push_back(Point3f(p1.x * dd2, p1.y * dd2, dd2));
    }

    cout<<"3d-3d pairs: "<<pts1.size()<<endl;

    //********end of creating 3d points
    Mat R, t;
    cout<<"check check"<<endl;
    pose_estimation_3d3d(pts1, pts2, R, t);
    cout<<"ICP via SVD result:   "<<endl;
    cout<<"R ="<<R<<endl;
    cout<<"t ="<<t<<endl;
    cout<<"R_inverse="<<R.t()<<endl;//????
    cout<<"t_inv = "<<-R.t()<<endl;


//    cout<<"calling BA"<<endl;
//    bundleAdjustment(pts1, pts2, R, t);
//
//    // vevrify p1 = R*p2 + t
//    for ( int i=0; i<5; i++ )
//    {
//        cout<<"p1 = "<<pts1[i]<<endl;
//        cout<<"p2 = "<<pts2[i]<<endl;
//        cout<<"(R*p2+t) = "<<
//            R * (Mat_<double>(3,1)<<pts2[i].x, pts2[i].y, pts2[i].z) + t
//            <<endl;
//        cout<<endl;
//    }



}