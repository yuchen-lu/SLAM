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


int main(int argc, char** argv) {

    vector<Point3f> pts1, pts2;
    Mat img_1 = imread( argv[1], CV_LOAD_IMAGE_COLOR);
    Mat depth_1 = imread( argv[2], CV_LOAD_IMAGE_UNCHANGED);



    cout << "Hello, World!" << std::endl;
    return 0;
}