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
        ( const vector<Point3f>& pts1, const vector<Point3f>& pts2,
        Mat& R, Mat& t)
{
    Point3f p1, p2; // center of mass

    // p1 = 1/n * Sumof(pi)
    // p2 = 1/n * Sumof(pi')
    int N = pts1.size();
    for ( int i=0; i<N; i++)
    {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = p1/N;
    p2 = p2/N;

    // qi = pi -p
    Point3f q1, q2
    for (int i=0; i<N; i++)
    {
        q1[i] = pts1[i] - p1[i];
        q2[i] = pts2[i] - p2[i];
    }



}







int main() {
    std::cout << "Hello, World!" << std::endl;
    return 0;
}