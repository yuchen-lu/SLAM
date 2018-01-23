//
// Created by yuchen on 22/01/18.
//

#include <iostream>
#include<fstream>
#include <string>
#include <Eigen/Core>
#include<g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>

#include <sophus/se3.h>
#include <sophus/so3.h>

using namespace std;
using Sophus::SE3;
using Sophus::SO3;

// how to use g2o solve to do pose graph optimization
//using lie alg to rep pose, verex and edge are defined

typedef Eigen::Matrix<double,6,6> Matrix6d;

// approxi J_R^-1
Matrix6d JRInv (SE3 e)
{
    Matrix6d J;
    J.block(0,0,3,3) = SO3::hat(e.so3().log());
    
}

