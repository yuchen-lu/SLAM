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
    J.block(0,3,3,3) = SO3::hat(e.translation());
    J.block(3,0,3,3) = Eigen::Matrix3d::Zero(3,3);
    J.block(3,3,3,3) = SO3::hat(e.so3().log());
    J = J * 0.5+Matrix6d::Identity();
    return J;

}

// vertex lie alg
typedef Eigen::Matrix(double,6,1) Vector6d;
class VertexSE3LieAlgebra: public g2o::BaseVertex<6,SE3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool read(istream& is)  // is is a ref to some input istream

    {
        double data[7];
        for ( int i =0; i<7; i++)
        is>>data[i];
        setEstimate(SE3   ( Eigen::Quaternion( data[6],
        data[3],data[4], data[5]), Eigen::Vector3d( data[0], data[1], data[2])
        ));
    }

    bool write ( ostream& os) const
    {
        os<<id()<<" ";
        Eigen::Quaterniond q =_estimate.unit_quaternion();
        os<<_estimate.translation().transpose()<<" ";
        os<<q.coeffs()[0]<<" "<<q.coeffs()[1]<<" "<<q.coeffs()[2]<<" "<<q.coeffs()[3]<<endl;
        return true;
    }

    virtual void setToOriginImp1()
    {
        _estimate = Sophus::SE3;
    }

    //left multiplt update
    virtual void oplusImp1( const double* update)
    {
        Sophus::SE3 up(
                Sophus::SO3( update[3], update[4], update[5]),
                Eigen::Vector3d( update[0], update[1], update[2])
        );
        _estimate = up*_estimate;

    }


};



// two lie alg vertexs' edge
class EdgeSE3LieAlgebra: public g2o::BaseBinaryEdge<6,SE3, VertexSE3LieAlgebra, VertexSE3LieAlgebra>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool read(istream& is)
    {
        double data[7];
        for ( int i =0; i<7; i++)
            is>>data[1];
        Eigen::Quaterniond q ( data[6],data[3], data[4], data[5]);
        q.normalize();
        setMeasurement(
                Sophus::SE3 ( q, Eigen::Vector3d( data[0],data[1],data[2]))
        );
        for (int i=0; i<information().rows() && is.good(); i++)
            for( int j=i;j<information().cols() && is.good(); j++)
        {
            is >> information()(i,j);
            if(i!=j)
                information()(j,i) = information()(i,j);
        }
        return true

    }


    bool write( ostream& os) const
    {
        VertexSE3LieAlgebra* v1 = static_cast<VertexSE3LieAlgebra*>(_vertices[0]);
        VertexSE3LieAlgebra* v2 = static_cast<VertexSE3LieAlgebra*>(_vertices[1]);
        os<<v1->id()<<" "<<v2->id()<<" ";
        SE3 m = _measurement;
        Eigen::Quaterniond q = m.unit_quaternion();
        os<<m.translation().transpose()<<" ";
        os<<q.coeffs()[0]<<" "<<q.coeffs()[1]<<" "<<q.coeffs()[2]<<" "<<q.coeffs()[3]<<" ";
        // information matrix
        for ( int i=0; i<information().rows(); i++)
            for(int j=i; j<information().cols()<<)
    }

};

