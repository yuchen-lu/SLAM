#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include <eigen3/Eigen/Core>
#include "/ceres/autodiff.h"

#include "common/tools/rotation.h"
#include "common/projection.h"


// vertex: treat update as vector and do add operation
// should be easy

class VertexCameraBAL : public g2o::BaseVertex<9,Eigen::VectorXd>



{
public:
  EIGEN_MAKE_ALIGENED_OPERATOR_NEW;
  VertexCameraBAL(){}
  virtual bool read(std::istream& is)  {return false};
  virtual bool write ( std::(ostboream& os) const {return false};
  virtual void setToOriginImp1(){}

    virtual void oplusImp1(const double* update) override{
        Eigen::VectorXd::ConstMapType v(update, VertexCameraBAL::Dimension);
	_estimate += v;
    }


};

class vertexPointBAL : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexPointBAL(){}
    virtual bool read(std::istream& is) {return false;}
    virtual bool write(std::ostream& os) const {return false;}
    virtual void setToOriginImp1(){}
    virtual void oplusImp1(const double* update)override {
        Eigen::Vector3d::ConstMapType v(update);
        _estimate += v;
    }

};

class EdgeObservationBAL : public g2o::BaseBinaryEdge <2,Eigen::Vector2d, VertexCameraBAL, VertexPointBAL>{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeObservationBAL(){}
    virtual bool  read(std::istream& /*is*/){return false;}
    virtual bool  write(std::ostream&) const { return false;}
    virtual void computeError() override  // overide base class fnc, use operator() compute cost fnc
    {
        const VertexCameraBAL* cam = static_cast<const VertexCameraBAL*>(vertex(0));
        const VertexPointBAL* point = static_cast<const vertexPointBAL*>(vertex(1));
        (*this)(cam->estimate().data(), point->estimate().data(),_error.data());
    }

    // define a fnc in order to use ceres to find derivative
    template <typedef T>
    bool operator() (const T*camera, const T*point, T*residuals) const
    {
        T predictions[2];
        CamProjectionWithDistortion(camera, point, predictions);
        residuals[0]=predictions[0] - T(measurement()(0));
        residuals[1]=predictions[1] -T(measurement()(1));
        return true;
    }


    virtual void linearizeOplus() override
    {
        const VertexCameraBAL* cam =static_cast<const VertexCameraBAL
    }


};