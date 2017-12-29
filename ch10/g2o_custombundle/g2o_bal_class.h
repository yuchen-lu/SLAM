#include <iostream>
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
  virtual void set
  
}