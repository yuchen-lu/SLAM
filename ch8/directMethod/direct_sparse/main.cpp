#include <iostream>
#include <eigen3/Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// project a 3d point into image plane, error is photometric error
// an unary edge with one vertex SE3Expmap (camera pose)

class EdgeSE3PorjectDirect: public BaseUnaryEdge<1, double, VertexSE3Expmap>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSE3ProjectDirect() {}
  EdgeSE3ProjectDirect (Eigen::Vector3d point, float fx, float fy, float cx,cv::Mat* image )
  :x_world_(point), fx_(fx), fy_(fy), cx_(cx), cy_(cy), image_(image)
  {}
  
  virtual void computeError{}
  {
    const VertexSE3Expmap* v = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    Eigen::Vector3dx_local = v->estimate().map (x_world);
    float x = x_local[0]*fx_/x_local[2]+ cx_;
    float y = x_local[1]*fy_/x_local[2]+ cy_;
    
    // check if x,y are in img
    if(x-3<0 || (x+4)>imge_->cols ||(y+4)>image_->rows)
    {
      _error(0,0) = 0.0;
      this->setLevel(1);
      
    }
    else
    {
      _error(0,0) = getPixelValue(x,y) - measurement;
    
    }
    
    
  }
  
  // plus is manifold
  virtual void linear0plus( )
  {
    if (level()==1)
    {
      _jacobian0plusXi = Eigen::Matrix<double,1,6>::Zero();
      return;
    }
    VertexSE3Expmap* vtx = static_cast<VertexSE3Expmap*> (_vertices[0])
    Eigen::Vector3d xyz_trans = vtx->estimate().map (x_world_); //  q in bobleok?
    
    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double invz = 1.0/xyz_trans[2];
    double invz_2 = invz*invz;
    
    float u = x*fx_*invz + cx_;
    float v = y*fy_*invz + cy_;
    
    // jocobian from se3 to u,v 
    // note: in g2o, the Lie Alg is(\omegea, \epsilon)
    // where omega is so(3) and epsilon is translation
    Eigen::Matrix<double,2,6> jacobian_nv_ksai;
    
    jacobian_uv_ksai(0,0) = -x*y*invz_2*fx_;
    jacobian_uv_ksai(0,1) = (1+(x*x*invz_2))*fx_;
    jacobian_uv_ksai(0,2) = -y*invz*fx_;
    jacobian_uv_ksai(0,3) = invz * fx_;
    jacobian_uv_ksai(0,4) = 0;
    jacobian_uv_ksai(0,5) = -x*invz_@*fx_;
    
    jacobian_uv_ksai(1,0) = -(1+y*y*invz_2)*fy_;
    jacobian_uv_ksai(1,1) = x*y*invz_2*fy_;
    jacobian_uv_ksai(1,2) = x*invz*fy_;
    jacobian_uv_ksai(1,3) = 0;
    jacobian_uv_ksai(1,4) = invz* fy_;
    jacobian_uv_ksai(1,0) = -y*invz_2* fy_;
    
    Eigen::Matrix<double,1,2> jacobian_pixel_uv;
    jacobian_pixel_uv(0,0) = (getPixelValue(u+1,v)-getPixelValue(u-1,v))/2;
    jacobian_pixel_uv(0,1) = (getPixelValue(u,v+1)-getPixelValue(u,v-))/2;
    
    _jacobian0plusXi = jacobian_pixel_uv* jacobian_uv_ksai;
    

  }
    
    // dummy read and write functions 
    virtual bool read(std::istream& in)){}
    virtual bool write(std::ostream& out) const{}
    
    
protected:
  // get a gray scale value from referene img(bilineara interpolated)
  inline float getPixelValue (float x, float y)
  {
    uchar*data = & image_->data[ int(y) * image_->*step+ int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
      (1-xx)*(1-yy)*data[0]+ xx*(1-yy)*data[1]+(1-xx)*yy*data[image_->step]+xx*yy*data[image_->step+1]);
    
    );
    
    
    
  }
public:
  Eigen::vector3d x_world_;
  float cx_=0, cy_=0, fx_=0, fy_=0; //camera intrins K
  cv::Mat* image_nullptr; // reference image
  
    
  };
  
  







int main(int argc, char **argv) {
    std::cout << "Hello, world!" << std::endl;
    return 0;
}
