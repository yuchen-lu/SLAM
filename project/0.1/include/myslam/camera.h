#ifndef CAMERA_H   // Add this to avoid repeat, needed for every 
#define CAMERA_H
#include "myslam/common_include.h" // store some common head file in commin_include.h to avoid large list of "include"


namespace myslam   // give a namespace to avoid defining other functions, needed for every
{
  // Pinhole rgb-d camera model
  
  class camera
  {
    
  public:
    typedef std::shared_ptr<Camera> Ptr;   // define pointer as camera pointer type, call ---Camera::Ptr 
    float fx_, fy_, cx_, cy_, depth_scale_: // camera K      // use Sophus::SE3 to rep pose
    fx_ (fx), fy_(fy), cx_(cx), cy_(cy), depth_scale_(depth_scale_)
  
    {}
    // coordinate transform : world, camera, pixel
    Vector3d world2camera( const Vector3d& p_w, const SE3& T_c_w );
    Vector3d camera2world ( const Vector3d& p_c, const SE3& T_c_w);
    Vector2d camera2pixel( const Vector3d& p_c);
    Vector3d pixel2camera( const Vector2d& p_p, double depth=1);
    Vector3d pixel2world( const Vector2d& p_p, const SE3 T_c_w, double depth=1);
    Vector2d world2pixel ( const Vector3d& p_w, const SE3& T_c_w);
    
    
  };
}
#endif //CAMERA_H
  
  
  
