#include <iostream>
#include <cmath>
using namespace std;

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry> //geo module

int main( int argc, char**argv)

{
  
  Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
  cout<<"rotation matrix=\n" <<rotation_matrix <<endl; //transform to matrix

  Eigen::AngleAxisd rotation_vector(M_PI/4, Eigen::Vector3d(0,0,1)); // 45 along z axis
  cout .precision(3);
  cout<<"rotation matrix=\n" <<rotation_vector.matrix() <<endl; //transform to matrix
  rotation_matrix = rotation_vector.toRotationMatrix();
  
  //using AngleAxis(3X1 vector) to transform coords
  Eigen::Vector3d v (1,0,0);
  Eigen::Vector3d v_rotated = rotation_vector * v;
  cout<<"(1,0,0) after rotation = " <<v_rotated.transpose()<<endl;
  
  //or using roation matrix
  v_rotated = rotation_matrix *v;
  cout<<"(1,0,0) after rotation = " <<v_rotated.transpose()<<endl;

  
  //euler's Angle, transform rotation matrix to eulers angle
  Eigen::Vector3d euler_angles =rotation_matrix.eulerAngles(2,1,0);//zyx,rpy
  cout << "paw pitch roll =" <<euler_angles.transpose();
  
  
  //isometry
  Eigen::Isometry3d T=Eigen::Isometry3d::Identity(); //actually 4*4 matrix
  T.rotate(rotation_vector); 
  T.pretranslate (Eigen::Vector3d (1,3,4)); //move
  cout << "transofrm matrix =\n" <<T.matrix() <<endl;
  
  //transofrm using matrix
  Eigen::Vector3d v_transformed =T*v; //R*v+t
  cout<<"v_transformed =" <<v_transformed.transpose()<<endl;
  
  
  //quaternion
  //give AngleAxis to quaternion directly
  Eigen::Quaterniond q= Eigen::Quaterniond(rotation_vector);
  cout<<"quaternion =\n"<<q.coeffs()<<endl; //coeffs(x,y,z,w),w is real value
  
  // or give rotatoin_matrix to quaternion
  q=Eigen::Quaterniond( rotation_matrix);
  cout<<"quaternion =\n"<<q.coeffs()<<endl;
  
  //use quaertion to roate a vector, using multiply
  v_rotated =q*v; //qvq^-1
  cout<<"(1,0,0) after rotation =" <<v_rotated.transpose()<<endl;
  
  return 0;
  
  
  
  
 
  
  
  
  
  
  
  
  
  
  
  
}