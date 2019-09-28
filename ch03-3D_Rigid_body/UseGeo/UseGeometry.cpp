#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry> //geo module

using namespace Eigen;
using namespace std;
int main( int argc, char**argv)

{

  
    Matrix3d rotation_matrix = Matrix3d::Identity();
    cout<<"rotation matrix=\n" <<rotation_matrix <<endl; //transform to matrix

    AngleAxisd rotation_vector (M_PI/4, Vector3d(0,0,1)); // (theta,n)45 along z axis
    cout.precision(3);
    cout<<"rotation matrix = \n" <<rotation_vector.matrix() <<endl; //angleaxis to matrix
    rotation_matrix = rotation_vector.toRotationMatrix();
  
    //using AngleAxis(3X1 vector) to transform coords
    Vector3d v (1,0,0);
    Vector3d v_rotated = rotation_vector * v;
    cout<<"(1,0,0) after rotation = " <<v_rotated.transpose()<<endl;
  
    //or using roation matrix
    v_rotated = rotation_matrix *v;
    cout<<"(1,0,0) after rotation = " <<v_rotated.transpose()<<endl;

  
    //euler's Angle, transform rotation matrix to eulers angle
    Vector3d euler_angles = rotation_matrix.eulerAngles(2,1,0);//zyx,ypr
    cout << "yaw pitch roll =" <<euler_angles.transpose() << endl;
  
  
    //isometry
    // how to construct T from R,t
    Isometry3d T=Isometry3d::Identity(); //actually 4*4 matrix
    T.rotate(rotation_vector);
    T.pretranslate (Vector3d (1,3,4)); //move
    cout << "transofrm matrix =\n" <<T.matrix() <<endl;

    //transofrm using matrix
    Vector3d v_transformed =T*v; //R*v+t
    cout<<"v_transformed =" <<v_transformed.transpose()<<endl;


    //quaternion

    // Note the order of the arguments: the real w coefficient first, while internally the coefficients are stored in the following order: [x, y, z, w]
    //give AngleAxis to quaternion directly
    Quaterniond q = Quaterniond(rotation_vector);
    cout << "quaternion =\n"<<q.coeffs().transpose() << endl; //coeffs(x,y,z,w),w is real value

    // or give rotatoin_matrix to quaternion
    q = Quaterniond(rotation_matrix);
    cout << "quaternion =\n"<<q.coeffs().transpose() << endl;

    //use quaertion to roate a vector, using multiply
    v_rotated = q * v; //qvq^-1 in math!
    cout<<"(1,0,0) after rotation =" <<v_rotated.transpose()<<endl;

    return 0;

}