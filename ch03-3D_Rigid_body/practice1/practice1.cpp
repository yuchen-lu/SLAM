
#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

using namespace Eigen;
using namespace std;
int main( int argc, char**argv)

{


// Question 1:
// solve x in Ax = b when A is a random 100x100 matrix using QR and Cholesky decomp.
//



    MatrixXd A = MatrixXd::Random(100,100);
//    cout << A << endl;
    VectorXd b = VectorXd::Random(100,1);
//    cout << b << endl;

    //QR
    VectorXd x = A.colPivHouseholderQr().solve(b);
//    cout << "the qr sol is : \n" << x << endl;
    //Chol
//    LLT<MatrixXd> lltOfA(A); // compute the Cholesky decomposition of A
//    MatrixXd L = lltOfA.matrixL();
//    cout << "The Cholesky factor L is" << endl << L << endl;
//    cout << "To check this, let us compute L * L.transpose()" << endl;
//    cout << L * L.transpose() << endl;
//    cout << "This should equal the matrix A" << endl;

    VectorXd x2 = A.ldlt().solve(b);
    cout << "x2 = \n" << x2<< endl;


// Question 2:
// robot1 pose in world coords： q1 = [0:55; 0:3; 0:2; 0:2]; t1 = [0:7; 1:1; 0:2]T（first of q is real num. q, t rep Tcw (world to cam transform relationship)
// r2 pose: q2 = [−0:1; 0:3; −0:7; 0:2]; t2 = [−0:1; 0:4; 0:8]T
// now, r1 sees p1 = [0:5; −0:1; 0:2]T in its coordinates,  find p1's coods from r2's view
//
    Quaterniond q1 (0.55, 0.3, 0.2, 0.2);
    q1.normalize();

    Quaterniond q2 (-0.1, 0.3, -0.7, 0.2);
    q2.normalize();
//
    Vector3d t1 (0.7, 1.1, 0.2);
    Vector3d t2 (-0.1, 0.4, 0.8);
    Vector3d p1 (0.5, -0.1, 0.2);

    // transform p1 back to world coods
    Quaterniond p_temp;
    p_temp.w() = 0;
    p_temp.vec() = p1 - t1;
    Quaterniond p_world = q1.inverse() * p_temp * q1;
    cout << "p_world :\n" << p_world.vec() << endl;

    // move p1 to p2's coords
    Quaterniond p2;
    p_temp.vec() = p_world.vec();
    p2 = q2 * p_temp * q2.inverse();
    p2.vec() = p2.vec() + t2;
    cout<< "result is : \n" << p2.vec() << endl;

    // method2 : TODO
//    Vector3d p_2;
//    p_2 = q2 *(t1 + q1.conjugate() * p1 - t2);
//    cout<<"p_2" << p_2 << endl;

    return 0;
}

