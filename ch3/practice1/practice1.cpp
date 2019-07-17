//
// solve x in Ax = b when A is a random 100x100 matrix using QR and Cholesky decomp.
//

#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry> //geo module

using namespace Eigen;
using namespace std;
int main( int argc, char**argv)

{

    MatrixXd A = MatrixXd::Random(100,100);
//    cout << A << endl;
    VectorXf b = VectorXf::Random(100,1);
    VectorXf x = A.colPivHouseholderQr().solve(b);
    cout << "the qr sol is : \n" << x << endl;

    return 0;
}

