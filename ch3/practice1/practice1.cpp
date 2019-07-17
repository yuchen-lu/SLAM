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
//    cout << "A"<< A;


    return 0;
}

