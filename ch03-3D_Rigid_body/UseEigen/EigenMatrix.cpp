#include <iostream>
#include <ctime>
//eigen part
#include <eigen3/Eigen/Core>
// dense matrix alg comput(inverse,eigenvalue)
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

#define MATRIX_SIZE 50

int main( int argc, char** argv)
{
    //declare a matrix
    Matrix<float, 2, 3> matrix_23;
    Vector3d v_3d;

    Matrix3d matrix_33 = Matrix3d::Zero();  //double 3x3


    Matrix<double, Dynamic, Dynamic> matrix_dynamic;
    MatrixXd matrix_x;

    //input valye
    matrix_23 << 1, 2, 3, 4, 5, 6;
    cout<< matrix_23 << endl;
  
    for(int i = 0;i < 1; i++)
        for (int j = 0; j < 2;j++)
        cout << matrix_23(i,j) << endl;
    
  
    
    
    v_3d << 3, 2, 1;
    Matrix<double,2,1> result = matrix_23.cast<double>() * v_3d; //transfer type
 // Eigen::Matrix<double,2,3>wrong_result=matrix_23.cast<double>() *v_3d; //transfer type


    //other manipulation
    matrix_33 = Matrix3d::Random();
    cout << matrix_33 << endl << endl;
    cout<< matrix_33.transpose() << endl; // sum(); trace(); inverse();determinant();10*matrix_33;
  
    //eigen values &vectors
    SelfAdjointEigenSolver<Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33);
    cout << "Eigen values = \n" << eigen_solver.eigenvalues() << endl;
    cout << "Eigen vectors = \n " << eigen_solver.eigenvectors() <<endl;

    MatrixXf randomcheck = MatrixXf::Random(4,4);
    cout<<"********************"<<endl;
    EigenSolver<MatrixXf> YCsolver;
    YCsolver.compute(randomcheck,  /* computeEigenvectors = */ true);
    cout<<"eigenvalues of randomcheck are:"<<YCsolver.eigenvalues().transpose()<<endl;
    cout<<"1st eigenvector of randomcheck is:"<<YCsolver.eigenvectors().col(0)<<endl;
    cout<<"whats gonna happen without col(0):\n"<<YCsolver.eigenvectors()<<endl;




    // solve matrix_NN * x= v_Nd
    Matrix< double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN;
    matrix_NN = MatrixXd::Random( MATRIX_SIZE, MATRIX_SIZE);
    Matrix<double,MATRIX_SIZE,1>v_Nd;
    v_Nd = MatrixXd::Random(MATRIX_SIZE,1);
  
    clock_t time_stt=clock();//timer

    //inverse method
    Matrix<double, MATRIX_SIZE,1> x = matrix_NN.inverse() * v_Nd;
    cout <<"time use in normal invers is " << 1000* (clock()-time_stt)/(double)CLOCKS_PER_SEC <<"ms" << endl;
  
    // QR decompo
    time_stt = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    cout << "time used in qr decompo is " << 1000* (clock()-time_stt)/(double)CLOCKS_PER_SEC <<"ms" << endl;

    return 0;

  
}