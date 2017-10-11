#include <iostream>
#include <ctime>
using namespace std;

//eigen part
#include <eigen3/Eigen/Core>
// dense matrix alg comput(inverse,eigenvalue)
#include<eigen3/Eigen/Dense>

#define MATRIX_SIZE 50

int main( int argc, char** argv)
{
//declare a matrix
  Eigen::Matrix<float,2,3> matrix_23;
  Eigen::Vector3d v_3d;
  Eigen::Matrix3d matrix_33;  //double 3x3
  Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic>matrix_dynamic;
  Eigen::MatrixXd matrix_x;
  
//input valye
  matrix_23 << 1, 2, 3, 4, 5, 6;
  cout<< matrix_23<<endl;
  
  for(int i=0;i<1; i++)
    for (int j=0; j<2;j++)
      cout<<matrix_23(i,j)<<endl;
    
  
    
    
  v_3d<< 3, 2, 1;
  Eigen::Matrix<double,2,1>result=matrix_23.cast<double>() *v_3d; //transfer type
 // Eigen::Matrix<double,2,3>wrong_result=matrix_23.cast<double>() *v_3d; //transfer type

  
  //other manipulation
  matrix_33 =Eigen::Matrix3d::Random();
  cout << matrix_33 << endl << endl;
  cout<< matrix_33.transpose() << endl; // sum(); trace(); inverse();determinant();10*matrix_33;
  
  //eigen values &vectors
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(matrix_33);
  cout << "Eigen values=" << eigen_solver.eigenvalues() << endl;
  cout << "Eigen vectors=" << eigen_solver.eigenvectors() <<endl;
  
  
  
  // solve matrix_NN * x= v_Nd
  Eigen::Matrix< double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN;
  matrix_NN = Eigen::MatrixXd::Random( MATRIX_SIZE, MATRIX_SIZE);
  Eigen::Matrix<double,MATRIX_SIZE,1>v_Nd;
  v_Nd = Eigen::MatrixXd::Random(MATRIX_SIZE,1);
  
  clock_t time_stt=clock();//timer
  //inverse method
  Eigen::Matrix<double,MATRIX_SIZE,1> x=matrix_NN.inverse()*v_Nd;
  cout <<"time use in normal invers is " << 1000* (clock()-time_stt)/(double)CLOCKS_PER_SEC <<"ms" << endl;
  
  // QR decompo
  time_stt = clock();
  x =matrix_NN.colPivHouseholderQr().solve(v_Nd);
  cout << "time used in qr decompo is " << 1000* (clock()-time_stt)/(double)CLOCKS_PER_SEC <<"ms" << endl;
  
  return 0;
  
  
  
  
  
  
  
  
  
  
  
  
  
}