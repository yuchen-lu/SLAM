
#include<iostream>
#include<g2o/core/base_vertex.h>
#include<g2o/core/base_unary_edge.h>
#include<g2o/core/block_solver.h>
#include<g2o/core/optimization_algorithm.h>
#include<g2o/core/optimization_algorithm_gauss_newton.h>
#include<g2o/core/optimization_algorithm_dogleg.h>
#include<g2o/core/optimization_algorithm_levenberg.h>
#include<g2o/solvers/dense/linear_solver_dense.h>
#include<eigen3/Eigen/Core>
#include<opencv2/core/core.hpp>
#include<cmath>
#include<chrono>

//The problem is that at installation of g2o they do not copy the "FindG2O.cmake" into the "/usr/share/cmake-X.X/Modules/" directory.
// This is where the "find_package" function of cmake usually looks for it. So just copying the "FindG2O.cmake" file there manually solved the problem for me.

using namespace std;

// peak if curve func, model params: opti variable dimension and data type

class CurveFittingVertex: public g2o::BaseVertex<3,Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void setToOriginTmpl() //reset
    {
      _estimate << 0,0,0;
      
      
    }
    
    virtual void oplusImpl( const double* update) //update
    {
      _estimate+=Eigen::Vector3d(update);
      
      
    }
    
    //read and write: leave blank
    virtual bool read( istream& in ){}
    virtual bool write(ostream& out) const{}
    
  
  
  
  
};

//error model params: observation dimension, type, connection vertex type

class CurveFittingEdge: public g2o::BaseUnaryEdge<1,double,CurveFittingVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge(double x): BaseUnaryEdge(),_x(x){}
    
    // compute curve model error
    void computeError()
    {
      const CurveFittingVertex* v = static_cast<const CurveFittingVertex*>(_vertices[0]);
      const Eigen::Vector3d abc = v->estimate();
      _error(0,0) = _measurement -std::exp( abc(0,0)*_x*-_x+abc(1,0)*abc(2,0));
      
    }
    virtual bool read(istream& in){}
    virtual bool write(ostream& out) const{}
public:
  
  double _x; // x, y are measurement
};

int main( int argc, char** argv)
{
  double a=1.0,b=2.0,c=1.0;
  int N=100;
  double w_sigma=1.0;
  cv::RNG rng;
  double abc[3]={0,0,0};
  
  vector<double> x_data, y_data;
  cout<<"generating data:"<<endl;
  for (int i=0; i<N;i++)
  {
    double x=i/100.0;
    x_data.push_back(x);
    y_data.push_back(exp(a*x*x+b*x+c)+rng.gaussian(w_sigma));
    cout<<x_data[i]<<' '<<y_data[i]<<endl;
  }
    
  // construct img opti,first set up g20
  // matrix blocks(has a term in the book): every error term opti variable is 3d, error dim is 1d
 // typedef g2o::BlockSolver<g2o::BlockSolverTraits<3,1>> Block;
  typedef g2o::BlockSolver< g2o::BlockSolverTraits<3,1> > Block;
  // linear func solver: dense delta x func
  //Block::LinearSolverType* linearSolver =new g2o::LinearSolverDense<Block::PoseMatrixType>();   // linear func solver
 Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
  
 // Block* solver_ptr = new Block( linearSolver ); // matrix block solver
  Block* solver_ptr = new Block( linearSolver ); 
  // gradient decrese methods, pick from GN, LM and dogleg
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  
  // gm or dogleg in comment below
  //g2o::OptimizationAlgorithmDogleg* slver = new g2o::OptimizationAlgorithmDogleg(solver_ptr);
  
  g2o::SparseOptimizer optimizer; // img model
  optimizer.setAlgorithm(solver); // set solver
  optimizer.setVerbose(true);
  
  // add vertex to imges
  CurveFittingVertex* v = new CurveFittingVertex();
  v->setEstimate(Eigen::Vector3d(0,0,0));
  v->setId(0);
  optimizer.addVertex(v);

  // add edges to imges
  for ( int i=0; i<N; i++)
  {
    CurveFittingEdge* edge = new CurveFittingEdge(x_data[i]);
    edge->setId(i);
    edge->setVertex(0,v); // set connection vertex
    edge->setMeasurement(y_data[i]);
    
    // info matrix:
    edge->setInformation(Eigen::Matrix<double,1,1>::Identity()*1/(w_sigma*w_sigma));
    optimizer.addEdge(edge);
    
    
    
    
  }
  
  // excuate optimization
  cout<<"start optimization"<<endl;
  chrono::steady_clock::time_point t1= chrono::steady_clock::now();
  optimizer.initializeOptimization();
  optimizer.optimize(100);
  chrono::steady_clock::time_point t2 =chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
  cout<<"solve time cost"<<time_used.count()<<"seconds"<<endl;
  
  // output optimization value
  Eigen::Vector3d abc_estimate = v->estimate();
  cout<<"esti model:"<<abc_estimate.transpose()<<endl;
   
    
    
  return 0;
  
  
  
  
}
  
  
  
}







/}