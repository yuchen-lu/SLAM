// g20: a library based on graph optimization
// graph opti is a theory combining non-lin op & graphic theory

// some intro to graphic optimization:
// before, in non-linear lsp, the solution is by sum of many error terms
// problem: one set of optimized varibles and many error terms, no relationship found

// need graph opt, treat opti problem as graph, consists of mant vertexs and edges
// use  vertexs to rep opti variables(such as abc in curve fitting)
// edges to rep cost func(error term), so any lsp can be a graph

#include<iostream>
#include<eigen3/Eigen/Core>
#include<opencv2/core/core.hpp>
#include<cmath>
#include<chrono>

#include<g2o/core/block_solver.h>
#include<g2o/core/optimization_algorithm.h>
#include<g2o/core/optimization_algorithm_gauss_newton.h>
#include<g2o/core/optimization_algorithm_dogleg.h>
#include<g2o/core/optimization_algorithm_levenberg.h>
#include<g2o/solvers/dense/linear_solver_dense.h>
#include<g2o/core/base_vertex.h>
#include<g2o/core/base_unary_edge.h>

// using g2o to do curvefitting, remember vertexs are opti variables, edges are error terms
// steps: 1. define types for vertexs and edges
// 	  2. struct graph
// 	  3. choose optimization_algorithm
// 	  4. call g2o to optimize, return result



using namespace std;

// peak if curve func, model params: opti variable dimension and data type

// step1-1: define vertex type
class CurveFittingVertex: public g2o::BaseVertex<3,Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void setToOriginTmpl() //vertex reset fnc
    {
      _estimate << 0,0,0;
    }
    
    virtual void oplusImpl( const double* update) //vertex update fnc
    {
      _estimate += Eigen::Vector3d(update);
    }
    
    //read and write disk : leave blank for now
    virtual bool read( istream& in ){}
    virtual bool write(ostream& out) const{}

};

//error model params: observation dimension, type, connection vertex type

// step 1-2: define edge type
class CurveFittingEdge: public g2o::BaseUnaryEdge<1,double,CurveFittingVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge(double x): BaseUnaryEdge(),_x(x){}

    void computeError()  //edge error fnc
    //takes two vertexs that connects to one edge, use curve model to compare with its actual value; same as LSP
    {
      const CurveFittingVertex* v = static_cast<const CurveFittingVertex*>(_vertices[0]);
      const Eigen::Vector3d abc = v->estimate();
      _error(0,0) = _measurement - std::exp( abc(0,0)*_x*_x + abc(1,0)*_x + abc(2,0)); //error = measure- ax^2+bx+c
    }
    virtual bool read(istream& in){}
    virtual bool write(ostream& out) const{}

public:
  
  double _x; // x, y are _measurement
};



int main( int argc, char** argv)
{
  double a=1.0, b=2.0, c=1.0;
  int N=100;
  double w_sigma = 1.0;
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
    
  // construct graph opti,first set up g2o
  // matrix blocks: every error term opti variable is 3d, error value dim is 1d
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<3,1> > Block; // opti variable is 3d(a,b,c), error variable 1d(scalar)


  // linear func solver: dense delta x func
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();// linear func solver
    // Block* solver_ptr = new Block( linearSolver );
    Block* solver_ptr = new Block( linearSolver );     // matrix block solver


    // choose gradient desent methods, pick from GN, LM and dogleg
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    // gm or dogleg in comment below
    //g2o::OptimizationAlgorithmDogleg* slver = new g2o::OptimizationAlgorithmDogleg(solver_ptr);

    g2o::SparseOptimizer optimizer; // graph model
    optimizer.setAlgorithm(solver); // set solver
    optimizer.setVerbose(true); // open output

    // add vertex to graphs
    CurveFittingVertex* v = new CurveFittingVertex();
    v->setEstimate( Eigen::Vector3d(0,0,0) );
    v->setId(0);
    optimizer.addVertex( v );


    // add edges to graphs
    for ( int i=0; i<N; i++)
    {
    CurveFittingEdge* edge = new CurveFittingEdge(x_data[i]);
    edge->setId(i);
    edge->setVertex(0,v); // set connection's vertex
    edge->setMeasurement(y_data[i]);  // observation data

    // info matrix: inverse matrix of sth difficult
    edge->setInformation( Eigen::Matrix<double,1,1>::Identity()*1/(w_sigma*w_sigma) );
    optimizer.addEdge(edge);
    }

    // execute optimization
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
  
  
  


/*

#include<eigen3/Eigen/Core>
#include<opencv2/core/core.hpp>
#include<cmath>
#include<chrono>

#include<g2o/g2o/core/block_solver.h>
#include<g2o/g2o/core/optimization_algorithm.h>
#include<g2o/g2o/core/optimization_algorithm_gauss_newton.h>
#include<g2o/g2o/core/optimization_algorithm_dogleg.h>
#include<g2o/g2o/core/optimization_algorithm_levenberg.h>
#include<g2o/g2o/solvers/dense/linear_solver_dense.h>
#include</usr/include/g2o/g2o/core/base_vertex.h>
#include</usr/include/g2o/g2o/core/base_unary_edge.h>

using namespace std;

// 曲线模型的顶点，模板参数：优化变量维度和数据类型
class CurveFittingVertex: public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void setToOriginImpl() // 重置
    {
        _estimate << 0,0,0;
    }

    virtual void oplusImpl( const double* update ) // 更新
    {
        _estimate += Eigen::Vector3d(update);
    }
    // 存盘和读盘：留空
    virtual bool read( istream& in ) {}
    virtual bool write( ostream& out ) const {}
};

// 误差模型 模板参数：观测值维度，类型，连接顶点类型
class CurveFittingEdge: public g2o::BaseUnaryEdge<1,double,CurveFittingVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge( double x ): BaseUnaryEdge(), _x(x) {}
    // 计算曲线模型误差
    void computeError()
    {
        const CurveFittingVertex* v = static_cast<const CurveFittingVertex*> (_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        _error(0,0) = _measurement - std::exp( abc(0,0)*_x*_x + abc(1,0)*_x + abc(2,0) ) ;
    }
    virtual bool read( istream& in ) {}
    virtual bool write( ostream& out ) const {}
public:
    double _x;  // x 值， y 值为 _measurement
};

int main( int argc, char** argv )
{
    double a=1.0, b=2.0, c=1.0;         // 真实参数值
    int N=100;                          // 数据点
    double w_sigma=1.0;                 // 噪声Sigma值
    cv::RNG rng;                        // OpenCV随机数产生器
    double abc[3] = {0,0,0};            // abc参数的估计值

    vector<double> x_data, y_data;      // 数据

    cout<<"generating data: "<<endl;
    for ( int i=0; i<N; i++ )
    {
        double x = i/100.0;
        x_data.push_back ( x );
        y_data.push_back (
            exp ( a*x*x + b*x + c ) + rng.gaussian ( w_sigma )
        );
        cout<<x_data[i]<<" "<<y_data[i]<<endl;
    }

    // 构建图优化，先设定g2o
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<3,1> > Block;  // 每个误差项优化变量维度为3，误差值维度为1
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>(); // 线性方程求解器
    Block* solver_ptr = new Block( linearSolver );      // 矩阵块求解器
    // 梯度下降方法，从GN, LM, DogLeg 中选
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr );
    // g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( solver_ptr );
    g2o::SparseOptimizer optimizer;     // 图模型
    optimizer.setAlgorithm( solver );   // 设置求解器
    optimizer.setVerbose( true );       // 打开调试输出

    // 往图中增加顶点
    CurveFittingVertex* v = new CurveFittingVertex();
    v->setEstimate( Eigen::Vector3d(0,0,0) );
    v->setId(0);
    optimizer.addVertex( v );

    // 往图中增加边
    for ( int i=0; i<N; i++ )
    {
        CurveFittingEdge* edge = new CurveFittingEdge( x_data[i] );
        edge->setId(i);
        edge->setVertex( 0, v );                // 设置连接的顶点
        edge->setMeasurement( y_data[i] );      // 观测数值
        edge->setInformation( Eigen::Matrix<double,1,1>::Identity()*1/(w_sigma*w_sigma) ); // 信息矩阵：协方差矩阵之逆
        optimizer.addEdge( edge );
    }

    // 执行优化
    cout<<"start optimization"<<endl;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;

    // 输出优化值
    Eigen::Vector3d abc_estimate = v->estimate();
    cout<<"estimated model: "<<abc_estimate.transpose()<<endl;

    return 0;
}*/
