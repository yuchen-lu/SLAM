#include<iostream>
#include<opencv2/core/core.hpp>
#include<chrono>
#include<ceres/ceres.h>


// problem : curve fitting estimate a,b,c (not x) to minimize an equation
//whole idea:
// use opencv noise generator, get 100 data with gaussian noise, then use ceres:
// define cost function, write as a functor, like a function a<double>()
//use AddRedisualBlock to add error terms into func
//coz optimization needs gradient, we can 1. auto diff by ceres 2.numeric diff 3.by hand provide deri formula to ceres

//auto diff needs specify dimensions of error term and opti variables; error scale 1d, opti abc 3d

//after setting up, use solve func to solve
//in option, we can choose line search/trust region, iter times, steps...

// good: auto diff, no need to find J; achieved by module, can be done when compling

using namespace std;

//model cost funct
//http://ceres-solver.org/automatic_derivatives.html#


struct CURVE_FITTING_COST
{
  CURVE_FITTING_COST (double x, double y): _x (x), _y (y) {}  //initilization
  
  // find residual difference
  // template is required by ceres: http://ceres-solver.org/nnls_modeling.html#autodiffcostfunction
  template <typename T>
  bool operator()(const T* const abc, /* model para. 3d*/ T* residual /* residu*/) const
  {
    //y-exp(ax^2+bx+c)
    residual[0] = T(_y) - ceres::exp(abc[0]*T(_x)*T(_x) +abc[1]*T(_x)+abc[2]);
    return true;
  }
  const double _x,_y;  //x,y data
};



int main(int argc, char**argv)
{
    double a=1.0, b=2.0, c=1.0; //true params
    int N=100; //data pts
    double w_sigma=1.0; // sigma of noise
    cv::RNG rng; // opencv random number generator
    double abc[3]={0,0,0}; // estimaytion of abc params
  
    vector<double> x_data, y_data;  //data
    cout<<"generating data"<<endl;
    for (int i=0;i<N;i++)
    {
      double x =i/100.0;
      x_data.push_back ( x );
      y_data.push_back(
      exp(a*x*x +b*x+c)+rng.gaussian(w_sigma)

    );
    cout<<x_data[i]<<" "<<y_data[i]<<endl;
    }



    // construct lsp 
    ceres::Problem LSPprob;
    for (int i=0; i<N;i++)
    {
        LSPprob.AddResidualBlock( // add error term;
        //use auto derivative, module params: error type,output/input dimensions
        // values refer to previous struct style

        new ceres::AutoDiffCostFunction<CURVE_FITTING_COST,1,3>( new CURVE_FITTING_COST (x_data[i], y_data[i])),
        nullptr, //no core function here, so empty
        abc // abc is params waited to be estimated
          ); //
    }
    
    
    // config solver
    ceres::Solver::Options options; // many config terms to fill out
    options.linear_solver_type = ceres::DENSE_QR; // how to solve delta x func
    options.minimizer_progress_to_stdout = true; //output to cout
    
    ceres::Solver::Summary summary; //optimization info
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve(options,&LSPprob,&summary);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;    
   
    
    //output result
    cout<<summary.BriefReport()<<endl;
    cout<<"estimated abc =";
    for (auto a:abc) cout<<a<<" ";
    cout<<endl;
    
    return 0;
    
  }
