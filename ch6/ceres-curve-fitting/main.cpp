#include<iostream>
#include<opencv2/core/core.hpp>
#include<chrono>
#include<ceres/ceres.h>




using namespace std;

//model cost funct

struct CURVE_FITTING_COST
{
  CURVE_FITTING_COST (double x, double y): _x (x), _y (y) {}
  
  // find residual difference
  template <typename T>
  bool operator()(
    const T* const abc, // model para. 3d
    T* residual ) const // residu
  
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
    double abc[3]={0,0,0}; // approxi of abc params
  
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
    ceres::Problem problem;
    for (int i=0; i<N;i++)
    {
      problem.AddResidualBlock( // add error term; use auto derivative, module params: error type,output/input dimensions
	new ceres::AutoDiffCostFunction<CURVE_FITTING_COST,1,3>(
	  
	  new CURVE_FITTING_COST (x_data[i], y_data[i])
	), nullptr,abc); // no core function here, so empty, abc is params waited to be estimated
	
      
      
    }
    
    
    // config solver
    ceres::Solver::Options options; // mant config terms to fill out
    options.linear_solver_type = ceres::DENSE_QR; // how to solve delta x func
    options.minimizer_progress_to_stdout = true; //output to cout
    
    ceres::Solver::Summary summary; //optimization info
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve(options,&problem,&summary);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;    
   
    
    //output result
    cout<<summary.BriefReport()<<endl;
    cout<<"etimated abc =";
    for (auto a:abc) cout<<a<<" ";
    cout<<endl;
    
    return 0;
    
  }
