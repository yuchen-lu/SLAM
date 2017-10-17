#include <iostream>
#include <chrono>
using namespace std;

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main ( int argc, char**argv)
{
  
  //read argv[1] image 
  cv::Mat image;
  image =cv::imread(argv[1]); // cv::imread --function read specific path
  //check if read correctly
  if (image.data == nullptr) 
  {
    cerr<<"file"<<"argv[1]<<"not exist"<<endl;
    return 0; 
  }
  
  
  //output some basics
  cout<<"width is"<<image.cols<<"height is"<<image.rows<<"channel number is"<<image.channels()<<endl;
  cv::imshow( "image",image);
  cv::waitKey (0); //pause function, wait one key input
  
  //check image type
  if (image.type() !=CV_8UC1 && image.type() !=CV_8UC3 )
  {
    // image type not fit
    cout<<"please input a color or greyscale image"<<endl;
    return 0;
  }    
  
  
  
  
}