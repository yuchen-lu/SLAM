// Kdevelop add argument should be quoted ""




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
    cerr<<"file"<<argv[1]<<"not exist"<<endl;
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
  
  //time using std::chrono
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  for ( size_t y=0; y<image.rows;y++)
  {
    for ( size_t x=0; x<image.cols; x++)
    {
      // visit x,y pixel
      //using cv::Mat::ptr to get image ptr
      unsigned char* row_ptr = image.ptr<unsigned char> (y); // row_ptr is yth row's row_ptr
      unsigned char* data_ptr = &row_ptr[x*image.channels()]; // data_ptr points to next visiting pixel data
      //output every channel in the pixel, if greyscale, output one channel
      for ( int c=0; c!= image.channels(); c++)
      {
	
	unsigned char data = data_ptr[c]; // data  is cth channel value in I(x,y)

      }

      
    }
   
    
  }
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
  cout<<"scan image used time :"<<time_used.count()<<"second(s)"<<endl;
  
  
  
  
  
  
  
 // about copying cv::Mat, direct giving value doesn't copy data
  cv::Mat image_another = image;
  //edit image_another will change image 
  image_another ( cv::Rect (0,0,100,100)).setTo(0); // zero top left 100*100
  cv::imshow("image",image);
  cv::waitKey(0);
  
  
  
  
  
  // clone data using clone function
  cv::Mat image_clone = image.clone();
  image_clone (cv::Rect(0,0,100,100)).setTo(255);
  cv::imshow("image",image);
  cv::imshow("image_clone", image_clone);
  cv::waitKey(0);
  
  cv::destroyAllWindows();
  return 0;
  
  
  
}


