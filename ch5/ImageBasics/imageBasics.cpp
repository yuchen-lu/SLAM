// summary of this program:
// read image, show image, read all pixels, copy, give values



// ----------highlight: Kdevelop add argument should be quoted ""


#include <iostream>
#include <chrono>
using namespace std;

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main ( int argc, char**argv)
{
  
  //read argv[1] image
  // image type declare
  cv::Mat image; 
  image =cv::imread(argv[1]); // cv::imread --function read specific path
  //check if read correctly
  if (image.data == nullptr) 
  {
    cerr<<"file"<<argv[1]<<"not exist"<<endl;
    return 0; 
  }
  
  
  //cv::mat properties: rows, cols, channels, type 
  cout<<"width is "<<image.cols<<"height is "<<image.rows<<
  "channel number is "<<image.channels()<<endl;
  cv::imshow( "image",image);
  cv::waitKey (0); //pause function, wait one key input
  
  
  
  //check image type
  //grayscale CV_8UC1 , 3 channel image of type CV_8UC3
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

      //---------------highlight: how to read each pixel in a loop
      // visit x,y pixel
      //using cv::Mat::ptr to get image ROW ptr
      // dont understand ??
      unsigned char* row_ptr = image.ptr<unsigned char> (y); // row_ptr is yth row's row_ptr    // mat ptr Returns a pointer to the specified matrix row. 
      unsigned char* data_ptr = &row_ptr[x*image.channels()]; // data_ptr points to next visiting pixel data
//  cout<<*data_ptr<<endl;
      
      //output every channel in the pixel, if greyscale, only output one channel
      for ( int c=0; c!= image.channels(); c++)
      {
	
	unsigned char data = data_ptr[c]; // data  is c-th channel value in I(x,y)

      }

      
    }
   
    
  }
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
  cout<<"scan image used time :"<<time_used.count()<<" second(s)"<<endl;
  
  
  
  
  
  
  
  // about copying cv::Mat, directly giving value doesn't copy data!
 
  cv::Mat image_another = image;
  //-----highlight-----edit image_another will change image!
  // has to use image.clone()
  
  
  image_another ( cv::Rect (0,0,100,100) ).setTo(0); // zero top left 100*100
  // 0 is black
  cv::imshow("image_another",image);
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


