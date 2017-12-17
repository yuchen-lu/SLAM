# include<opencv2/core/core.hpp>

int main (int argc, char** argv)
{
  if (argc!= 2)
  {
    cout<<"usage: run_vo parameter_file"<<endl;
    return 1;
  }
  
  myslam::Config::SeParametersFile (argv[1]);
  myslam::VisualOdometry::Ptr vo( new myslam::VisualOdometry );
  string dataset_Dir = myslam::Config::get<string> ("dataset_Dir");
  cout<<"dataset:  "<<dataset_dir<<endl;
  ifstream fin( dataset_dir+"/associate.txt");
  if ( !fin)
  {
    cout<<"please generate associate file!"<<endl;
    return 1;
    
    
  }
  
  vector<string> rgb_files, depth_files;
  vector<double> rgb_times, depth_times;
  while( !fin.eof() )
  {
    string rgb_time, rgb_file, depth_time, depth_file;
    fin>>rgb_time>>rgb_file>>depth_time>>depth_file;
    rgb_times.push_back(atof ( rgb_time.c_str()));
    depth_times.push_back ( atof( depth_time.c_str()));
    rgb_files.push_back( dataset_dir+"/"+rgb_file );
    depth_files.push_back( dataset_dir+"/"+depth_file);
    
    if( fin.good() == false)
      break;
    
  }
  myslam::Camera::Ptr camera ( new myslam::Camera);
  
  // visualization
  cv::viz::Viz3d vis("Visual Odometry");
  cv::viz::WCoordinateSystem world_coor(1,0), camera_coor(0.5);
  cv::Point3d cam_pos(0, -1.0, -1.0), camera_focal_point(0,0,0), cam_y_dir(0,1,0);
  cv::Affine3d cam_pose = cv::viz::makeCameraPose( cam_pos, camera_focal_point, cam_y_dir);
  vis.setViewerPose( cam_pose);
  
  world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
  cmera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
  vis.showWidget("World",world_coor);
  vis.showWidget("Camera", camera_coor);
  
  cout<<"read total"<<rgb_files_.size()<<" entries"<<endl;
  for ( int i=0; i<rgb_files.size(); i++)
  {
    Mat color - cv::imread(rgb_files[i]);
    Mat depth = cv::imread( dpth_files[i], -1);
    if (color.data==nullptr || depth.data == nullptr)
      break;
    myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
    pFrame->camera_ = camera;
    pFrame->color_ = color;
    pFrame->depth_ = depth;
    pFrame->timestamp_ = rgb_times[i];
    
    boost::timer timer;
    vo->addFrame (pFrame);
    cout<<"VO costs time:  "<timer.elapsed()<<endl;
    
    
    
  }