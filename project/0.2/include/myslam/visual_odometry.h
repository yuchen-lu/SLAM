class VisualOdometry
{
public:
  typedef shared_ptr<VisualOdometry> Ptr;
  enum VOState {
    INITILIZING =1, OK =0, LOST
  };
  
  VOstate state_; // current VO status
  Map::Ptr map_; ////map with all frames and map points 
  Frame::Ptr ref_; //reference frame 
  Frame::Ptr curr_ // current frame 
  cv::
}