

PointsTracker::PointsTracker(){}
PointsTracker::~PointsTracker(){}
void PointsTracker::set_frame(cv::Mat &frame){
  update_points(frame);
  this->previous_frame = frame.clone();
}
void PointsTracker::set_frame(cv::Mat &frame, float x, float y){
  points.push_back(cv::Point2f(x,y));
  update_points(frame);
  this->previous_frame = frame.clone();
}
std::vector<cv::Point2f> PointsTracker::get_points(){
  return this->previous_frame;
}
int PointsTracker::get_status(){
  return this->status;
}
int PointsTracker::get_size(){
  return this->points.size();
}
cv::Mat* get_error_matrix(){
  return &(this->error);
}
void PointsTracker::update_points(cv::Mat &frame){
  if(this->previous_frame.data!=NULL){
    cv::calcOpticalFlowPyrLK(
			     this->previous_frame, frame, // 2 consecutive images
			     this->points, // input point positions in first im
			     this->updated_points, // output point positions in the 2nd
			     this->status,    // tracking success
			     this->error      // tracking error
			     );
    this->points = this->updated_points;
  }
}
