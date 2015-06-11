class PointsTracker {
public:
  PointsTracker();
  ~PointsTracker();
  void set_frame(cv::Mat frame);
  void set_frame(cv::Mat frame, float x, float y);
  std::vector<cv::Point2f> get_points();
  int get_status();
  cv::Mat* get_error_matrix();
private:
  void update_points(cv::Mat frame);
  std::vector<cv::Point2f> updated_points;
  std::vector<cv::Point2f> points;
  std::vector<uchar> status;
  cv::Mat previous_frame;
  cv::Mat error;
};
