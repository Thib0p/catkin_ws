#include "PlaneTracker.h"
#include "PointsTracker.h"


class PlaneTrackerManager {
 public:
  PlaneTrackerManager(const double object_height, const double object_width,
					   const int px, const int py, cv::Mat &frame);
  track(cv::Mat &frame,vpHomogeneousMatrix &update_cMo, double* update_orientation);
 private:
  vpCameraParameters camera;
  vpImage<vpRGBa> rgb_image;
  vpImage<unsigned char> image;
  vpDisplayX display;
  vector<vpPoint> real_object;
  vector<vpDot2> pixel_points;
  PlaneTracker *plane_tracker;
};

