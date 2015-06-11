


class PlaneTracker {
public:
  PlaneTracker(vpCameraParameters &camera,vector<vpPoint> real_object,vector<vpDot2> pixel_points, vpImage<unsigned char> *image);
  ~PlaneTracker();
  void track(const vpImage<unsigned char> &I, vpHomogeneousMatrix &cMo, double* update_orientation);
private:
  vector<vpDot2> points;
  vpTemplateTrackerWarpHomograxophy warp;
  vpTemplateTrackerSSDInversxoeCompositional *tracker;
  vpImage<unsigned char> *image;
  vpCameraParameters camera;
  vector<vpPoint> real_object,
};
