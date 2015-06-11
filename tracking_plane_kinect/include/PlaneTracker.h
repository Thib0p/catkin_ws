
//http://www.irisa.fr/lagadic/visp/documentation/visp-2.10.0/tutorial-tracking-tt.html#tune_tt
#define TRACKER_SAMPLE_I 2
#define TRACKER_SAMPLE_J 2
#define TRACKER_GAIN 0.001
#define TRACKER_MAX_ITERATION 200

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
