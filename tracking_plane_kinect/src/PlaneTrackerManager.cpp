

static void cvMat2VpImage(const cv::Mat &frame, vpImage<unsigned char> &get_rgb_image,vpImage<unsigned char> &get_image){
  if ( (frame.rows!=get_rgb_image.getRows()) || (frame.rows!=get_rgb_image.getRows()) ){
    throw std::exception("cvMat2VpImage: images dimension mismatch");
  }
  for (int i = 0; i < frame.rows; ++i)
    {
      for (int j = 0; j < frame.cols; ++j)
	{
	  get_rgb_image[i][j].R = frame.data[3*(w*i +j)+0];
	  get_rgb_image[i][j].G = frame.data[3*(w*i +j)+1];
	  get_rgb_image[i][j].B = frame.data[3*(w*i +j)+2];
	}
    }
  vpImageConvert::convert(get_rgb_image,get_image);
}

static void initialize_object_model(const double height, const double width, vector<vpPoint> get_real_object){
  get_real_object.push_back(vpPoint());
  get_real_object.push_back(vpPoint());
  get_real_object.push_back(vpPoint());
  get_real_object.push_back(vpPoint());
  get_real_object[0].setWorldCoordinates (-width/2, -height/2, 0);
  get_real_object[1].setWorldCoordinates ( width/2, -height/2, 0);
  get_real_object[2].setWorldCoordinates ( width/2,  height/2, 0);
  get_real_object[3].setWorldCoordinates (-width/2, height/2, 0);
}


PlaneTrackerManager::PlaneTrackerManager(const double object_height, const double object_width, const int px, const int py, cv::Mat &frame){
  this->camera = vpCameraParameters(px, py, frame.cols/2, frame.rows/2);
  this->rgb_image.resize(frame.rows,frame.cols);
  cvMat2VpImage(this->frame,this->rgb_image,this->image);
  this->display.init(this->image, 100, 100, "Plane Tracker");
  initialize_object_model(object_height, object_width, this->real_object);
  this->plane_tracker = PlaneTracker(this->camera,this->real_object,this->pixel_points,this->image)
}

PlaneTrackerManager::track(cv::Mat &frame,vpHomogeneousMatrix &update_cMo, double* update_orientation){
  static PointsTracker pointsTracker;
  static vpImagePoint buffer;
  cvMat2VpImage(this->frame,this->rgb_image,this->image);
  if(pointsTracker.get_size()<4){
    if (vpDisplay::getClick(this->image,buffer, false))
      {
	pointsTracker.set_frame(frame,buffer.get_u(),buffer.get_v());
      }
    else 
      {
	pointsTracker.set_frame(frame);
      }
  } else {
    if (this->plane_tracker==NULL){
      this->plane_tracker = new PlaneTracker(this->camera,this->real_object,pointsTracker.get_points(),this->image);
    }
    this->plane_tracker.track(this->image,update_cMo,update_orientation);
  }
}




