
/*This function convert a rotation matrix to a quaternion*/
static void toQuat(vpHomogeneousMatrix cMo, double &x, double &y, double &z, double &w)
{
  double trace =cMo[0][0]+cMo[1][1]+cMo[2][2]+1;
  double S;
  if(trace>0)
  {
    S=0.5/sqrt(trace);
    x=(cMo[2][1]-cMo[1][2])*S;
    y=(cMo[0][2]-cMo[2][0])*S;
    z=(cMo[1][0]-cMo[0][1])*S;
    w=0.25/S;
  }
  else
  {
    int max=0;
    for (int i = 1; i < 3; ++i)
    {
      if(cMo[i][i]>cMo[max][max])
      {
        max=i;
      }
    }

    switch(max)
    {
      case 0:
      S=sqrt(1+cMo[0][0]-cMo[1][1]-cMo[2][2])*2;
      x=0.25*S;
      y =(cMo[0][1]+cMo[1][0])/S;
      z = (cMo[0][2]+cMo[2][0])/S;
      w = (cMo[1][2]+cMo[2][1])/S;
      break;
      case 1:
      S=sqrt(1-cMo[0][0]+cMo[1][1]-cMo[2][2])*2;
      x=(cMo[0][1]+cMo[1][0])/S;
      y =0.25*S;
      z = (cMo[1][2]+cMo[2][1])/S;
      w = (cMo[0][2]+cMo[2][0])/S;
      break;
      case 2:
      S=sqrt(1-cMo[0][0]-cMo[1][1]+cMo[2][2])*2;
      x=(cMo[0][2]+cMo[2][0])/S;
      y =(cMo[1][2]+cMo[2][1])/S;
      z = 0.25*S;
      w = (cMo[0][1]+cMo[1][0])/S;
      break;
    }
  }
}




static void computePose(const std::vector<vpDot2> &pixel_points,
		  const vpCameraPaorameters &cam, std::vector<vpPoint> &real_object, vpHomogeneousMatrix &cMo)
 {
   static int init = 0;
   vpPose pose;
   double x=0, y=0;
  /*The lines below enable to match the 3D points with the features based one*/
   for (unsigned int i=0; i < pixel_points.size(); i ++) {
     vpImagePoint buf(dot[i].getCog());
     vpPixelMeterConversion::convertPoint(cam, dot[i].getCog(), x, y);
     pixel_points[i].set_x(x);
     pixel_points[i].set_y(y);
     pose.addPoint(pixel_points[i]);
  }
  /*Compute the initial pose of the plane*/
  if (init == 0) {
    vpHomogeneousMatrix cMo_dem;
    vpHomogeneousMatrix cMo_lag;
    pose.computePose(vpPose::DEMENTHON, cMo_dem);
    pose.computePose(vpPose::LAGRANGE, cMo_lag);
    double residual_dem = pose.computeResidual(cMo_dem);
    double residual_lag = pose.computeResidual(cMo_lag);
    if (residual_dem < residual_lag)
      cMo = cMo_dem;
    else
      cMo = cMo_lag;
  } else {
    pose.computePose(vpPose::VIRTUAL_VS, cMo);
  }
  cMo.vpMatrix::print(std::cout,4);
  
}





PlaneTracker::PlaneTracker(vpCameraParameters &camera,vector<vpPoint> real_object,vector<vpDot2> pixel_points, vpImage<unsigned char> *image){
  this->camera = camera;
  this->image = image;
  this->pixel_points = pixel_points;
  this->real_object = real_object;
  this->tracker = new vpTemplateTrackerSSDInverseCompositional(&(this->warp));
  this->tracker->initFromPoints(*(this->image),this->pixel_points);
  this->ref = this->tracker->getZoneRef();
}
PlaneTracker::~PlaneTracker(){
  delete(this->tracker);
}
PlaneTracker::void track(vpHomogeneousMatrix &cMo,double* update_orientation){
  static vpTemplateTrackerZone zone_warped;
  static vpTemplateTrackerTriangle triangle0,triangle1;
  static std::vector<vpImagePoint> corner0,corner1;
  static vpDot2 corners[4];
  /*Informations about the features point are updated*/
  this->tracker->track(I);
  vpColVector p = this->tracker->getp();
  this->warp.warpZone(this->zone_ref, p, zone_warped);
  zone_warped.getTriangle(0, triangle0);
  zone_warped.getTriangle(1, triangle1);
  triangle0.getCorners( corner0 );
  triangle1.getCorners( corner1 );
  corners[0]=vpDot2(corner0[0]);
  corners[1]=vpDot2(corner0[1]);
  corners[2]=vpDot2(corner0[2]);
  corners[3]=vpDot2(corner1[1]);
  /*The new pose is computed*/
  computePose(this->points, this->camera, this->real_object,myVisp.cMo);
  /*The rotation information in the pose estimation matrix cMo have to be converted into quaternions for the markers*/
  double x,y,z,w;
  toQuat(cMo,x,y,z,w);
  update_orientation[0]=x;
  update_orientation[1]=y;
  update_orientation[2]=z;
  update_orientation[3]=w;
}
