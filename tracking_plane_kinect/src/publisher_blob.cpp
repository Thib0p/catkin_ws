#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include <visp/vpDisplay.h>
#include <visp/vpDisplayX.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpServo.h>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpPoint.h>
#include <visp/vpException.h>
#include <visp/vpTemplateTrackerSSDInverseCompositional.h>
#include <visp/vpTemplateTrackerWarpHomography.h>
#include <visp/vpTime.h>
#include <visp/vpDot2.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpPose.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

/* Visp structure that contains all the variables linked to the visp library AND the image processing data*/
struct Visp{
  vpImage<unsigned char> I ;
  vpImage<vpRGBa> Irgb;
  vpImagePoint p;
  int pointSet;
  cv::Point pointcv;
  bool depth_ready;
  vpTemplateTrackerWarpHomography warp;
  vpTemplateTrackerSSDInverseCompositional *tracker;
  std::vector< vpImagePoint >    points2D_list;
  vpColVector    newPoints;
  std::vector< vpPoint >     points3D_list; 
  vpDisplayX display;
  std::vector<cv::Point2f> features_prev, features_next;
  std::vector<uchar> status;
  std::vector<vpPoint> realWorldpoints;
  vpHomogeneousMatrix cMo;
  vpCameraParameters *cam;
  std::vector<vpDot2> imageDots;
  double x,y,z,w;
  cv_bridge::CvImageConstPtr image;
  // Instantiate and get the reference zone
  vpTemplateTrackerZone zone_ref;
// Instantiate a warped zone
  vpTemplateTrackerZone zone_warped;
};




void toQuat(Visp &myVisp);
void computePose(std::vector<vpPoint> &point, const std::vector<vpDot2> &dot,
  const vpCameraParameters &cam, int init, vpHomogeneousMatrix &cMo);





/*This class will publish the marker datas*/
class Publisher{
public:
 Publisher()
 {


   double L = 0.101,l=0.101;
   myVisp.tracker = new vpTemplateTrackerSSDInverseCompositional(&(myVisp.warp));
   myVisp.tracker->setSampling(2, 2);
   myVisp.tracker->setLambda(0.001);
   myVisp.tracker->setIterationMax(200);
   myVisp.tracker->setPyramidal(2, 1);
   myVisp.points3D_list.push_back(vpPoint());
   myVisp.points3D_list.push_back(vpPoint());
   myVisp.points3D_list.push_back(vpPoint());
   myVisp.points3D_list.push_back(vpPoint());
   myVisp.realWorldpoints.push_back(vpPoint());
   myVisp.realWorldpoints.push_back(vpPoint());
   myVisp.realWorldpoints.push_back(vpPoint());
   myVisp.realWorldpoints.push_back(vpPoint());
   myVisp.realWorldpoints[0].setWorldCoordinates (-l/2, -L/2, 0);
   myVisp.realWorldpoints[1].setWorldCoordinates ( l/2, -L/2, 0);
   myVisp.realWorldpoints[2].setWorldCoordinates ( l/2,  L/2, 0);
   myVisp.realWorldpoints[3].setWorldCoordinates (-l/2, L/2, 0);
   myVisp.cam = new vpCameraParameters(525, 525, 320, 240);
   myVisp.pointSet=0;
   myVisp.Irgb.resize(480,640);
   vpImage<unsigned char> I ;
   vpImage<vpRGBa> Irgb;
   vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
   initialized=0;
   initialized2=0;
   init_pose=0;
   opticaldone=0;
   h=480;
   w=640;
 }
  /*The Callback function called when an image is avaible on the topic we are listening*/
 void subCallBack(const sensor_msgs::ImageConstPtr& msg)
 {

 myVisp.image = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::BGR8); // This function link the message image withe the image object in opencv
 vpImagePoint buf;

  /*After the plane that will be tracked is defined, the tracking is initialized*/
 if(myVisp.pointSet==4)
 {
  if(initialized2==0)
  {
    for (int i = 0; i < 3; ++i)
    {
      (myVisp.points2D_list).push_back(vpImagePoint((myVisp.features_next)[i].y,(myVisp.features_next)[i].x));
    }
    (myVisp.points2D_list).push_back(vpImagePoint((myVisp.features_next)[2].y,(myVisp.features_next)[2].x));
    (myVisp.points2D_list).push_back(vpImagePoint((myVisp.features_next)[3].y,(myVisp.features_next)[3].x));
    (myVisp.points2D_list).push_back(vpImagePoint((myVisp.features_next)[0].y,(myVisp.features_next)[0].x));
    (myVisp.tracker)->initFromPoints(myVisp.I,
      myVisp.points2D_list 
      );
    myVisp.zone_ref = myVisp.tracker->getZoneRef();
    initialized2=1;   
  }
}








/* As soon as an image is available from the kineect, it is converted to a gray one to provide to the tracker*/
if(initialized==1 && initialized2 ==0)
{
  frame_old = frame_gray.clone();
}

/*let's create an object easily manipulable*/
frame.create(h, w, CV_8UC3); 
memcpy(frame.data, myVisp.image->image.data, 3*h*w*sizeof(uint8_t)); 
frame.convertTo(frame, CV_8U); 
cv::cvtColor(frame,frame,CV_BGR2RGB);
   /* No need to convert the gray image once the plane has been defined so the conversion is desactivated as soon as the 4 points are defined */
if(initialized2 ==0)
{
  cvtColor( frame, frame_gray, CV_BGR2GRAY ); 
}
if(myVisp.pointSet<4)
{
  try
  {

    myVisp.features_prev = myVisp.features_next;
    if((myVisp.features_prev).size()>0)
      cv::calcOpticalFlowPyrLK(
      frame_old, frame_gray, // 2 consecutive images
      myVisp.features_prev, // input point positions in first im
      myVisp.features_next, // output point positions in the 2nd
      myVisp.status,    // tracking success
      err      // tracking error
      );
    opticaldone=1;
  }
  catch(...)
  {

  }
}
/* the clicked points are shown on the image*/
if(myVisp.pointSet<4)
{
  for (int i = 0; i < (myVisp.features_next).size(); ++i)
  {
    cv::Point a;
    a.x=floor((myVisp.features_next)[i].x);
    a.y=floor((myVisp.features_next)[i].y);
    circle(frame,a,3, cv::Scalar( 255, 0, 0 ),-1, 8 );
  }
}
for (int i = 0; i < h; ++i)
{
  for (int j = 0; j < w; ++j)
  {
   (myVisp.Irgb)[i][j].R = frame.data[3*(w*i +j)+0];
   (myVisp.Irgb)[i][j].G = frame.data[3*(w*i +j)+1];
   (myVisp.Irgb)[i][j].B = frame.data[3*(w*i +j)+2];
 }
}
vpImageConvert::convert((myVisp.Irgb), (myVisp.I));
if(initialized==0)
{
  myVisp.display.init(myVisp.I, 100, 100, "Tracker");
}

vpDisplay::display(myVisp.I);

/*If the 4 points have been defined...*/
if(initialized2==1)
{
/*Informations about the features point are updated*/

  /*If the initial pose has not been computed...*/
  if(init_pose==0)
  {
    for (int i = 0; i < 4; ++i)
    {
      myVisp.imageDots.push_back(vpDot2());
    }
    for (int i = 0; i < 4; ++i)
    {
      myVisp.imageDots[i].initTracking(myVisp.I,vpImagePoint((myVisp.features_next)[i].y,(myVisp.features_next)[i].x));
    }    
  }
  for (int i = 0; i < 4; ++i)
  {
     myVisp.imageDots[i].setGraphics(true);
    myVisp.imageDots[i].track(myVisp.I);
  }
  /*The new pose is computed*/
  computePose(myVisp.realWorldpoints, myVisp.imageDots, *(myVisp.cam), init_pose, myVisp.cMo);
  /*The rotation information in the pose estimation matrix cMo have to be converted into quaternions for the markers*/
  toQuat(myVisp);
  /*The frame of the plane is added to the display*/
  vpDisplay::displayFrame(myVisp.I, myVisp.cMo, *(myVisp.cam), 0.05, vpColor::none, 3);
  init_pose=1;

  /*Creating the Marker that we will publish*/
  visualization_msgs::Marker marker;
  marker.header.frame_id = "camera_depth_optical_frame";
  marker.header.stamp = ros::Time();
  marker.ns = "book";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = myVisp.cMo[0][3]+0.027;
  marker.pose.position.y = myVisp.cMo[1][3];
  marker.pose.position.z = myVisp.cMo[2][3];
  marker.pose.orientation.x = myVisp.x;
  marker.pose.orientation.y = myVisp.y;
  marker.pose.orientation.z = myVisp.z;
  marker.pose.orientation.w = myVisp.w;
  marker.scale.x = 0.175;
  marker.scale.y = 0.175;
  marker.scale.z = 0.01;
  marker.color.a = 0.5; 
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  
/*The Marker is then published*/
  vis_pub.publish( marker );


}
vpDisplay::flush(myVisp.I);
if (vpDisplay::getClick(myVisp.I,buf, false))
{
  myVisp.pointSet++;
  myVisp.features_next.push_back(cv::Point2f(buf.get_u(), buf.get_v()));
  printf("x: %f               y: %f\n",buf.get_u(),buf.get_v());
}
initialized=1;












cv::waitKey(3);
}
private:
  ros::NodeHandle n;
  ros::Publisher vis_pub;
  int initialized;
  int h,w;
  cv::Mat frame,frame_old,frame_gray;
  cv::Mat err;
  struct Visp myVisp;
  int initialized2;
  int init_pose;
  int opticaldone;
};

















/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
 int main(int argc, char **argv)
 {

   ros::init(argc, argv, "talker");
   Publisher myPublisher;
   ros::NodeHandle n2;
   
   ros::Subscriber sub = n2.subscribe("/rgbd_capture/rgb/image_color", 5, &Publisher::subCallBack,&myPublisher);

   ros::spin();


   return 0;
 }


/*This function makes additional stuff to compute the pose of our plane*/
 void computePose(std::vector<vpPoint> &point, const std::vector<vpDot2> &dot,
   const vpCameraParameters &cam, int init, vpHomogeneousMatrix &cMo)
 {
  vpPose pose;     double x=0, y=0;
  /*The lines below enable to match the 3D points with the features based one*/
  for (unsigned int i=0; i < point.size(); i ++) {
    vpImagePoint buf(dot[i].getCog());
    vpPixelMeterConversion::convertPoint(cam, dot[i].getCog(), x, y);
    printf("x: %f, y: \n",x,y );
    point[i].set_x(x);
    point[i].set_y(y);
    pose.addPoint(point[i]);
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
  }
  pose.computePose(vpPose::VIRTUAL_VS, cMo);
  cMo.vpMatrix::print(std::cout,4);
}


/*This function convert a rotation matrix inside the visp objetc to a quaternion*/
void toQuat(Visp &myVisp)
{
  double trace =myVisp.cMo[0][0]+myVisp.cMo[1][1]+myVisp.cMo[2][2]+1;

  double S;
  if(trace>0)
  {
    S=0.5/sqrt(trace);
    myVisp.x=(myVisp.cMo[2][1]-myVisp.cMo[1][2])*S;
    myVisp.y=(myVisp.cMo[0][2]-myVisp.cMo[2][0])*S;
    myVisp.z=(myVisp.cMo[1][0]-myVisp.cMo[0][1])*S;
    myVisp.w=0.25/S;
  }
  else
  {
    int max=0;
    for (int i = 1; i < 3; ++i)
    {
      if(myVisp.cMo[i][i]>myVisp.cMo[max][max])
      {
        max=i;
      }
    }

    switch(max)
    {
      case 0:
      S=sqrt(1+myVisp.cMo[0][0]-myVisp.cMo[1][1]-myVisp.cMo[2][2])*2;
      myVisp.x=0.25*S;
      myVisp.y =(myVisp.cMo[0][1]+myVisp.cMo[1][0])/S;
      myVisp.z = (myVisp.cMo[0][2]+myVisp.cMo[2][0])/S;
      myVisp.w = (myVisp.cMo[1][2]+myVisp.cMo[2][1])/S;
      break;
      case 1:
      S=sqrt(1-myVisp.cMo[0][0]+myVisp.cMo[1][1]-myVisp.cMo[2][2])*2;
      myVisp.x=(myVisp.cMo[0][1]+myVisp.cMo[1][0])/S;
      myVisp.y =0.25*S;
      myVisp.z = (myVisp.cMo[1][2]+myVisp.cMo[2][1])/S;
      myVisp.w = (myVisp.cMo[0][2]+myVisp.cMo[2][0])/S;
      break;
      case 2:
      S=sqrt(1-myVisp.cMo[0][0]-myVisp.cMo[1][1]+myVisp.cMo[2][2])*2;
      myVisp.x=(myVisp.cMo[0][2]+myVisp.cMo[2][0])/S;
      myVisp.y =(myVisp.cMo[1][2]+myVisp.cMo[2][1])/S;
      myVisp.z = 0.25*S;
      myVisp.w = (myVisp.cMo[0][1]+myVisp.cMo[1][0])/S;
      break;
    }
  }
}