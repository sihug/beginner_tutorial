/*
 * disparity.cpp
 *
 *  Created on: Apr 10, 2014
 *      Author: adrl
 */
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <rosconsole/macros_generated.h>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <stdlib.h>
#include <stdio.h>
#include <iostream>


//static const std::string OPENCV_WINDOW = "Image window";

class DisparityImage
{
  ros::NodeHandle n;
  ros::NodeHandle nInfo;
  image_transport::ImageTransport imageTransporter;
  image_transport::Subscriber subImageL;
  image_transport::Subscriber subImageR;

  ros::CallbackQueue infoCbQueue;
  image_transport::Subscriber subInfoL;
  image_transport::Subscriber subInfoR;

  image_transport::Publisher  pubImageL;
  image_transport::Publisher  pubImageR;

  cv_bridge::CvImagePtr  cvImageL;
  cv_bridge::CvImagePtr  cvImageR;

//  sensor_msgs::CameraInfoPtr camInfoL;
//  sensor_msgs::CameraInfo camInfoR;



public:
  DisparityImage()
    : imageTransporter(n)
  {
	// Subscribe to input images
	  subImageL = imageTransporter.subscribe("/cam0/image_raw", 1, &DisparityImage::imageCbL, this);
	  subImageR = imageTransporter.subscribe("/cam1/image_raw", 1, &DisparityImage::imageCbR, this);

	  // Subscribe camera info
	  nInfo.setCallbackQueue(&infoCbQueue);
//	  subInfoL = nInfo.subscribe("/stereo/left/camera_info", 1, DisparityImage::infoCbL);

	  pubImageL = imageTransporter.advertise("/stereo/left/image_raw", 1);
	  pubImageR = imageTransporter.advertise("/stereo/right/image_raw", 1);

  }

  ~DisparityImage()
  {}

 // void infoCbL(const sensor_msgs::CameraInfoConstPtr& msg) { camInfoL = msg; }

  void imageCbL(const sensor_msgs::ImageConstPtr& msg)
  {
	  try
	  {
		  cvImageL = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	  }
	  catch (cv_bridge::Exception& e)
	  {
		  ROS_ERROR("cv_bridge exception: %s", e.what());
		  return;
	  }
  }
  void imageCbR(const sensor_msgs::ImageConstPtr& msg)
  {
 	  try
 	  {
 		  cvImageR = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
 	  }
 	  catch (cv_bridge::Exception& e)
 	  {
 		  ROS_ERROR("cv_bridge exception: %s", e.what());
 		  return;
 	  }
   }

  void disparity()
  {
	  int numDisparities = 112, windowSize = 5;
	  cv::StereoBM stereo(0, numDisparities, windowSize);
	  cv::Mat disparity;

	  stereo(cvImageL->image, cvImageR->image, disparity, CV_32F);

	  double minVal, maxVal;
	  cv::Point minLoc, maxLoc;

	  cv::minMaxLoc(disparity, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );

	  //std::cout << "min val : " << minVal << std::endl;
	  //std::cout << "max val: " << maxVal << std::endl;


	  cv::Mat disparity_8U;
	  disparity.convertTo(disparity_8U,CV_8U,255.0/(maxVal - minVal));

	    imshow( "Disparity", disparity_8U );
  }

  void calibration()
  {

  }


  void run()
  {
    infoCbQueue.callOne();
 //   std::cout << "camInfoL:  " << camInfoL.height << std::endl;

	  while (ros::ok())
	  {
		  ros::spinOnce();
		  cv::waitKey(3);

		  // Only Process if both pointers are allocated
		  if(cvImageL != 0 && cvImageR != 0)
		  {
			 // ROS_INFO("Publish left and right images");
			  pubImageL.publish(cvImageL->toImageMsg());
			  pubImageR.publish(cvImageR->toImageMsg());

			  DisparityImage::disparity();
		  }
	  }
  }

};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "disparity");
  DisparityImage di;
  di.run();

  return 0;
}

