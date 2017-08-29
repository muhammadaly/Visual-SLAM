#include "OpenCVUtilities.h"

OpenCVUtilities::OpenCVUtilities()
{

}

void visual_slam::OpenCVUtilities::depthToCV8UC1(cv::Mat& depth_img, cv::Mat& mono8_img){
  //Process images
  if(depth_img.type() == CV_32FC1){
    depth_img.convertTo(mono8_img, CV_8UC1, 100,0); //milimeter (scale of mono8_img does not matter)
  }
  else if(depth_img.type() == CV_16UC1){
    mono8_img = cv::Mat(depth_img.size(), CV_8UC1);
    cv::Mat float_img;
    depth_img.convertTo(mono8_img, CV_8UC1, 0.05, -25); //scale to 2cm
    depth_img.convertTo(float_img, CV_32FC1, 0.001, 0);//From mm to m(scale of depth_img matters)
    depth_img = float_img;
  }
  else {
    printMatrixInfo(depth_img, "Depth Image");
    ROS_ERROR_STREAM("Don't know how to handle depth image of type "<< openCVCode2String(depth_img.type()));
  }
}
