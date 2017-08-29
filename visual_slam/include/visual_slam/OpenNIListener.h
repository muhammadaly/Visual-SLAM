#ifndef OPENNILISTENER_H
#define OPENNILISTENER_H

#include "ros/ros.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace visual_slam {
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image,
                                                      sensor_msgs::PointCloud2> KinectSyncPolicy;

class OpenNIListener
{
public:
  OpenNIListener();
private:

  ros::NodeHandle _node;

  tf::TransformListener* tflistener_;
  tf::TransformBroadcaster tf_br_;

  message_filters::Synchronizer<KinectSyncPolicy>* kinect_sync_;

  message_filters::Subscriber<sensor_msgs::Image> *visua_sub_;
  message_filters::Subscriber<sensor_msgs::Image> *depth_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *cinfo_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> *odom_sub_;

  void setupsubscribers();
  void kinectCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                                       const sensor_msgs::ImageConstPtr& depth_img_msg,
                                       const sensor_msgs::PointCloud2ConstPtr& point_cloud);
  void cameraCallback(cv::Mat visual_img,
                                      PointCloudT::Ptr point_cloud,
                                      cv::Mat depth_mono8_img);
  void retrieveTransformations(std_msgs::Header depth_header, FrameData* frame);
  void processFrame(FrameData* new_node);
};

}

#endif // OPENNILISTENER_H
