#ifndef OPENNILISTENER_H
#define OPENNILISTENER_H

#include "ros/ros.h"

#include "definitions.h"
#include "framedata.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
namespace visual_slam {
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image,
                                                      sensor_msgs::PointCloud2> KinectSyncPolicy;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image,
                                                        sensor_msgs::CameraInfo> NoCloudSyncPolicy;

class OpenNIListener
{
public:
  OpenNIListener();
private:
  ros::NodeHandle _node;

  tf::TransformListener* tflistener_;
  tf::TransformBroadcaster tf_br_;

  message_filters::Synchronizer<KinectSyncPolicy>* kinect_sync_;
  message_filters::Synchronizer<NoCloudSyncPolicy>* no_cloud_sync_;

  message_filters::Subscriber<sensor_msgs::Image> *visua_sub_;
  message_filters::Subscriber<sensor_msgs::Image> *depth_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *cinfo_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> *odom_sub_;

  std::unique_ptr<FeatureExtractorAndDescriptor> featureExtractorAndDescriptor;

  cv::Mat depth_mono8_img_;
  std::string image_encoding_;
  int num_processed_;

  void setupsubscribers();
  void kinectCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                                       const sensor_msgs::ImageConstPtr& depth_img_msg,
                                       const sensor_msgs::PointCloud2ConstPtr& point_cloud);
  void cameraCallback(cv::Mat visual_img,
                                      PointCloudT::Ptr point_cloud,
                                      cv::Mat depth_mono8_img);
  void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg);

  void retrieveTransformations(std_msgs::Header depth_header, FrameData* frame);
  void processFrame(FrameData* new_node);
  void noCloudCameraCallback(cv::Mat visual_img,
                                             cv::Mat depth,
                                             cv::Mat depth_mono8_img,
                                             std_msgs::Header depth_header,
                                             const sensor_msgs::CameraInfoConstPtr& cam_info);
  void callProcessing(cv::Mat visual_img, FrameData* node_ptr);
  void noCloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                                        const sensor_msgs::ImageConstPtr& depth_img_msg,
                                        const sensor_msgs::CameraInfoConstPtr& cam_info_msg);
};

}

#endif // OPENNILISTENER_H
