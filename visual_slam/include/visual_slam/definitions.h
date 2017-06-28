#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Geometry>

namespace visual_slam {

  static const int ORBFeatureVectorLength = 32;
  static const int SIFTFeatureVectorLength = 128;

  typedef pcl::PointXYZRGB PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;
  typedef pcl::Histogram<SIFTFeatureVectorLength> FeatureT;
  typedef pcl::PointCloud<FeatureT> FeatureCloudT;
  typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> ColorHandlerT;
  typedef Eigen::Matrix4f TFMatrix;
  typedef Eigen::Matrix<double, 4, 4> DTFMatrix;
  typedef Eigen::Matrix3f FRotationMatrix;
  typedef Eigen::Vector3f FTranslatonVec;
  typedef Eigen::Isometry3d Pose_6D;
  typedef Eigen::Quaternionf FQuarterionRotation ;

  // For robot pose
  typedef Eigen::Matrix< double, 6, 1 > RobotPose6D;
  typedef Eigen::Vector3d RobotRotationAngles3D;
  typedef Eigen::Vector3d RobotTranslation3D;
  typedef Eigen::Matrix3d RobotRotationMatrix3D;

  static const float fx = 525.0;  // focal length x
  static const float fy = 525.0;  // focal length y
  static const float cx = 319.5;  // optical center x
  static const float cy = 239.5;  // optical center y
}


#endif // DEFINITIONS_H
