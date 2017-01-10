#ifndef TRANSFORMATIONESTIMATOR_H
#define TRANSFORMATIONESTIMATOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <eigen3/Eigen/Core>

#include <visual_slam/definitions.h>

class TransformationEstimator
{
public:
    virtual void estimateTransformation(PointCloudT::Ptr, FeatureCloudT::Ptr ,PointCloudT::Ptr , FeatureCloudT::Ptr,  TFMatrix&)=0;
  Pose_6D operator+(Pose_6D const& pose, TFMatrix const& transformation){

  }
};

#endif // TRANSFORMATIONESTIMATOR_H
