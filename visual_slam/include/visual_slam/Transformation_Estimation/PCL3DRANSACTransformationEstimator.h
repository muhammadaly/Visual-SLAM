#ifndef PCL3DRANSACTransformationEstimator_H
#define PCL3DRANSACTransformationEstimator_H

#include <Transformation_Estimation/TransformationEstimator.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/common/time.h>
class PCL3DRANSACTransformationEstimator : public TransformationEstimator
{
  // TransformationEstimator interface
public:
  bool estimateTransformation(visual_slam::PointCloudT::Ptr, visual_slam::FeatureCloudT::Ptr, visual_slam::PointCloudT::Ptr, visual_slam::FeatureCloudT::Ptr, visual_slam::TFMatrix & , visual_slam::PointCloudT::Ptr&);
};

#endif // PCL3DRANSACTransformationEstimator_H
