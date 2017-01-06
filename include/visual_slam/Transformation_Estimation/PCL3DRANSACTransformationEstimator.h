#ifndef PCL3DRANSACTransformationEstimator_H
#define PCL3DRANSACTransformationEstimator_H

#include <visual_slam/Transformation_Estimation/TransformationEstimator.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/common/time.h>
class PCL3DRANSACTransformationEstimator : public TransformationEstimator
{


  // TransformationEstimator interface
public:
  void estimateTransformation(PointCloudT::Ptr, FeatureCloudT::Ptr, PointCloudT::Ptr, FeatureCloudT::Ptr, TFMatrix &);
};

#endif // PCL3DRANSACTransformationEstimator_H
