#ifndef TRANSFORMATIONESTIMATOR_H
#define TRANSFORMATIONESTIMATOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <eigen3/Eigen/Core>

#include <definitions.h>
namespace visual_slam {
class TransformationEstimator
{
public:
    virtual bool estimateTransformation(visual_slam::PointCloudT::Ptr, visual_slam::FeatureCloudT::Ptr, visual_slam::PointCloudT::Ptr,
                                        visual_slam::FeatureCloudT::Ptr,  visual_slam::TFMatrix&, visual_slam::PointCloudT::Ptr&)=0;
};
}
#endif // TRANSFORMATIONESTIMATOR_H
