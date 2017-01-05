#ifndef TRANSFORMATIONESTIMATOR_H
#define TRANSFORMATIONESTIMATOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <eigen3/Eigen/Core>

#include <visual_slam/definitions.h>

class TransformationEstimator
{
public:
    virtual void estimateTransformation(PointCloudT , PointCloudT, TFMatrix&)=0;
};

#endif // TRANSFORMATIONESTIMATOR_H
