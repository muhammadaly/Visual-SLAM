#ifndef PCLFEATUREEXTRACTION_H
#define PCLFEATUREEXTRACTION_H

#include <visual_slam/framedata.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/core/core.hpp>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class PCLFeatureExtraction
{
public:
  PCLFeatureExtraction();
  PointCloudT::Ptr fromCVKeypoint(std::vector<cv::KeyPoint> pKeypoints , FrameData pFrameData);
};

#endif // PCLFEATUREEXTRACTION_H
