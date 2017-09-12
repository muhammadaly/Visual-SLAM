#ifndef PCLFEATUREDESCRIPTOR_H
#define PCLFEATUREDESCRIPTOR_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv2/core.hpp>
namespace visual_slam {
static const int ORBFeatureVectorLength = 32;
static const int BRISKFeatureVectorLength = 64;

typedef pcl::Histogram<ORBFeatureVectorLength> FeatureT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;

class PCLFeatureDescriptor
{
public:
  PCLFeatureDescriptor();

  FeatureCloudT::Ptr fromCVDescriptor(cv::Mat pDescriptors);
};
}
#endif // PCLFEATUREDESCRIPTOR_H
