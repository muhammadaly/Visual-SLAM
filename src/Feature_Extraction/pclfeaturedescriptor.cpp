#include "visual_slam/Feature_Extraction/pclfeaturedescriptor.h"

PCLFeatureDescriptor::PCLFeatureDescriptor()
{

}

FeatureCloudT::Ptr PCLFeatureDescriptor::fromCVDescriptor(cv::Mat pDescriptors)
{
  FeatureCloudT::Ptr FeaturePointCloud ( new FeatureCloudT);
  for(int KeyPointsInd = 0 ; KeyPointsInd < pDescriptors.rows ; KeyPointsInd++)
  {
      FeatureT PCLDescriptor;
      for(int ind = 0 ; ind < pDescriptors.cols; ind++)
      {
          PCLDescriptor.histogram[ind] = pDescriptors.at<unsigned char>(KeyPointsInd,ind);
      }
      FeaturePointCloud->points.push_back(PCLDescriptor);
  }
  return FeaturePointCloud;
}
