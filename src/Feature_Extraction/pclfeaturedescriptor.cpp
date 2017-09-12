#include "Feature_Extraction/pclfeaturedescriptor.h"

visual_slam::PCLFeatureDescriptor::PCLFeatureDescriptor()
{

}

visual_slam::FeatureCloudT::Ptr visual_slam::PCLFeatureDescriptor::fromCVDescriptor(cv::Mat pDescriptors)
{
  visual_slam::FeatureCloudT::Ptr FeaturePointCloud ( new visual_slam::FeatureCloudT);
  for(int KeyPointsInd = 0 ; KeyPointsInd < pDescriptors.rows ; KeyPointsInd++)
  {
      visual_slam::FeatureT PCLDescriptor;
      for(int ind = 0 ; ind < pDescriptors.cols; ind++)
      {
          PCLDescriptor.histogram[ind] = pDescriptors.at<unsigned char>(KeyPointsInd,ind);
      }
      FeaturePointCloud->points.push_back(PCLDescriptor);
  }
  return FeaturePointCloud;
}
