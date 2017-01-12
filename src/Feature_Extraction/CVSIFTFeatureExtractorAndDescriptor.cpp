#include "visual_slam/Feature_Extraction/CVSIFTFeatureExtractorAndDescriptor.h"

CVSIFTFeatureExtractorAndDescriptor::CVSIFTFeatureExtractorAndDescriptor()
{
  sift = features2d::SIFT::create();
}

void CVSIFTFeatureExtractorAndDescriptor::computeDescriptors(FrameData pFrameData, std::vector<cv::KeyPoint> &tkeypoint, cv::Mat &tdescriptors)
{
  cv::Mat img = pFrameData.getFrameMatrix();
  sift->detectAndCompute(img, cv::noArray(),tkeypoint, tdescriptors);
}
