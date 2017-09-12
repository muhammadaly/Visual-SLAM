#include "Feature_Extraction/CVSIFTFeatureExtractorAndDescriptor.h"

visual_slam::CVSIFTFeatureExtractorAndDescriptor::CVSIFTFeatureExtractorAndDescriptor()
{
  sift = cv::xfeatures2d::SIFT::create();
}

void visual_slam::CVSIFTFeatureExtractorAndDescriptor::computeDescriptors(FrameData pFrameData, std::vector<cv::KeyPoint> &tkeypoint, cv::Mat &tdescriptors)
{
  cv::Mat img = pFrameData.getFrameMatrix();
  sift->detectAndCompute(img, cv::noArray(),tkeypoint, tdescriptors);
}
