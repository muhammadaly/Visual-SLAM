#include "Feature_Extraction/CVSURFFeatureExtractorAndDescriptor.h"

CVSURFFeatureExtractorAndDescriptor::CVSURFFeatureExtractorAndDescriptor()
{
  double minHessian = 400.0;
  surf = cv::xfeatures2d::SURF::create( minHessian);
}

void CVSURFFeatureExtractorAndDescriptor::computeDescriptors(FrameData pFrameData, std::vector<cv::KeyPoint> &tkeypoint, cv::Mat &tdescriptors)
{
  cv::Mat img = pFrameData.getFrameMatrix();
  surf->detectAndCompute(img, cv::noArray(),tkeypoint, tdescriptors);
}
