#include "Feature_Extraction/CVORBFeatureExtractorAndDescriptor.h"


visual_slam::CVORBFeatureExtractorAndDescriptor::CVORBFeatureExtractorAndDescriptor()
{
    orb = cv::ORB::create();
}


void visual_slam::CVORBFeatureExtractorAndDescriptor::computeDescriptors(FrameData pFrameData , std::vector<cv::KeyPoint>& tkeypoint ,cv::Mat& tdescriptors)
{
    cv::Mat img = pFrameData.getFrameMatrix();
    orb->detectAndCompute(img, cv::noArray(),tkeypoint, tdescriptors);
}
