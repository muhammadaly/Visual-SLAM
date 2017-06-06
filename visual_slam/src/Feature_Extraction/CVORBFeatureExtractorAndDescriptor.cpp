#include "Feature_Extraction/CVORBFeatureExtractorAndDescriptor.h"


CVORBFeatureExtractorAndDescriptor::CVORBFeatureExtractorAndDescriptor()
{
    orb = cv::ORB::create();
}


void CVORBFeatureExtractorAndDescriptor::computeDescriptors(FrameData pFrameData , std::vector<cv::KeyPoint>& tkeypoint ,cv::Mat& tdescriptors)
{
    cv::Mat img = pFrameData.getFrameMatrix();
    orb->detectAndCompute(img, cv::noArray(),tkeypoint, tdescriptors);
}
