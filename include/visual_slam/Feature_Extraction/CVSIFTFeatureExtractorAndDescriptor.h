#ifndef CVSIFTFEATUREEXTRACTORANDDESCRIPTOR_H
#define CVSIFTFEATUREEXTRACTORANDDESCRIPTOR_H

#include "FeatureExtractorAndDescriptor.h"
#include <opencv2/features2d.hpp>

class CVSIFTFeatureExtractorAndDescriptor : public FeatureExtractorAndDescriptor
{
public:
    CVSIFTFeatureExtractorAndDescriptor();
    void computeDescriptors(FrameData pFrameData , std::vector<cv::KeyPoint>& tkeypoint ,cv::Mat& tdescriptors);
private:
    cv::Ptr<cv::Feature2D>  sift ;
};

#endif // CVSIFTFEATUREEXTRACTORANDDESCRIPTOR_H
