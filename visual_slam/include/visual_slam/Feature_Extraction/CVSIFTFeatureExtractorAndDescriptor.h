#ifndef CVSIFTFEATUREEXTRACTORANDDESCRIPTOR_H
#define CVSIFTFEATUREEXTRACTORANDDESCRIPTOR_H

#include "FeatureExtractorAndDescriptor.h"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"

class CVSIFTFeatureExtractorAndDescriptor : public FeatureExtractorAndDescriptor
{
public:
    CVSIFTFeatureExtractorAndDescriptor();
    void computeDescriptors(FrameData pFrameData , std::vector<cv::KeyPoint>& tkeypoint ,cv::Mat& tdescriptors);
private:
    cv::Ptr<cv::xfeatures2d::SIFT>  sift ;
};

#endif // CVSIFTFEATUREEXTRACTORANDDESCRIPTOR_H
