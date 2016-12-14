#ifndef FEATUREEXTRACTORANDDESCRIPTOR_H
#define FEATUREEXTRACTORANDDESCRIPTOR_H

#include <opencv2/core/core.hpp>
#include "visual_slam/framedata.h"

class FeatureExtractorAndDescriptor
{
public:
    virtual cv::Mat computeDescriptors(FrameData pFrameData , std::vector<cv::KeyPoint>& tkeypoint ,cv::Mat& tdescriptors)=0;
};

#endif // FEATUREEXTRACTORANDDESCRIPTOR_H
