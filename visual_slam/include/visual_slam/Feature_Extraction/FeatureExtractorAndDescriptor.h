#ifndef FEATUREEXTRACTORANDDESCRIPTOR_H
#define FEATUREEXTRACTORANDDESCRIPTOR_H

#include "opencv2/core.hpp"
#include "framedata.h"

class FeatureExtractorAndDescriptor
{
public:
    virtual void computeDescriptors(FrameData pFrameData , std::vector<cv::KeyPoint>& tkeypoint ,cv::Mat& tdescriptors)=0;
};

#endif // FEATUREEXTRACTORANDDESCRIPTOR_H
