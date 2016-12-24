#ifndef FEATUREMATCHER_H
#define FEATUREMATCHER_H

#include <opencv2/core/core.hpp>
#include "visual_slam/framedata.h"

class FeatureMatcher
{
public:
    virtual cv::Mat matchFeatures(cv::Mat pPreviousDescriptors, cv::Mat pCurrentDescriptors)=0;
};

#endif // FEATUREMATCHER_H
