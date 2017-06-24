#ifndef FEATUREMATCHER_H
#define FEATUREMATCHER_H

#include <opencv2/core/core.hpp>
namespace visual_slam {
class FeatureMatcher
{
public:
    virtual void matching2ImageFeatures(cv::Mat, cv::Mat, std::vector<cv::DMatch>&) = 0;
};
}
#endif // FEATUREMATCHER_H
