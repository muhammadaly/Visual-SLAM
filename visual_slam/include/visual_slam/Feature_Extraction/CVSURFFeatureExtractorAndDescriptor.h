#ifndef CVSURFFEATUREEXTRACTORANDDESCRIPTOR_H
#define CVSURFFEATUREEXTRACTORANDDESCRIPTOR_H

#include "FeatureExtractorAndDescriptor.h"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
namespace visual_slam {
class CVSURFFeatureExtractorAndDescriptor : public FeatureExtractorAndDescriptor
{
public:
    CVSURFFeatureExtractorAndDescriptor();
    void computeDescriptors(FrameData pFrameData , std::vector<cv::KeyPoint>& tkeypoint ,cv::Mat& tdescriptors);
private:
    cv::Ptr<cv::xfeatures2d::SURF>  surf ;
};
}
#endif // CVSURFFEATUREEXTRACTORANDDESCRIPTOR_H
