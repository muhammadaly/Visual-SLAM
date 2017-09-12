#ifndef CVFLANNFEATUREMATCHER_H
#define CVFLANNFEATUREMATCHER_H

#include <Feature_Matching/FeatureMatcher.h>
#include <opencv2/features2d/features2d.hpp>

namespace visual_slam {

class CVFLANNFeatureMatcher : public FeatureMatcher
{
  // FeatureMatcher interface
public:
  void matching2ImageFeatures(cv::Mat, cv::Mat, std::vector<cv::DMatch> &);
private :
  cv::FlannBasedMatcher matcher;

  void filterMatches(std::vector<cv::DMatch> , std::vector<cv::DMatch>&);
};
}
#endif // CVFLANNFEATUREMATCHER_H
