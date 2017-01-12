#ifndef CVFLANNFEATUREMATCHER_H
#define CVFLANNFEATUREMATCHER_H

#include <visual_slam/Feature_Matching/FeatureMatcher.h>
#include <opencv2/features2d/features2d.hpp>

class CVFLANNFeatureMatcher : public FeatureMatcher
{
  // FeatureMatcher interface
public:
  void matching2ImageFeatures(cv::Mat, cv::Mat, std::vector<cv::DMatch> &);
private :
  cv::FlannBasedMatcher matcher;

  void filterMatches(std::vector<cv::DMatch> , std::vector<cv::DMatch>&);
};

#endif // CVFLANNFEATUREMATCHER_H
