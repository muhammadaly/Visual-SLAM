#ifndef CVFLANNFEATUREMATCHER_H
#define CVFLANNFEATUREMATCHER_H

#include <visual_slam/Loop_Closure/LoopClosureDetector.h>
#include <opencv2/features2d/features2d.hpp>

class CVFeatureBasedLoopClosureDetector : public LoopClosureDetector
{
public :
  void detectScene(cv::Mat,int);
private:
  std::vector<std::pair<cv::Mat,int>> FeaturesMap;
  std::unique_ptr<FeatureMatcher> featureMatcher;
};

#endif // CVFLANNFEATUREMATCHER_H
