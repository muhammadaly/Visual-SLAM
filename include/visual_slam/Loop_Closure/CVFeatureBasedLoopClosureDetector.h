#ifndef CVFLANNFEATUREMATCHER_H
#define CVFLANNFEATUREMATCHER_H

#include <visual_slam/Loop_Closure/LoopClosureDetector.h>
#include <opencv2/features2d/features2d.hpp>

class CVFeatureBasedLoopClosureDetector : public LoopClosureDetector
{
  public detectScene(cv::Mat);
private:
  std::vector<cv::Mat> FeaturesMap;
};

#endif // CVFLANNFEATUREMATCHER_H
