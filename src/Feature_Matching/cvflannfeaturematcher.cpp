#include <visual_slam/Feature_Matching/cvflannfeaturematcher.h>

void CVFLANNFeatureMatcher::matching2ImageFeatures(cv::Mat previousImageDescriptors, cv::Mat currentImageDescriptors, std::vector<cv::DMatch> & matches)
{
  if (!currentImageDescriptors.empty() && !previousImageDescriptors.empty()) {
    if (currentImageDescriptors.type() != CV_32F) {
      currentImageDescriptors.convertTo(currentImageDescriptors, CV_32F);
    }
    if (previousImageDescriptors.type() != CV_32F) {
      previousImageDescriptors.convertTo(previousImageDescriptors, CV_32F);
    }
    matcher.match(currentImageDescriptors, previousImageDescriptors, matches);
//    std::vector<cv::DMatch> filtered_matches;
//    filterMatches(matches,filtered_matches);
  }
}

void CVFLANNFeatureMatcher::filterMatches(std::vector<cv::DMatch> matches, std::vector<cv::DMatch> & good_matches)
{
  double max_dist = 0;
  double min_dist = 100;

  for (int i = 0; i < matches.size(); i++) {
    double dist = matches[i].distance;
    if (dist < min_dist) min_dist = dist;
    if (dist > max_dist) max_dist = dist;
  }
  double thr = (50 * min_dist);
  for (int i = 0; i < matches.size(); i++) {
    if (matches[i].distance < thr) { good_matches.push_back(matches[i]); }
  }
}
