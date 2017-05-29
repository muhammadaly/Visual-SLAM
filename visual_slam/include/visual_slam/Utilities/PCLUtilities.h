#include <opencv2/core/core.hpp>

#include "visual_slam/definitions.h"
#include "visual_slam/framedata.h"


class PCLUtilities{
public:
  PCLUtilities();
  static PCLUtilities & Instance()
  {
      // Since it's a static variable, if the class has already been created,
      // It won't be created again.
      // And it **is** thread-safe in C++11.

      static PCLUtilities myInstance;

      // Return a reference to our instance.
      return myInstance;
  }
  void getKeypointsAndDescriptors(std::vector<cv::DMatch> matches,
                                  std::vector<cv::KeyPoint> previousKeypoints, std::vector<cv::KeyPoint> currentKeypoints,
                                  cv::Mat previousDescriptors , cv::Mat currentDescriptors,
                                  FrameData previousFrameData, FrameData currentFrameData,
                                  PointCloudT::Ptr previousKeypointsPointCloud, PointCloudT::Ptr currentKeypointsPointCloud,
                                  FeatureCloudT::Ptr previousFeaturesPointCloud, FeatureCloudT::Ptr currentFeaturesPointCloud);
};
