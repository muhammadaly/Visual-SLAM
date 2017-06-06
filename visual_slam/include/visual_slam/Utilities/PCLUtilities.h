#include <opencv2/core/core.hpp>

#include "definitions.h"
#include "framedata.h"


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
                                  visual_slam::PointCloudT::Ptr previousKeypointsPointCloud, visual_slam::PointCloudT::Ptr currentKeypointsPointCloud,
                                  visual_slam::FeatureCloudT::Ptr previousFeaturesPointCloud, visual_slam::FeatureCloudT::Ptr currentFeaturesPointCloud);
};
