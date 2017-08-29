#ifndef OPENCVUTILITIES_H
#define OPENCVUTILITIES_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "ros/ros.h"

namespace visual_slam {

class OpenCVUtilities
{
private:
  static OpenCVUtilities * s_instance;
  OpenCVUtilities();

public:
  static OpenCVUtilities *instance()
  {
    if (!s_instance)
      s_instance = new OpenCVUtilities;
    return s_instance;
  }
  void depthToCV8UC1(cv::Mat&, cv::Mat&);
};

}
#endif // OPENCVUTILITIES_H
