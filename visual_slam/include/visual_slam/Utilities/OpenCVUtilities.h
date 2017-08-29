#ifndef OPENCVUTILITIES_H
#define OPENCVUTILITIES_H

namespace visual_slam {

class OpenCVUtilities
{
public:
  static OpenCVUtilities *instance()
  {
    if (!s_instance)
      s_instance = new OpenCVUtilities;
    return s_instance;
  }
  void depthToCV8UC1(cv::Mat& float_img, cv::Mat& mono8_img);


private:
  static OpenCVUtilities * s_instance;
  OpenCVUtilities();

};

}
#endif // OPENCVUTILITIES_H
