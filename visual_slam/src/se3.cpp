#include "se3.h"

visual_slam::SE3::SE3()
{
  _tm = TFMatrix::Identity();
}
