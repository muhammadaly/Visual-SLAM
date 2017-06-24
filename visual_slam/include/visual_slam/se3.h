#ifndef SE3_H
#define SE3_H

#include <Eigen/Geometry>

namespace visual_slam {

typedef Eigen::Matrix4f TFMatrix;
typedef float Scalar;

class SE3
{
public:
  SE3();
  Robot_Pose& operator *= (const visual_slam::TFMatrix&);
  void transform(const visual_slam::TFMatrix&);
  void inverse();
private:
  TFMatrix _tm;
};

}
#endif // SE3_H
