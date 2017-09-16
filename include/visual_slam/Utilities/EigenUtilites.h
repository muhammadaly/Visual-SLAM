#ifndef EIGENUTILITIES_H
#define EIGENUTILITIES_H

#include "definitions.h"
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>

namespace visual_slam {

class EigenUtilites
{
  double max_translation_meter,max_rotation_degree,min_translation_meter,min_rotation_degree;
  static EigenUtilites * s_instance;
  EigenUtilites();

public:
  static EigenUtilites* instance()
  {
    if (!s_instance)
      s_instance = new EigenUtilites;
    return s_instance;
  }
  FQuarterionRotation ExtractRotationMatrixAsQuaternion(DTFMatrix);
  FTranslatonVec ExtractTranslationVector(DTFMatrix);
  bool isBigTranfo(const Eigen::Matrix4f);
  bool isSmallTrafo(const Eigen::Isometry3d& t, double seconds);
  void mat2RPY(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw);
  void mat2dist(const Eigen::Matrix4f& t, double &dist);
  void trafoSize(const Eigen::Isometry3d& t, double& angle, double& dist);
  template <typename T >
  tf::Transform eigenTransf2TF(const T& transf);
};

}
#endif // EIGENUTILITIES_H
