#ifndef EIGENUTILITIES_H
#define EIGENUTILITIES_H
#include "definitions.h"
#include <Eigen/Geometry>

namespace visual_slam {

class EigenUtilites{
public:
  static EigenUtilites *instance()
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
  tf::Transform eigenTransf2TF(const T& transf)
  {
      tf::Transform result;
      tf::Vector3 translation;
      translation.setX(transf.translation().x());
      translation.setY(transf.translation().y());
      translation.setZ(transf.translation().z());

      tf::Quaternion rotation;
      Eigen::Quaterniond quat;
      quat = transf.rotation();
      rotation.setX(quat.x());
      rotation.setY(quat.y());
      rotation.setZ(quat.z());
      rotation.setW(quat.w());

      result.setOrigin(translation);
      result.setRotation(rotation);
      //printTransform("from conversion", result);
      return result;
  }
private:
  static EigenUtilites * s_instance;
  EigenUtilites();

};

}
#endif // EIGENUTILITIES_H
