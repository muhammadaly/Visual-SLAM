#include "Utilities/EigenUtilites.h"

visual_slam::EigenUtilites* visual_slam::EigenUtilites::s_instance = NULL;

visual_slam::FQuarterionRotation visual_slam::EigenUtilites::ExtractRotationMatrixAsQuaternion(DTFMatrix transformationMatrix)
{
  FRotationMatrix rotationMatrix;

  rotationMatrix(0,0) = (float)transformationMatrix(0,0);
  rotationMatrix(0,1) = (float)transformationMatrix(0,1);
  rotationMatrix(0,2) = (float)transformationMatrix(0,2);

  rotationMatrix(1,0) = (float)transformationMatrix(1,0);
  rotationMatrix(1,1) = (float)transformationMatrix(1,1);
  rotationMatrix(1,2) = (float)transformationMatrix(1,2);

  rotationMatrix(2,0) = (float)transformationMatrix(2,0);
  rotationMatrix(2,1) = (float)transformationMatrix(2,1);
  rotationMatrix(2,2) = (float)transformationMatrix(2,2);

  return FQuarterionRotation(rotationMatrix);
}
visual_slam::FTranslatonVec visual_slam::EigenUtilites::ExtractTranslationVector(DTFMatrix transformationMatrix)
{
  visual_slam::FTranslatonVec translationVec;
  translationVec(0) = transformationMatrix(0,3);
  translationVec(1) = transformationMatrix(1,3);
  translationVec(2) = transformationMatrix(2,3);
  return translationVec;
}

bool visual_slam::EigenUtilites::isBigTranfo(const Eigen::Matrix4f t)
{
  double roll, pitch, yaw, dist;

  mat2RPY(t, roll,pitch,yaw);
  mat2dist(t, dist);

  roll = roll/M_PI*180;
  pitch = pitch/M_PI*180;
  yaw = yaw/M_PI*180;

  double max_angle = std::max(roll,std::max(pitch,yaw));

  return (dist > min_translation_meter
      || max_angle > min_rotation_degree);
}

void visual_slam::EigenUtilites::mat2RPY(const Eigen::Matrix4f &t, double &roll, double &pitch, double &yaw)
{
  roll = atan2(t(2,1),t(2,2));
  pitch = atan2(-t(2,0),sqrt(t(2,1)*t(2,1)+t(2,2)*t(2,2)));
  yaw = atan2(t(1,0),t(0,0));
}

void visual_slam::EigenUtilites::mat2dist(const Eigen::Matrix4f &t, double &dist)
{
  dist = sqrt(t(0,3)*t(0,3)+t(1,3)*t(1,3)+t(2,3)*t(2,3));
}

bool visual_slam::EigenUtilites::isSmallTrafo(const Eigen::Isometry3d &t, double seconds)
{
  if(seconds <= 0.0){
    ROS_WARN("Time delta invalid: %f. Skipping test for small transformation", seconds);
    return true;
  }

  double angle_around_axis, dist;
  trafoSize(t, angle_around_axis, dist);

  return (dist / seconds < max_translation_meter &&
          angle_around_axis / seconds < max_rotation_degree);
}

void visual_slam::EigenUtilites::trafoSize(const Eigen::Isometry3d &t, double &angle, double &dist)
{
  angle = acos((t.rotation().trace() -1)/2 ) *180.0 / M_PI;
  dist = t.translation().norm();
  ROS_INFO("Rotation:% 4.2f, Distance: % 4.3fm", angle, dist);
}

visual_slam::EigenUtilites::EigenUtilites()
{

}

template <typename T >
tf::Transform visual_slam::EigenUtilites::eigenTransf2TF(const T& transf)
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