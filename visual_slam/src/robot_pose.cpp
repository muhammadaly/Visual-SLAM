#include "robot_pose.h"

Robot_Pose::Robot_Pose()
{

}

Robot_Pose::Robot_Pose(double Tx, double Ty, double Tz, double Rw, double Rroll, double Rpitch)
{

}

visual_slam::Robot_Pose visual_slam::Robot_Pose::operator *(const visual_slam::TFMatrix& transformation) const
{
  Robot_Pose result(*this);
//  result._t += _R*tr3._t;
//  result._R.angle()+= tr3._R.angle();
//  result._R.angle()=normalize_theta(result._R.angle());


  return result;
}

visual_slam::RobotRotationAngles3D visual_slam::Robot_Pose::getRotationMatrixFromHomogeneousTransformationMatrix(const visual_slam::TFMatrix& transformation_mat)
{
  visual_slam::RobotRotationAngles3D rotation_mat;

  rotation_mat(0,0) = transformation_mat(0,0);
  rotation_mat(0,1) = transformation_mat(0,1);
  rotation_mat(0,2) = transformation_mat(0,2);

  rotation_mat(1,0) = transformation_mat(1,0);
  rotation_mat(1,1) = transformation_mat(1,1);
  rotation_mat(1,2) = transformation_mat(1,2);

  rotation_mat(2,0) = transformation_mat(2,0);
  rotation_mat(2,1) = transformation_mat(2,1);
  rotation_mat(2,2) = transformation_mat(2,2);

  return rotation_mat;
}

visual_slam::RobotTranslation3D visual_slam::Robot_Pose::getTranslationVectorFromHomogeneousTransformationMatrix(const visual_slam::TFMatrix& transformation_mat)
{
  visual_slam::RobotTranslation3D translation_vec;

  translation_vec(0,0) = transformation_mat(0,3);
  translation_vec(1,0) = transformation_mat(1,3);
  translation_vec(2,0) = transformation_mat(2,3);

  return translation_vec;
}
