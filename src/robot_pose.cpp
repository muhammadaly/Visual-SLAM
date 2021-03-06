#include "robot_pose.h"

visual_slam::Robot_Pose::Robot_Pose()
{
  _T = visual_slam::RobotTranslation3D(0.0, 0.0, 0.0);
  _R = visual_slam::RobotRotationMatrix3D::Identity();
}
visual_slam::Robot_Pose::Robot_Pose(double Tx, double Ty, double Tz, double Rroll, double Rpitch, double Ryaw)
{
  _T = visual_slam::RobotTranslation3D(Tx, Ty, Tz);
  _R = getRotationMatrixFromRotationAngles(visual_slam::RobotRotationAngles3D(Rroll,Rpitch,Ryaw));
}
void visual_slam::Robot_Pose::setRotationMatrix(visual_slam::RobotRotationMatrix3D r)
{
  _R = r;
}
void visual_slam::Robot_Pose::setTranslationVector(visual_slam::RobotTranslation3D t)
{
  _T = t;
}


visual_slam::Robot_Pose& visual_slam::Robot_Pose::operator *= (const visual_slam::TFMatrix& transformation)
{
  transform(transformation);
  return *this;
}
void visual_slam::Robot_Pose::transform(const visual_slam::TFMatrix& transformation)
{
  visual_slam::RobotRotationMatrix3D new_rotation_mat = getRotationMatrixFromHomogeneousTransformationMatrix(transformation);
  visual_slam::RobotTranslation3D new_translation_vec = getTranslationVectorFromHomogeneousTransformationMatrix(transformation);

  _R *= new_rotation_mat;
  _T += new_rotation_mat * new_translation_vec;
}
visual_slam::RobotRotationMatrix3D visual_slam::Robot_Pose::getRotationMatrixFromHomogeneousTransformationMatrix(const visual_slam::TFMatrix& transformation_mat)
{
  visual_slam::RobotRotationMatrix3D rotation_mat;

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

visual_slam::RobotRotationAngles3D visual_slam::Robot_Pose::getRotationAnglesFromRotationMatrix(const visual_slam::RobotRotationMatrix3D& rotation_mat)
{
  float sy = sqrt(rotation_mat(0,0) * rotation_mat(0,0) +  rotation_mat(1,0) * rotation_mat(1,0));
  bool singular = (sy < 1e-6);
  float x, y, z;
  if (!singular)
  {
    x = atan2(rotation_mat(2,1) , rotation_mat(2,2));
    y = atan2(-rotation_mat(2,0), sy);
    z = atan2(rotation_mat(1,0), rotation_mat(0,0));
  }
  else
  {
    x = atan2(-rotation_mat(1,2), rotation_mat(1,1));
    y = atan2(-rotation_mat(2,0), sy);
    z = 0;
  }
  return visual_slam::RobotRotationAngles3D(x, y, z);
}
visual_slam::RobotRotationMatrix3D visual_slam::Robot_Pose::getRotationMatrixFromRotationAngles(const visual_slam::RobotRotationAngles3D& rotation_angles)
{
  visual_slam::RobotRotationMatrix3D R;
  R = Eigen::AngleAxisd(rotation_angles(2), visual_slam::RobotRotationAngles3D::UnitZ())
    * Eigen::AngleAxisd(rotation_angles(1), visual_slam::RobotRotationAngles3D::UnitY())
    * Eigen::AngleAxisd(rotation_angles(0), visual_slam::RobotRotationAngles3D::UnitZ());
  return R;
}

visual_slam::RobotPose6D visual_slam::Robot_Pose::getRobotPose()
{
  visual_slam::RobotPose6D p;
  p(0) = _T(0);
  p(1) = _T(1);
  p(2) = _T(2);

  visual_slam::RobotRotationAngles3D rotation_angles = getRotationAnglesFromRotationMatrix(_R);
  p(3) = rotation_angles(0);
  p(4) = rotation_angles(1);
  p(5) = rotation_angles(2);
  return p;
}
visual_slam::TFMatrix visual_slam::Robot_Pose::getTransformationMatrix()
{
  visual_slam::TFMatrix tf = visual_slam::TFMatrix::Zero();

  tf(0,0) = _R(0,0);
  tf(0,1) = _R(0,1);
  tf(0,2) = _R(0,2);
  tf(0,3) = _T(0);

  tf(1,0) = _R(1,0);
  tf(1,1) = _R(1,1);
  tf(1,2) = _R(1,2);
  tf(1,3) = _T(1);

  tf(2,0) = _R(2,0);
  tf(2,1) = _R(2,1);
  tf(2,2) = _R(2,2);
  tf(2,3) = _T(2);

  tf(3,3) = 1.0;
  return tf;
}
visual_slam::TFMatrix visual_slam::Robot_Pose::getTransformationMatrix(const visual_slam::RobotRotationMatrix3D& rotation_mat, const visual_slam::RobotTranslation3D& translation_vec)
{
  visual_slam::TFMatrix tf = visual_slam::TFMatrix::Zero();

  tf(0,0) = rotation_mat(0,0);
  tf(0,1) = rotation_mat(0,1);
  tf(0,2) = rotation_mat(0,2);
  tf(0,3) = translation_vec(0);

  tf(1,0) = rotation_mat(1,0);
  tf(1,1) = rotation_mat(1,1);
  tf(1,2) = rotation_mat(1,2);
  tf(1,3) = translation_vec(1);

  tf(2,0) = rotation_mat(2,0);
  tf(2,1) = rotation_mat(2,1);
  tf(2,2) = rotation_mat(2,2);
  tf(2,3) = translation_vec(2);

  tf(3,3) = 1.0;
  return tf;
}
visual_slam::TFMatrix visual_slam::Robot_Pose::getTransformationMatrix(double Tx, double Ty, double Tz, double Rroll, double Rpitch, double Ryaw)
{
  visual_slam::RobotTranslation3D t = visual_slam::RobotTranslation3D(Tx, Ty, Tz);
  visual_slam::RobotRotationMatrix3D rm = getRotationMatrixFromRotationAngles(visual_slam::RobotRotationAngles3D(Rroll,Rpitch,Ryaw));
  return getTransformationMatrix(rm,t);
}
visual_slam::TFMatrix visual_slam::Robot_Pose::getInverseTransformation(const visual_slam::TFMatrix & transformation)
{
  visual_slam::RobotRotationMatrix3D new_rotation_mat = getRotationMatrixFromHomogeneousTransformationMatrix(transformation);
  visual_slam::RobotTranslation3D new_translation_vec = getTranslationVectorFromHomogeneousTransformationMatrix(transformation);
  visual_slam::RobotRotationMatrix3D Inverse_new_rotation_mat = getInverseRotation(new_rotation_mat);
  visual_slam::RobotTranslation3D Inverse_new_translation_vec = getInverseTranslation(new_translation_vec, Inverse_new_rotation_mat);
  return getTransformationMatrix(Inverse_new_rotation_mat,Inverse_new_translation_vec);
}
visual_slam::RobotRotationMatrix3D visual_slam::Robot_Pose::getInverseRotation(const visual_slam::RobotRotationMatrix3D & rotationMatrix)
{
  return rotationMatrix.transpose();
}
visual_slam::RobotTranslation3D visual_slam::Robot_Pose::getInverseTranslation(const visual_slam::RobotTranslation3D & translationVector, visual_slam::RobotRotationMatrix3D & inverseRotationMatrix)
{
  visual_slam::RobotTranslation3D InverseTranslation;
  inverseRotationMatrix *= -1.0;
  InverseTranslation = inverseRotationMatrix * translationVector;
  return InverseTranslation;
}


double visual_slam::Robot_Pose::convertFromDegreeToRadian(double degrees)
{
  return (degrees * (double) PI) /180.0;
}

double visual_slam::Robot_Pose::convertFromRadianToDegree(double radians)
{
  return ( radians * 180.0 ) / (double)PI ;
}

visual_slam::RobotRotationMatrix3D visual_slam::Robot_Pose::getRotationVector() const
{
  return _R;
}

visual_slam::RobotTranslation3D visual_slam::Robot_Pose::getTranslationVetor() const
{
  return _T;
}
