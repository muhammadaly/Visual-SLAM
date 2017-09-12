#ifndef ROBOT_POSE_H
#define ROBOT_POSE_H

#include "g2o/stuff/misc.h"
#include "g2o/stuff/macros.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "definitions.h"

namespace visual_slam {

class Robot_Pose {
public:
  Robot_Pose();
  Robot_Pose(double, double, double, double, double, double);

  Robot_Pose& operator *= (const visual_slam::TFMatrix&);
  void transform(const visual_slam::TFMatrix&);

  void setRotationMatrix(visual_slam::RobotRotationMatrix3D);
  void setTranslationVector(visual_slam::RobotTranslation3D);
  visual_slam::RobotPose6D getRobotPose();
  visual_slam::TFMatrix getTransformationMatrix();
  visual_slam::TFMatrix getTransformationMatrix(const visual_slam::RobotRotationMatrix3D&, const visual_slam::RobotTranslation3D&);
  visual_slam::TFMatrix getTransformationMatrix(double, double, double, double, double, double);

  double convertFromRadianToDegree(double);
  double convertFromDegreeToRadian(double);

  visual_slam::RobotTranslation3D getTranslationVetor() const;
  visual_slam::RobotRotationMatrix3D getRotationVector() const;
  visual_slam::TFMatrix getInverseTransformation(const visual_slam::TFMatrix&);

private:
  visual_slam::RobotTranslation3D _T;
  visual_slam::RobotRotationMatrix3D _R;

  visual_slam::RobotRotationMatrix3D getRotationMatrixFromHomogeneousTransformationMatrix(const visual_slam::TFMatrix&);
  visual_slam::RobotTranslation3D getTranslationVectorFromHomogeneousTransformationMatrix(const visual_slam::TFMatrix&);
  visual_slam::RobotRotationMatrix3D getInverseRotation(const visual_slam::RobotRotationMatrix3D&);
  visual_slam::RobotTranslation3D getInverseTranslation(const visual_slam::RobotTranslation3D&, visual_slam::RobotRotationMatrix3D&);

  visual_slam::RobotRotationAngles3D getRotationAnglesFromRotationMatrix(const visual_slam::RobotRotationMatrix3D&);
  visual_slam::RobotRotationMatrix3D getRotationMatrixFromRotationAngles(const visual_slam::RobotRotationAngles3D&);


};

}

#endif // ROBOT_POSE_H
