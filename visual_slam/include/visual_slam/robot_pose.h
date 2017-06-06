#ifndef ROBOT_POSE_H
#define ROBOT_POSE_H

#include "g2o/stuff/misc.h"
#include "g2o/stuff/macros.h"

#include "g2o_tutorial_slam2d_api.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "definations.h"

namespace visual_slam {

class Robot_Pose {
public:
  Robot_Pose();
  Robot_Pose(double Tx, double Ty, double Tz,
      double Rw, double Rroll, double Rpitch);

  Robot_Pose operator * (const visual_slam::TFMatrix&) const;

private:
  visual_slam::RobotTranslation3D _T;
  visual_slam::RobotRotationAngles3D _R;

  visual_slam::RobotRotationAngles3D getRotationMatrixFromHomogeneousTransformationMatrix(const visual_slam::TFMatrix&);
  visual_slam::RobotTranslation3D getTranslationVectorFromHomogeneousTransformationMatrix(const visual_slam::TFMatrix&);

};

}

#endif // ROBOT_POSE_H
