#ifndef EIGENUTILITIES_H
#define EIGENUTILITIES_H
#include "visual_slam/definitions.h"
#include <Eigen/Geometry>

class EigenUtilites{
public :
  EigenUtilites();
  FQuarterionRotation ExtractRotationMatrixAsQuaternion(DTFMatrix);
  FTranslatonVec ExtractTranslationVector(DTFMatrix);

};
#endif // EIGENUTILITIES_H



