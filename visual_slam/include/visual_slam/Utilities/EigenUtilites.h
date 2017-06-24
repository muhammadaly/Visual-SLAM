#ifndef EIGENUTILITIES_H
#define EIGENUTILITIES_H
#include "definitions.h"
#include <Eigen/Geometry>

namespace visual_slam {

class EigenUtilites{
public :
  EigenUtilites();
  FQuarterionRotation ExtractRotationMatrixAsQuaternion(DTFMatrix);
  FTranslatonVec ExtractTranslationVector(DTFMatrix);

};

}
#endif // EIGENUTILITIES_H
