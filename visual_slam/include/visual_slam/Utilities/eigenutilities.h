#ifndef EIGENUTILITIES_H
#define EIGENUTILITIES_H
#include "visual_slam/definitions.h"

class EigenUtilites{
public :
  EigenUtilites();
  Quaternion ExtractRotationMatrixAsQuaternion(DTFMatrix);
  Quaternion ExtractTranslationVector(DTFMatrix);

};
#endif // EIGENUTILITIES_H

