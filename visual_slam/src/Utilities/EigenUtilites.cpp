#include "Utilities/EigenUtilites.h"

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
visual_slam::EigenUtilites::EigenUtilites()
{

}
