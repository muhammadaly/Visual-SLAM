Quaternion EigenUtilites::ExtractRotationMatrixAsQuaternion(DTFMatrix transformationMatrix)
{
  DRotationMatrix rotationMatrix;

  rotationMatrix(0,0) = transformationMatrix(0,0);
  rotationMatrix(0,1) = transformationMatrix(0,1);
  rotationMatrix(0,2) = transformationMatrix(0,2);
  rotationMatrix      =
  rotationMatrix(1,0) = transformationMatrix(1,0);
  rotationMatrix(1,1) = transformationMatrix(1,1);
  rotationMatrix(1,2) = transformationMatrix(1,2);
  rotationMatrix      =
  rotationMatrix(2,0) = transformationMatrix(2,0);
  rotationMatrix(2,1) = transformationMatrix(2,1);
  rotationMatrix(2,2) = transformationMatrix(2,2);

  return Quaternionf(mat);
}
