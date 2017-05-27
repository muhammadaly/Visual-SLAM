#include "visual_slam/Utilites/tumutilities.h"

TUMUtilities::TUMUtilities(std::string ground_truth_filename)
{
  eigen_utilities = std::unique_ptr<EigenUtilites> (new EigenUtilites);
}

bool TUMUtilities::writingResults(std::vector<Pose_6D> poses)
{
  for(int ind = 0 ; i < poses.size();i++)
  {
    DTFMatrix tmp = poses[ind].matrix();
    Quaternion q = eigen_utilities->ExtractRotationMatrixAsQuaternion(tmp);
  }
}
