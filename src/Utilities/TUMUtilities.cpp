#include "Utilities/TUMUtilities.h"

visual_slam::TUMUtilities::TUMUtilities(std::string ground_truth_filename,std::string presult_filename)
{
  gt_filename = ground_truth_filename;
  result_filename = presult_filename;
}

visual_slam::TUMUtilities::TUMUtilities()
{

}
bool visual_slam::TUMUtilities::writingResults(std::vector<visual_slam::Pose_6D> poses)
{
  std::ofstream result_file ;
  result_file.open(result_filename.c_str());
  std::vector<std::string> timestamps = getTimeStamps();
  for(int ind = 0 ; ind < poses.size();ind++)
  {
    DTFMatrix tmp = poses[ind].matrix();
    FQuarterionRotation rotation = EigenUtilites::instance()->ExtractRotationMatrixAsQuaternion(tmp);
    FTranslatonVec translation = EigenUtilites::instance()->ExtractTranslationVector(tmp);

    std::string line = timestamps[ind];
    line += " ";
    line += std::to_string(translation(0));
    line += " ";
    line += std::to_string(translation(1));
    line += " ";
    line += std::to_string(translation(2));
    line += " ";
    line += std::to_string(rotation.x());
    line += " ";
    line += std::to_string(rotation.y());
    line += " ";
    line += std::to_string(rotation.z());
    line += " ";
    line += std::to_string(rotation.w());
    line += "\n";

    result_file << line ;
  }
  result_file.close();
}

std::vector<std::string> visual_slam::TUMUtilities::getTimeStamps()
{
  std::ifstream gt_file(gt_filename.c_str());
  std::vector<std::string> timestamps;
  std::string line;
  if (gt_file.is_open())
  {
    std::string delimiter = " ";
    size_t pos = 0;
    while ( std::getline(gt_file,line) )
    {
      pos = line.find(delimiter);
      std::string token = line.substr(0, pos);
      timestamps.push_back(token);
    }
    return timestamps;
  }
}

