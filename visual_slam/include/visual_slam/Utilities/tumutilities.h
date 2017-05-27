#ifndef TUMUTILITIES_H
#define TUMUTILITIES_H

#include <string>
#include <iostream>
#include <fstream>

#include "eigenutilities.h"

class TUMUtilities{
public :
  TUMUtilities(std::string ground_truth_filename);
  bool writingResults(std::vector<Pose_6D>);
private:
  ifstream gt_file;
  std::ofstream result_file;
  std::unique_ptr<EigenUtilites> eigen_utilities;
};

#endif // TUMUTILITIES_H
