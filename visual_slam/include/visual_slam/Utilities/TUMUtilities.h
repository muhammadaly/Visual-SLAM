#ifndef TUMUTILITIES_H
#define TUMUTILITIES_H

#include <string>
#include <iostream>
#include <fstream>

#include "EigenUtilites.h"

class TUMUtilities{

public :
  TUMUtilities();
  TUMUtilities(std::string,std::string);
  bool writingResults(std::vector<Pose_6D>);

private:
  std::string gt_filename , result_filename;
  std::unique_ptr<EigenUtilites> eigen_utilities;

  std::vector<std::string> getTimeStamps();
};

#endif // TUMUTILITIES_H
