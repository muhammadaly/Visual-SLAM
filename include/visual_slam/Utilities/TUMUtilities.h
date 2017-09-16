#ifndef TUMUTILITIES_H
#define TUMUTILITIES_H

#include <string>
#include <iostream>
#include <fstream>

#include "EigenUtilites.h"

namespace visual_slam {

class TUMUtilities{

public :
  TUMUtilities();
  TUMUtilities(std::string,std::string);
  bool writingResults(std::vector<visual_slam::Pose_6D>);

private:
  std::string gt_filename , result_filename;

  std::vector<std::string> getTimeStamps();
};

}

#endif // TUMUTILITIES_H
