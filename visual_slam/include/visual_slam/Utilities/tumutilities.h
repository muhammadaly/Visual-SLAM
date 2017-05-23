#ifndef TUMUTILITIES_H
#define TUMUTILITIES_H

#include <string>
#include <iostream>
#include <fstream>

class TUMUtilities{
public :
  TUMUtilities(std::string ground_truth_filename);
  bool writingResults();

private:
  ifstream gt_file;
  std::ofstream result_file;
};

#endif // TUMUTILITIES_H
