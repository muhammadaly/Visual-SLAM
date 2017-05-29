#ifndef LOGFILE_H
#define LOGFILE_H

#include <string>
#include <iostream>
#include <fstream>

class LogFile{
public:
  LogFile(std::string datasetFileName);
  ~LogFile();
  void Log(std::string);

private:
  std::ofstream myfile;
  const std::string loggingFilename = "log.txt";
};
#endif // LOGFILE_H
