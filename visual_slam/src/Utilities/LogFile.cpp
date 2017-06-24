#include "Utilities/LogFile.h"

visual_slam::LogFile::LogFile(std::string datasetFileName)
{
  myfile.open (datasetFileName + loggingFilename);
}

visual_slam::LogFile::~LogFile()
{
  myfile.close();
}

void visual_slam::LogFile::Log(std::string message)
{
  myfile << (message + "\n");
}
