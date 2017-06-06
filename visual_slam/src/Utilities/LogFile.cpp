#include "Utilities/LogFile.h"

LogFile::LogFile(std::string datasetFileName)
{
  myfile.open (datasetFileName + loggingFilename);
}

LogFile::~LogFile()
{
  myfile.close();
}

void LogFile::Log(std::string message)
{
  myfile << (message + "\n");
}
