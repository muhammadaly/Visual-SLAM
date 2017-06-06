#include <framedata.h>

FrameData::FrameData()
{
}

FrameData::FrameData(cv::Mat pframeMatrix, std::string pfileName)
{
    _fileName = pfileName;
    _frameMatrix = pframeMatrix;
    int found = pfileName.find_last_of(".png")-3;
    int start = pfileName.find_last_of("/")+1;
    _timestamp = pfileName.substr(start , found-start);
}

FrameData::FrameData(cv::Mat pframeMatrix, std::string pfileName, cv::Mat pdepthMatrix)
{
    _fileName = pfileName;
    _frameMatrix = pframeMatrix;
    int found = pfileName.find_last_of(".png")-3;
    int start = pfileName.find_last_of("/")+1;
    _timestamp = pfileName.substr(start , found-start);
    _depthMatrix = pdepthMatrix;
}

cv::Mat FrameData::getFrameMatrix() const
{
    return _frameMatrix;
}

std::string FrameData::getFileName() const
{
    return _fileName;
}

std::string FrameData::getTimestamp() const
{
    return _timestamp;
}

cv::Mat FrameData::getDepthMatrix() const
{
    return _depthMatrix;
}

void FrameData::setSceneFeatureDescriptors(cv::Mat pSceneFeatureDescriptors)
{
  SceneFeatureDescriptors = pSceneFeatureDescriptors;
}

cv::Mat FrameData::getSceneFeatureDescriptors() const
{
  return SceneFeatureDescriptors;
}

int FrameData::getGraphNodeId() const
{
  return GraphNodeId;
}

void FrameData::setGraphNodeId(int pGraphNodeId)
{
  GraphNodeId = pGraphNodeId;
}

visual_slam::TFMatrix FrameData::getRobotPose() const
{
  return RobotPose;
}

void FrameData::setRobotPose(visual_slam::TFMatrix pRobotPose)
{
  RobotPose = pRobotPose;
}
