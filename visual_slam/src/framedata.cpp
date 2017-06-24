#include <framedata.h>

visual_slam::FrameData::FrameData()
{
}

visual_slam::FrameData::FrameData(cv::Mat pframeMatrix, std::string pfileName)
{
    _fileName = pfileName;
    _frameMatrix = pframeMatrix;
    int found = pfileName.find_last_of(".png")-3;
    int start = pfileName.find_last_of("/")+1;
    _timestamp = pfileName.substr(start , found-start);
}

visual_slam::FrameData::FrameData(cv::Mat pframeMatrix, std::string pfileName, cv::Mat pdepthMatrix)
{
    _fileName = pfileName;
    _frameMatrix = pframeMatrix;
    int found = pfileName.find_last_of(".png")-3;
    int start = pfileName.find_last_of("/")+1;
    _timestamp = pfileName.substr(start , found-start);
    _depthMatrix = pdepthMatrix;
}

cv::Mat visual_slam::FrameData::getFrameMatrix() const
{
    return _frameMatrix;
}

std::string visual_slam::FrameData::getFileName() const
{
    return _fileName;
}

std::string visual_slam::FrameData::getTimestamp() const
{
    return _timestamp;
}

cv::Mat visual_slam::FrameData::getDepthMatrix() const
{
    return _depthMatrix;
}

void visual_slam::FrameData::setSceneFeatureDescriptors(cv::Mat pSceneFeatureDescriptors)
{
  SceneFeatureDescriptors = pSceneFeatureDescriptors;
}

cv::Mat visual_slam::FrameData::getSceneFeatureDescriptors() const
{
  return SceneFeatureDescriptors;
}

int visual_slam::FrameData::getGraphNodeId() const
{
  return GraphNodeId;
}

void visual_slam::FrameData::setGraphNodeId(int pGraphNodeId)
{
  GraphNodeId = pGraphNodeId;
}

visual_slam::TFMatrix visual_slam::FrameData::getRobotPose() const
{
  return RobotPose;
}

void visual_slam::FrameData::setRobotPose(visual_slam::TFMatrix pRobotPose)
{
  RobotPose = pRobotPose;
}
