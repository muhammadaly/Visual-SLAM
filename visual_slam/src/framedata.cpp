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

visual_slam::FrameData::FrameData(cv::Mat pframeMatrix,cv::Mat pdepthMatrix, std::string pfileName)
{
    _fileName = pfileName;
    _frameMatrix = pframeMatrix;
    int found = pfileName.find_last_of(".png")-3;
    int start = pfileName.find_last_of("/")+1;
    _timestamp = pfileName.substr(start , found-start);
    _depthMatrix = pdepthMatrix;
}

visual_slam::FrameData::FrameData(cv::Mat pframeMatrix, cv::Mat pdepthMatrix, PointCloudT::Ptr pPointCloud)
{
  _frameMatrix = pframeMatrix;
  _depthMatrix = pdepthMatrix;
  _pointCloud = pPointCloud;
}

visual_slam::FrameData::FrameData(cv::Mat pframeMatrix, cv::Mat pdepthMatrix, sensor_msgs::CameraInfoConstPtr pCameraInfoMsg,
                                  std_msgs::Header pDepthHeader, std::string pGTFrameName, std::string pBaseFrameName)
{
  _frameMatrix = pframeMatrix;
  _depthMatrix = pdepthMatrix;
  ground_truth_transform_(tf::Transform::getIdentity(), pDepthHeader.stamp,pGTFrameName,pBaseFrameName);
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
  _sceneFeatureDescriptors = pSceneFeatureDescriptors;
}

cv::Mat visual_slam::FrameData::getSceneFeatureDescriptors() const
{
  return _sceneFeatureDescriptors;
}

unsigned int visual_slam::FrameData::getId() const
{
  return _id;
}

void visual_slam::FrameData::setId(unsigned int pId)
{
  _id = pId;
}

visual_slam::TFMatrix visual_slam::FrameData::getRobotPose() const
{
  return RobotPose;
}

void visual_slam::FrameData::setRobotPose(visual_slam::TFMatrix pRobotPose)
{
  RobotPose = pRobotPose;
}

std::vector<cv::KeyPoint> FrameData::getKeypoints() const
{
  return _keypoints;
}

void FrameData::setKeypoints(const std::vector<cv::KeyPoint> &keypoints)
{
  _keypoints = keypoints;
}

void visual_slam::FrameData::setBase2PointsTransform(tf::StampedTransform pTransform)
{

}

void visual_slam::FrameData::setGroundTruthTransform(tf::StampedTransform)
{

}

void visual_slam::FrameData::setOdomTransform(tf::StampedTransform)
{

}

unsigned int FrameData::getSequenceId() const
{
  return _sequence_id;
}

void FrameData::setSequenceId(unsigned int sequenceId)
{
  _sequenceId = sequenceId;
}

unsigned int FrameData::getVertexId() const
{
  return _vertexId;
}

void FrameData::setVertexId(unsigned int vertexId)
{
  _vertexId = vertexId;
}

void FrameData::setGroundTruthTransform(tf::StampedTransform gt){
    ground_truth_transform_ = gt;
}
tf::StampedTransform FrameData::getGroundTruthTransform() const {
    return ground_truth_transform_;
}
