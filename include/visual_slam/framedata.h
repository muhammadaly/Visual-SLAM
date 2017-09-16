#ifndef FRAMEDATA_H
#define FRAMEDATA_H


#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "definitions.h"

#include <tf/transform_datatypes.h>
#include <sensor_msgs/CameraInfo.h>
#include <MatchingResult.h>
#include <myHeader.h>

namespace visual_slam {
class FrameData
{
public:
    FrameData() ;
    FrameData(cv::Mat pframeMatrix, std::string pfileName);
    FrameData(cv::Mat pframeMatrix,cv::Mat pdepthMatrix, std::string pfileName);
    FrameData(cv::Mat pframeMatrix, cv::Mat pdepthMatrix, PointCloudT::Ptr pPointCloud);
    FrameData(cv::Mat pframeMatrix, cv::Mat pdepthMatrix, sensor_msgs::CameraInfoConstPtr, std_msgs::Header);
    FrameData(cv::Mat pframeMatrix, cv::Mat pdepthMatrix, sensor_msgs::CameraInfoConstPtr pCameraInfoMsg,
                                  std_msgs::Header pDepthHeader, std::string pGTFrameName, std::string pBaseFrameName);

    cv::Mat getFrameMatrix() const;
    std::string getFileName() const;
    std::string getTimestamp() const;
    cv::Mat getDepthMatrix() const;

    std::vector<cv::KeyPoint> getKeypoints() const;
    void setKeypoints(const std::vector<cv::KeyPoint> &keypoints);
    cv::Mat getDescriptors() const;
    void setDescriptors(cv::Mat pSceneFeatureDescriptors);
    unsigned int getGraphNodeId() const;
    void setGraphNodeId(unsigned int);
    unsigned int getSequenceId() const;
    void setSequence_id(unsigned int sequenceId);
    visual_slam::TFMatrix getRobotPose()  const;
    void setRobotPose(visual_slam::TFMatrix);
    unsigned int getVertexId() const;
    void setVertexId(unsigned int vertexId);

    void setBase2PointsTransform(tf::StampedTransform);
    void setGroundTruthTransform(tf::StampedTransform);
    void setOdomTransform(tf::StampedTransform);
    void setSceneFeatureDescriptors(cv::Mat pSceneFeatureDescriptors);
    cv::Mat getSceneFeatureDescriptors() const;
    void setSequenceId(unsigned int sequenceId);
    tf::StampedTransform getGroundTruthTransform() const;
    MatchingResult matchNodePair(FrameData*);
    ros::Time getHeaderTime()const;
    bool isMatchable();

    int getNumberOfFeatures();
private:
    cv::Mat _frameMatrix;
    cv::Mat _depthMatrix;
    std::string _fileName;
    std::string _timestamp;
    PointCloudT::Ptr _pointCloud;
    std::vector<cv::KeyPoint> _keypoints;
    cv::Mat _descriptors;
    unsigned int _id;
    unsigned int _sequenceId;
    unsigned int _vertexId;
    TFMatrix RobotPose;
    tf::StampedTransform ground_truth_transform_;
    cv::Mat _sceneFeatureDescriptors;
    unsigned int _sequence_id;
    myHeader _header;
    bool matchable_;
};
}
#endif // FRAMEDATA_H
