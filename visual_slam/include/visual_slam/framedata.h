#ifndef FRAMEDATA_H
#define FRAMEDATA_H


#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "definitions.h"

#include <tf/transform_datatypes.h>
#include <sensor_msgs/CameraInfo.h>
namespace visual_slam {
class FrameData
{
public:
    FrameData() ;
    FrameData(cv::Mat pframeMatrix, std::string pfileName);
    FrameData(cv::Mat pframeMatrix,cv::Mat pdepthMatrix, std::string pfileName);
    FrameData(cv::Mat pframeMatrix, cv::Mat pdepthMatrix, PointCloudT::Ptr pPointCloud);
    FrameData(cv::Mat pframeMatrix, cv::Mat pdepthMatrix, sensor_msgs::CameraInfoConstPtr, std_msgs::Header);

    cv::Mat getFrameMatrix() const;
    std::string getFileName() const;
    std::string getTimestamp() const;
    cv::Mat getDepthMatrix() const;

    cv::Mat getSceneFeatureDescriptors() const;
    void setSceneFeatureDescriptors(cv::Mat pSceneFeatureDescriptors);

    int getGraphNodeId() const;
    void setGraphNodeId(int);
    void setBase2PointsTransform(tf::StampedTransform);
    void setGroundTruthTransform(tf::StampedTransform);
    void setOdomTransform(tf::StampedTransform);

    visual_slam::TFMatrix getRobotPose()  const;
    void setRobotPose(visual_slam::TFMatrix);

private:
    cv::Mat _frameMatrix;
    cv::Mat _depthMatrix;
    std::string _fileName;
    std::string _timestamp;
    PointCloudT::Ptr _pointCloud;
    cv::Mat SceneFeatureDescriptors;
    int GraphNodeId;
    visual_slam::TFMatrix RobotPose;
};
}
#endif // FRAMEDATA_H
