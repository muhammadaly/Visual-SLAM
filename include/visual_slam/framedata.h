#ifndef FRAMEDATA_H
#define FRAMEDATA_H


#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class FrameData
{
public:
    FrameData() ;
    FrameData(cv::Mat pframeMatrix, std::string pfileName);
    FrameData(cv::Mat pframeMatrix, std::string pfileName, cv::Mat pdepthMatrix);
    cv::Mat getFrameMatrix() const;
    std::string getFileName() const;
    std::string getTimestamp() const;
    cv::Mat getDepthMatrix() const;

private:
    cv::Mat _frameMatrix;
    cv::Mat _depthMatrix;
    std::string _fileName;
    std::string _timestamp;
};

#endif // FRAMEDATA_H
