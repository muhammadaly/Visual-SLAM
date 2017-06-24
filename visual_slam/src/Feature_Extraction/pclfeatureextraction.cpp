#include "Feature_Extraction/pclfeatureextraction.h"

visual_slam::PCLFeatureExtraction::PCLFeatureExtraction()
{

}

visual_slam::PointCloudT::Ptr visual_slam::PCLFeatureExtraction::fromCVKeypoint(std::vector<cv::KeyPoint> pKeypoints, visual_slam::FrameData pFrameData)
{
  cv::Mat Image = pFrameData.getFrameMatrix();
  cv::Mat depthImage = pFrameData.getDepthMatrix();

  visual_slam::PointCloudT::Ptr KeypointPC(new visual_slam::PointCloudT);

  float Z, factor = 5000;

  float fx = 525.0;  // focal length x
  float fy = 525.0;  // focal length y
  float cx = 319.5;  // optical center x
  float cy = 239.5;  // optical center y
  int rowInd , colInd;
  uint32_t r,g,b;
  for(int i = 0 ; i < pKeypoints.size() ; i++)
  {
    rowInd = pKeypoints[i].pt.y;
    colInd = pKeypoints[i].pt.x;

    cv::Vec3i intensity = Image.at<cv::Vec3i>(rowInd, colInd);
    b = (uint32_t)intensity.val[0];
    g = (uint32_t)intensity.val[1];
    r = (uint32_t)intensity.val[2];

    uint32_t rgb = (r << 16 | g << 8 | b);

    Z = depthImage.at<u_int16_t>(rowInd , colInd) / factor;

    visual_slam::PointT p;
    p.x = ((colInd - cx) * Z )/fx;
    p.y = ((rowInd - cy) * Z )/fy;
    p.z = Z;
    p.rgb = *reinterpret_cast<float*>(&rgb);
    KeypointPC->points.push_back(p);
  }
}
