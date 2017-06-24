#include "Utilities/PCLUtilities.h"

visual_slam::PCLUtilities::PCLUtilities()
{

}

void visual_slam::PCLUtilities::getKeypointsAndDescriptors(std::vector<cv::DMatch> matches,
                                              std::vector<cv::KeyPoint> previousKeypoints, std::vector<cv::KeyPoint> currentKeypoints,
                                              cv::Mat previousDescriptors , cv::Mat currentDescriptors,
                                              FrameData previousFrameData, FrameData currentFrameData,
                                              visual_slam::PointCloudT::Ptr previousKeypointsPointCloud, visual_slam::PointCloudT::Ptr currentKeypointsPointCloud,
                                              visual_slam::FeatureCloudT::Ptr previousFeaturesPointCloud, visual_slam::FeatureCloudT::Ptr currentFeaturesPointCloud)
{
  float Z , factor = 5000;
  cv::Mat currentDepthImage = currentFrameData.getDepthMatrix();
  cv::Mat previousDepthImage = previousFrameData.getDepthMatrix();

  cv::KeyPoint currentKeypoint , previousKeypoint;
  int rowInd , colInd;
  for(size_t matchInd = 1 ;matchInd < matches.size() ; matchInd++)
  {
      currentKeypoint = currentKeypoints[matches[matchInd].queryIdx];
      previousKeypoint = previousKeypoints[matches[matchInd].trainIdx];

      rowInd = previousKeypoint.pt.y;
      colInd = previousKeypoint.pt.x;
      Z = previousDepthImage.at<u_int16_t>(rowInd , colInd) / factor;

      visual_slam::PointT p;
      p.x = ((colInd - visual_slam::cx) * Z )/visual_slam::fx;
      p.y = ((previousKeypoint.pt.y - visual_slam::cy) * Z )/visual_slam::fy;
      p.z = Z;
      previousKeypointsPointCloud->points.push_back(p);

      visual_slam::FeatureT PCLDescriptor;
      for(int ind = 0 ; ind < previousDescriptors.cols; ind++)
      {
          PCLDescriptor.histogram[ind] = previousDescriptors.at<unsigned char>(matches[matchInd].queryIdx,ind);
      }
      previousFeaturesPointCloud->points.push_back(PCLDescriptor);

      rowInd = currentKeypoint.pt.y;
      colInd = currentKeypoint.pt.x;

      Z = currentDepthImage.at<u_int16_t>(rowInd , colInd) / factor;

      visual_slam::PointT p2;
      p2.x = ((colInd - visual_slam::cx) * Z )/visual_slam::fx;
      p2.y = ((rowInd - visual_slam::cy) * Z )/visual_slam::fy;
      p2.z = Z;
      currentKeypointsPointCloud->points.push_back(p2);

      visual_slam::FeatureT PCLDescriptor2;
      for(int ind = 0 ; ind < currentDescriptors.cols; ind++)
      {
          PCLDescriptor2.histogram[ind] = currentDescriptors.at<unsigned char>(matches[matchInd].queryIdx,ind);
      }
      currentFeaturesPointCloud->points.push_back(PCLDescriptor2);

  }
  //        visualizePointCloud(previousSelectedPointCloud , currentSelectedPointCloud);
}
