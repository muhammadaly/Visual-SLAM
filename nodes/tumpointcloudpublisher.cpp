#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <visual_slam/framedata.h>
#include <glob.h>

#include <visual_slam/framedata.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/core.hpp>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

float fx = 525.0;  // focal length x
float fy = 525.0;  // focal length y
float cx = 319.5;  // optical center x
float cy = 239.5;  // optical center y

std::string datasetDIR = "/home/muhammadaly/master_dataset/rgbd_dataset_freiburg1_xyz/";
std::string rgbImages = datasetDIR + "rgb/";
std::string depthImages = datasetDIR + "depth/";
std::string transformationMatrices = datasetDIR + "transformationMatrices.txt";
std::string featureMatching = datasetDIR + "matching/";
std::string frames_matching = datasetDIR + "matching_frames.txt";

std::vector<FrameData> readDataset()
{
  std::vector<FrameData> frames ;
  std::string line, tmp;
  char * pch;
  std::vector<std::string> depthFiles , rgbFiles ;
  std::ifstream myfile (frames_matching.c_str());
  if (myfile.is_open())
  {
    while ( std::getline(myfile,line) )
    {
      printf(line.c_str());
      pch = std::strtok(const_cast<char*>(line.c_str())," /");
      int i = 0;
      while (pch != NULL)
      {
        if (i == 2)
        {
          tmp = std::string(pch);
          rgbFiles.push_back(tmp);
        }
        else if (i == 5)
        {
          tmp = std::string(pch);
          depthFiles.push_back(tmp);
        }
        pch = strtok (NULL, " /");
        i++;
      }
    }
    for(int i = 0 ; i < rgbFiles.size() ; i++)
    {
      cv::Mat image = cv::imread((rgbImages+rgbFiles[i]).c_str());
      cv::Mat Dimage = cv::imread((depthImages+depthFiles[i]).c_str(),CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
      Dimage.convertTo(Dimage , CV_16U);
      if(image.data && Dimage.data)
        frames.push_back(FrameData(image , (depthImages+depthFiles[i]).c_str() ,Dimage));
    }
    myfile.close();

  }
  return frames;
}

PointCloudT::Ptr GeneratePointCloud(cv::Mat pImage)
{
  PointCloudT::Ptr cloud (new PointCloudT);

  float Z , factor = 5000;
  for(int rowInd = 0 ; rowInd < pImage.rows ; rowInd++)
    for(int colInd = 0 ; colInd < pImage.cols ; colInd++)
    {
      Z = pImage.at<u_int16_t>(rowInd , colInd) / factor;
      PointT p;
      p.x = ((colInd - cx) * Z )/fx;
      p.y = ((rowInd - cy) * Z )/fy;
      p.z = Z;

      cloud->points.push_back(p);
    }
  return cloud;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tumpointcloudpublisher");
  ros::NodeHandle n;
  std::vector<FrameData> Frames = readDataset();
  printf("Reading Finsihed !");
  ros::Publisher tum_dataset_pub = n.advertise<sensor_msgs::PointCloud2>("/cloud", 1000);

  ros::Rate loop_rate(10);
  int ind = 0;
  while (ros::ok())
  {
    PointCloudT::Ptr cloud = GeneratePointCloud(Frames[ind].getFrameMatrix());
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud , msg);
    tum_dataset_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
    ++ind;
  }
}
