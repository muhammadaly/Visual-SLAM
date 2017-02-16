#include <memory>
#include <boost/scoped_ptr.hpp>

#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <visual_slam/Transformation_Estimation/TransformationEstimator.h>
#include <visual_slam/Transformation_Estimation/PCL3DRANSACTransformationEstimator.h>
#include <visual_slam/framedata.h>
#include <visual_slam/Feature_Extraction/CVORBFeatureExtractorAndDescriptor.h>
#include <visual_slam/Feature_Matching/cvflannfeaturematcher.h>
#include <visual_slam/Utilities/PCLUtilities.h>

std::string machineName = "ivsystems";
std::string datasetName = "rgbd_dataset_freiburg1_360";
std::string datasetDIR = "/home/"+machineName+"/master_dataset/"+datasetName+"/";
std::string rgbImages = datasetDIR + "rgb/";
std::string depthImages = datasetDIR + "depth/";
std::string transformationMatrices = datasetDIR + "transformationMatricesPCL.txt";
std::string featureMatching = datasetDIR + "matching/";
std::string frames_matching = datasetDIR + "matching_frames.txt";

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

void visualizePointCloud(PointCloudT::ConstPtr secondPC , PointCloudT::ConstPtr firstPC, PointCloudT::ConstPtr aligned)
{
  pcl::visualization::PCLVisualizer visu("Point Cloud Visualizer");
  visu.addPointCloud(secondPC,ColorHandlerT(secondPC, 0.0, 0.0, 255.0), "secondPC");
  visu.addPointCloud(firstPC,ColorHandlerT(firstPC, 0.0, 255.0, 0.0), "firstPC");
  visu.addPointCloud(aligned,ColorHandlerT(aligned, 255.0, 255.0, 255.0), "cloud");
  visu.spin();
}
void visualizePointCloud(PointCloudT::ConstPtr secondPC , PointCloudT::ConstPtr firstPC)
{
  pcl::visualization::PCLVisualizer visu("Point Cloud Visualizer");
  visu.addPointCloud(secondPC,ColorHandlerT(secondPC, 255.0, 255.0, 255.0), "secondPC");
  visu.addPointCloud(firstPC,ColorHandlerT(firstPC, 0.0, 255.0, 0.0), "firstPC");
  visu.spin();
}
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

void showMatchingImage(std::vector<cv::DMatch> good_matches
                       , std::vector<cv::KeyPoint> currentImageKeypoints , std::vector<cv::KeyPoint> previousImageKeypoints
                       , FrameData currentFrame, FrameData previousFrame)
{

  cv::Mat currentImage = currentFrame.getFrameMatrix();
  cv::Mat previousImage = previousFrame.getFrameMatrix();

  cv::namedWindow("matches", 1);
  cv::Mat img_matches;
  cv::drawMatches(currentImage, currentImageKeypoints, previousImage, previousImageKeypoints, good_matches, img_matches);
  cv::imshow("matches", img_matches);
  cv::waitKey(0);
}

class Transformation_EstimatorNodeHandler {
public:
  Transformation_EstimatorNodeHandler();
  bool estimateTransformBetween2Scenes(FrameData previousFrame , FrameData currentFrame, Pose_6D& transformation);

  void publishOnTF(TFMatrix);
private:
  ros::NodeHandle _node;

  std::unique_ptr<TransformationEstimator> tfEstimator;
  std::unique_ptr<FeatureExtractorAndDescriptor> featureExtractorAndDescriptor;
  std::unique_ptr<FeatureMatcher> featureMatcher;
  PCLUtilities pclUTL;

  ros::Publisher odom_pub ;
  tf::TransformBroadcaster br;
};


Transformation_EstimatorNodeHandler::Transformation_EstimatorNodeHandler()
{
  tfEstimator = std::unique_ptr<PCL3DRANSACTransformationEstimator>(new PCL3DRANSACTransformationEstimator);
  featureExtractorAndDescriptor = std::unique_ptr<CVORBFeatureExtractorAndDescriptor>(new CVORBFeatureExtractorAndDescriptor);
  featureMatcher = std::unique_ptr<CVFLANNFeatureMatcher>(new CVFLANNFeatureMatcher);
  odom_pub = _node.advertise<nav_msgs::Odometry>("odom", 50);
}
bool Transformation_EstimatorNodeHandler::estimateTransformBetween2Scenes(FrameData previousFrame, FrameData currentFrame, Pose_6D& transformation)
{
  std::vector<cv::KeyPoint> tPreviousKeypoints , tCurrentKeypoints ;
  bool done = false;
  std::vector<cv::DMatch> matches;
  cv::Mat tPreviousDescriptors,tCurrentDescriptors;
  PointCloudT::Ptr tCurrentKeypointsPC (new PointCloudT),tPreviousKeypointsPC (new PointCloudT) , alignedPC(new PointCloudT);
  FeatureCloudT::Ptr tCurrentDescriptorsPC (new FeatureCloudT),tPreviousDescriptorsPC (new FeatureCloudT);
  TFMatrix trans;

  featureExtractorAndDescriptor->computeDescriptors(previousFrame , tPreviousKeypoints , tPreviousDescriptors);
  featureExtractorAndDescriptor->computeDescriptors(currentFrame , tCurrentKeypoints , tCurrentDescriptors);
  featureMatcher->matching2ImageFeatures(tPreviousDescriptors , tCurrentDescriptors,matches);
  pclUTL.getKeypointsAndDescriptors(matches,tPreviousKeypoints,tCurrentKeypoints,
                                    tPreviousDescriptors,tCurrentDescriptors,previousFrame,currentFrame,
                                    tPreviousKeypointsPC,tCurrentKeypointsPC,
                                    tPreviousDescriptorsPC,tCurrentDescriptorsPC);
  done = tfEstimator->estimateTransformation(tPreviousKeypointsPC,tPreviousDescriptorsPC,tCurrentKeypointsPC,tCurrentDescriptorsPC,trans,alignedPC);

  //  PointCloudT::Ptr currentScene = GeneratePointCloud(currentFrame.getDepthMatrix());
  //  PointCloudT::Ptr previousScene = GeneratePointCloud(currentFrame.getDepthMatrix());
  //  visualizePointCloud(tPreviousKeypointsPC , tCurrentKeypointsPC,alignedPC);
  //  visualizePointCloud(currentScene , tCurrentKeypointsPC);

  transformation.matrix() = trans;
  return done;
}

void Transformation_EstimatorNodeHandler::publishOnTF(TFMatrix transformation)
{
  double x = static_cast<double>(transformation(0,3));
  double y = static_cast<double>(transformation(1,3));
  double z = static_cast<double>(transformation(2,3));

  tf::Vector3 origin;
  origin.setValue(x,y,z);

  tf::Matrix3x3 tf3d;
  tf3d.setValue(static_cast<double>(transformation(0,0)), static_cast<double>(transformation(0,1)), static_cast<double>(transformation(0,2)),
                static_cast<double>(transformation(1,0)), static_cast<double>(transformation(1,1)), static_cast<double>(transformation(1,2)),
                static_cast<double>(transformation(2,0)), static_cast<double>(transformation(2,1)), static_cast<double>(transformation(2,2)));
  tf::Quaternion tfqt;
  tf3d.getRotation(tfqt);
  tfqt = tfqt.normalize();

  tf::Transform transform;
  transform.setOrigin(origin);
  transform.setRotation(tfqt);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

  nav_msgs::Odometry odom;
  geometry_msgs::Quaternion odom_quat;
  tf::quaternionTFToMsg(tfqt,odom_quat);

  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = z;
  odom.pose.pose.orientation = odom_quat;

  //publish the message
  odom_pub.publish(odom);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Transformation_Estimation_Node");

  std::unique_ptr<Transformation_EstimatorNodeHandler> nh(new Transformation_EstimatorNodeHandler);

  std::vector<FrameData> Frames = readDataset();
  Pose_6D robot_pose;
  robot_pose.matrix() = Eigen::Matrix4f::Zero();
  bool done = false;
  int scenes_start = 0 , scenes_end = 30;
  int first_scn = scenes_start , second_scn = first_scn+1;
  for(int i = scenes_start ; i <= scenes_end ; i ++)
  {
    ROS_INFO("%i" , i);
    FrameData previousFrame = Frames[first_scn];
    FrameData currentFrame = Frames[second_scn];
    Pose_6D tf;
    tf.matrix() = Eigen::Matrix4f::Zero();

    done = nh->estimateTransformBetween2Scenes(previousFrame,currentFrame,tf);
    first_scn++;
    second_scn++;
    if(done)
    {
      robot_pose = tf * robot_pose;
      nh->publishOnTF(robot_pose.matrix());
    }
  }
  return 0;
}
