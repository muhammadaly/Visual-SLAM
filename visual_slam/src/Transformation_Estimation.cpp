#include <memory>
#include <boost/scoped_ptr.hpp>

#include "ros/ros.h"
#include <eigen_conversions/eigen_msg.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <visual_slam/Transformation_Estimation/TransformationEstimator.h>
#include <visual_slam/Transformation_Estimation/PCL3DRANSACTransformationEstimator.h>
#include <visual_slam/framedata.h>
#include <visual_slam/Feature_Extraction/CVORBFeatureExtractorAndDescriptor.h>
#include <visual_slam/Feature_Matching/cvflannfeaturematcher.h>
#include <visual_slam/Utilities/PCLUtilities.h>

#include <visual_slam/definitions.h>
#include <visual_slam/Map_Optimization/g2omapoptimizer.h>

#include <visual_slam/definitions.h>
#include <visual_slam_msgs/scene.h>

std::string machineName = "ivsystems";
std::string datasetName = "rgbd_dataset_freiburg1_desk2";
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
      if(image.data && Dimage.data)
      {
//        Dimage.convertTo(Dimage , CV_16U);
        frames.push_back(FrameData(image , (depthImages+depthFiles[i]).c_str() ,Dimage));
      }

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
  bool estimateTransformBetween2Scenes(FrameData previousFrame , FrameData currentFrame, TFMatrix& transformation);
  int addToMap(Pose_6D newPose);
  void detectLoopClosure(cv::Mat currentFeature, Pose_6D Pose, int nodeId);

  void publishOnTF(TFMatrix);
  void publishOdometry(TFMatrix);
  void publishPose(TFMatrix);
  void publishFullPath(TFMatrix);
  cv::Mat currentSceneFeaturesDes;
private:
  ros::NodeHandle _node;

  std::unique_ptr<TransformationEstimator> tfEstimator;
  std::unique_ptr<FeatureExtractorAndDescriptor> featureExtractorAndDescriptor;
  std::unique_ptr<FeatureMatcher> featureMatcher;
  PCLUtilities pclUTL;

  ros::Publisher odom_pub ;
  ros::Publisher pose_pub;
  ros::Publisher path_pub;
  ros::Publisher scene_pub;
  tf::TransformBroadcaster br;
  std::vector<geometry_msgs::PoseStamped> fullPath;

  std::unique_ptr<G2OMapOptimizer> mapOptimizer;
  Pose_6D prevPose;
  int prevNodeId;
  short numberOfNode;
  std::vector<int> NodeIds;
  std::vector<std::pair<cv::Mat,int>> FeatureMap;
  void updateGraph();
  void publishOptomizedPath();
  int searchForSimilerScene(cv::Mat);
  Pose_6D convertToPose(TFMatrix);
};


Transformation_EstimatorNodeHandler::Transformation_EstimatorNodeHandler()
{
  tfEstimator = std::unique_ptr<PCL3DRANSACTransformationEstimator>(new PCL3DRANSACTransformationEstimator);
  featureExtractorAndDescriptor = std::unique_ptr<CVORBFeatureExtractorAndDescriptor>(new CVORBFeatureExtractorAndDescriptor);
  featureMatcher = std::unique_ptr<CVFLANNFeatureMatcher>(new CVFLANNFeatureMatcher);
  odom_pub = _node.advertise<nav_msgs::Odometry>("odom", 50);
  pose_pub = _node.advertise<geometry_msgs::PoseStamped>("robot_pose",50);
  path_pub = _node.advertise<nav_msgs::Path>("robot_path",50);
  scene_pub = _node.advertise<visual_slam_msgs::scene>("scene_data",50);
  fullPath.clear();

  mapOptimizer = std::unique_ptr<G2OMapOptimizer>(new G2OMapOptimizer);
  prevPose = Pose_6D::Identity();
  mapOptimizer->addPoseToGraph(prevPose, prevNodeId);
  numberOfNode = 0;
  NodeIds.push_back(prevNodeId);
}
bool Transformation_EstimatorNodeHandler::estimateTransformBetween2Scenes(FrameData previousFrame, FrameData currentFrame, TFMatrix& transformation)
{
  std::vector<cv::KeyPoint> tPreviousKeypoints , tCurrentKeypoints ;
  bool done = false;
  std::vector<cv::DMatch> matches;
  cv::Mat tPreviousDescriptors,tCurrentDescriptors;
  PointCloudT::Ptr tCurrentKeypointsPC (new PointCloudT),tPreviousKeypointsPC (new PointCloudT) , alignedPC(new PointCloudT);
  FeatureCloudT::Ptr tCurrentDescriptorsPC (new FeatureCloudT),tPreviousDescriptorsPC (new FeatureCloudT);

  featureExtractorAndDescriptor->computeDescriptors(previousFrame , tPreviousKeypoints , tPreviousDescriptors);
  featureExtractorAndDescriptor->computeDescriptors(currentFrame , tCurrentKeypoints , tCurrentDescriptors);
  currentSceneFeaturesDes = tCurrentDescriptors;
  featureMatcher->matching2ImageFeatures(tPreviousDescriptors , tCurrentDescriptors,matches);
  pclUTL.getKeypointsAndDescriptors(matches,tPreviousKeypoints,tCurrentKeypoints,
                                    tPreviousDescriptors,tCurrentDescriptors,previousFrame,currentFrame,
                                    tPreviousKeypointsPC,tCurrentKeypointsPC,
                                    tPreviousDescriptorsPC,tCurrentDescriptorsPC);
  done = tfEstimator->estimateTransformation(tPreviousKeypointsPC,tPreviousDescriptorsPC,tCurrentKeypointsPC,tCurrentDescriptorsPC,transformation,alignedPC);

  visual_slam_msgs::scene msg;
  geometry_msgs::Pose pose;
  Pose_6D p = convertToPose(transformation);
  tf::poseEigenToMsg(p,pose);
  msg.pose = pose;
  cv_bridge::CvImage out_msg;
//  out_msg.header.time = ros::Time::now(); // Same timestamp and tf frame as input image
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
  out_msg.image    = tCurrentDescriptors; // Your cv::Mat
  msg.features = *out_msg.toImageMsg();
  scene_pub.publish(msg);
  //  saliency_img_pub.publish();

  //  msg.features = tCurrentDescriptors;

  //  PointCloudT::Ptr currentScene = GeneratePointCloud(currentFrame.getDepthMatrix());
  //  PointCloudT::Ptr previousScene = GeneratePointCloud(currentFrame.getDepthMatrix());
  // visualizePointCloud(tPreviousKeypointsPC , tCurrentKeypointsPC,alignedPC);
  //  visualizePointCloud(currentScene , tCurrentKeypointsPC);
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
}
void Transformation_EstimatorNodeHandler::publishOdometry(TFMatrix robot_pose)
{
  double x = static_cast<double>(robot_pose(0,3));
  double y = static_cast<double>(robot_pose(1,3));
  double z = static_cast<double>(robot_pose(2,3));

  double qw = sqrt(1.0 + robot_pose(0,0) + robot_pose(1,1) + robot_pose(2,2)) / 2.0;
  double w4 = 4.0 * qw ;
  double qx = (robot_pose(2,1) - robot_pose(1,2)) / w4;
  double qy = (robot_pose(0,2) - robot_pose(2,0)) / w4;
  double qz = (robot_pose(1,0) - robot_pose(0,1)) / w4;

  nav_msgs::Odometry odom;


  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = z;
  odom.pose.pose.orientation.w = w4;
  odom.pose.pose.orientation.x = qx;
  odom.pose.pose.orientation.y = qy;
  odom.pose.pose.orientation.z = qz;

  //publish the message
  odom_pub.publish(odom);
}

void Transformation_EstimatorNodeHandler::publishPose(TFMatrix robot_pose)
{
  double x = static_cast<double>(robot_pose(0,3));
  double y = static_cast<double>(robot_pose(1,3));
  double z = static_cast<double>(robot_pose(2,3));

  double qw = sqrt(1.0 + robot_pose(0,0) + robot_pose(1,1) + robot_pose(2,2)) / 2.0;
  double w4 = 4.0 * qw ;
  double qx = (robot_pose(2,1) - robot_pose(1,2)) / w4;
  double qy = (robot_pose(0,2) - robot_pose(2,0)) / w4;
  double qz = (robot_pose(1,0) - robot_pose(0,1)) / w4;

  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "base_link";
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = z;
  msg.pose.orientation.w = w4;
  msg.pose.orientation.x = qx;
  msg.pose.orientation.y = qy;
  msg.pose.orientation.z = qz;

  pose_pub.publish(msg);

}

void Transformation_EstimatorNodeHandler::publishFullPath(TFMatrix robot_pose)
{
  double x = static_cast<double>(robot_pose(0,3));
  double y = static_cast<double>(robot_pose(1,3));
  double z = static_cast<double>(robot_pose(2,3));

  double qw = sqrt(1.0 + robot_pose(0,0) + robot_pose(1,1) + robot_pose(2,2)) / 2.0;
  double w4 = 4.0 * qw ;
  double qx = (robot_pose(2,1) - robot_pose(1,2)) / w4;
  double qy = (robot_pose(0,2) - robot_pose(2,0)) / w4;
  double qz = (robot_pose(1,0) - robot_pose(0,1)) / w4;

  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "base_link";
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = z;
  msg.pose.orientation.w = w4;
  msg.pose.orientation.x = qx;
  msg.pose.orientation.y = qy;
  msg.pose.orientation.z = qz;
  fullPath.push_back(msg);

  if(!fullPath.empty()){

    nav_msgs::Path pathMsg ;
    pathMsg.header.frame_id = "odom";
    pathMsg.header.stamp = ros::Time::now();
    pathMsg.poses.resize(fullPath.size());
    for(int i=0; i < fullPath.size(); i++){
      pathMsg.poses[i] = fullPath[i];
    }
    path_pub.publish(pathMsg);
  }
}

int Transformation_EstimatorNodeHandler::addToMap(Pose_6D newPose)
{
  int newNodeId;
  mapOptimizer->addPoseToGraph(newPose, newNodeId);
  NodeIds.push_back(newNodeId);
  mapOptimizer->addEdge(newPose ,newNodeId ,prevNodeId);
  prevPose = newPose;
  prevNodeId = newNodeId;
  if(numberOfNode ==10)
  {
    numberOfNode = 0;
    updateGraph();
  }
  return newNodeId;
}

void Transformation_EstimatorNodeHandler::detectLoopClosure(cv::Mat currentFeature, Pose_6D Pose, int nodeId)
{
  if(FeatureMap.size()==0)
  {
    std::pair<cv::Mat,int> tmp;
    tmp.first = currentFeature;
    tmp.second = nodeId;
  }
  else
  {
    int sceneNumber = searchForSimilerScene(currentFeature);
    if(sceneNumber > 0)
    {
      mapOptimizer->addEdge(Pose ,nodeId ,sceneNumber);
    }
    else
    {
      std::pair<cv::Mat,int> tmp;
      tmp.first = currentFeature;
      tmp.second = nodeId;
    }
  }
}

void Transformation_EstimatorNodeHandler::updateGraph()
{
  mapOptimizer->optimize();
}

int Transformation_EstimatorNodeHandler::searchForSimilerScene(cv::Mat pCurrentDescriptors)
{
  double threshold = 0.8;
  for(int i = 0; i < FeatureMap.size() ; i++)
  {
    std::vector<cv::DMatch> matches;
    featureMatcher->matching2ImageFeatures(FeatureMap[i].first , pCurrentDescriptors,matches);
    if((matches.size() / pCurrentDescriptors.rows) > threshold)
    {
      return FeatureMap[i].second;
    }
  }
  return -1;
}

Pose_6D Transformation_EstimatorNodeHandler::convertToPose(TFMatrix transformation)
{
  Pose_6D tmp;
  Eigen::Matrix<double, 4, 4> tmpMat ;
  for(int i = 0 ; i < 4  ; i ++)
    for(int j = 0 ; j < 4  ; j ++)
      tmpMat(i,j) = (double)transformation(i,j);
  tmp.matrix() = tmpMat;
  return tmp;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "Transformation_Estimation_Node");

  std::vector<FrameData> Frames = readDataset();
  ROS_INFO(" size %i" , (int)Frames.size());

  std::unique_ptr<Transformation_EstimatorNodeHandler> nh(new Transformation_EstimatorNodeHandler);

  ROS_INFO("Reading dataset done!");
  TFMatrix robot_pose;
  robot_pose = TFMatrix::Identity();
  bool done = false;
  int scenes_start = 0 , scenes_end = Frames.size();
  int first_scn = scenes_start , second_scn = first_scn+1;
  for(int i = scenes_start ; i <= scenes_end ; i ++)
  {
    ROS_INFO("%i" , i);
    FrameData previousFrame = Frames[first_scn];
    FrameData currentFrame = Frames[second_scn];
    TFMatrix tf;
    tf = TFMatrix::Zero();

    done = nh->estimateTransformBetween2Scenes(previousFrame,currentFrame,tf);
    second_scn++;
    if(done)
    {
      first_scn++;
      robot_pose = tf * robot_pose;
    }
  }
  return 0;
}
