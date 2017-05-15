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

class Trajectory_EstimationNodeHandler {
public:
  Trajectory_EstimationNodeHandler(std::vector<FrameData>);
  bool estimateTransformBetween2Scenes(int previousFrameId, int currentFrameId, TFMatrix& transformation);
  void process();
  void addToMap(Pose_6D newPose, int previousSceneInd, int currentSceneInd);
  void detectLoopClosure(int);

  void publishOnTF(TFMatrix);
  void publishOdometry(TFMatrix);
  void publishPose(TFMatrix);
  void publishFullPath(TFMatrix);

  void map(TFMatrix robot_pose, int previousSceneInd, int currentSceneInd);

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
  short numberOfNode = 0;

  std::unique_ptr<G2OMapOptimizer> mapOptimizer;
  std::vector<FrameData> Frames;
  void updateGraph();
  void publishOptomizedPath();
  int searchForSimilerScene(cv::Mat);
  Pose_6D convertToPose(TFMatrix);
};

Trajectory_EstimationNodeHandler::Trajectory_EstimationNodeHandler(std::vector<FrameData> pFrames)
{
  Frames = pFrames;
  tfEstimator = std::unique_ptr<PCL3DRANSACTransformationEstimator>(new PCL3DRANSACTransformationEstimator);
  featureExtractorAndDescriptor = std::unique_ptr<CVORBFeatureExtractorAndDescriptor>(new CVORBFeatureExtractorAndDescriptor);
  featureMatcher = std::unique_ptr<CVFLANNFeatureMatcher>(new CVFLANNFeatureMatcher);
  odom_pub = _node.advertise<nav_msgs::Odometry>("odom", 50);
  pose_pub = _node.advertise<geometry_msgs::PoseStamped>("robot_pose",50);
  path_pub = _node.advertise<nav_msgs::Path>("robot_path",50);
  scene_pub = _node.advertise<visual_slam_msgs::scene>("scene_data",50);
  fullPath.clear();

  mapOptimizer = std::unique_ptr<G2OMapOptimizer>(new G2OMapOptimizer);
  numberOfNode = 0;
}
void Trajectory_EstimationNodeHandler::process()
{
  TFMatrix robot_pose = TFMatrix::Identity();
  int scenes_start = 0,
      scenes_end = Frames.size();
  int first_scn = scenes_start,
      second_scn = first_scn+1;
  bool done = false;
  while(second_scn != scenes_end)
  {
    ROS_INFO("first ind %i , second ind %i" , first_scn , second_scn);

    TFMatrix tf = TFMatrix::Zero();
    done = estimateTransformBetween2Scenes(first_scn,second_scn,tf);
    second_scn++;
    if(done)
    {
      first_scn++;
      second_scn = first_scn + 1;
      robot_pose = tf * robot_pose;
      Frames[second_scn].setRobotPose(robot_pose);
      map(robot_pose, first_scn, second_scn);
    }
  }
}
bool Trajectory_EstimationNodeHandler::estimateTransformBetween2Scenes(int previousFrameId, int currentFrameId, TFMatrix& transformation)
{
  std::vector<cv::KeyPoint> tPreviousKeypoints , tCurrentKeypoints ;
  bool done = false;
  std::vector<cv::DMatch> matches;
  cv::Mat tPreviousDescriptors,tCurrentDescriptors;
  PointCloudT::Ptr tCurrentKeypointsPC (new PointCloudT),tPreviousKeypointsPC (new PointCloudT) , alignedPC(new PointCloudT);
  FeatureCloudT::Ptr tCurrentDescriptorsPC (new FeatureCloudT),tPreviousDescriptorsPC (new FeatureCloudT);

  featureExtractorAndDescriptor->computeDescriptors(Frames[previousFrameId] , tPreviousKeypoints , tPreviousDescriptors);
  featureExtractorAndDescriptor->computeDescriptors(Frames[currentFrameId] , tCurrentKeypoints , tCurrentDescriptors);
  Frames[previousFrameId].setSceneFeatureDescriptors(tPreviousDescriptors);
  Frames[currentFrameId].setSceneFeatureDescriptors(tCurrentDescriptors);

  featureMatcher->matching2ImageFeatures(tPreviousDescriptors , tCurrentDescriptors,matches);
  pclUTL.getKeypointsAndDescriptors(matches,tPreviousKeypoints,tCurrentKeypoints,
                                    tPreviousDescriptors,tCurrentDescriptors,Frames[previousFrameId],Frames[currentFrameId],
                                    tPreviousKeypointsPC,tCurrentKeypointsPC,
                                    tPreviousDescriptorsPC,tCurrentDescriptorsPC);
  done = tfEstimator->estimateTransformation(tPreviousKeypointsPC,tPreviousDescriptorsPC,tCurrentKeypointsPC,tCurrentDescriptorsPC,transformation,alignedPC);
  return done;
}
void Trajectory_EstimationNodeHandler::publishOnTF(TFMatrix transformation)
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
void Trajectory_EstimationNodeHandler::publishOdometry(TFMatrix robot_pose)
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
void Trajectory_EstimationNodeHandler::publishPose(TFMatrix robot_pose)
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
void Trajectory_EstimationNodeHandler::publishFullPath(TFMatrix robot_pose)
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
Pose_6D Trajectory_EstimationNodeHandler::convertToPose(TFMatrix transformation)
{
  Pose_6D tmp;
  Eigen::Matrix<double, 4, 4> tmpMat ;
  for(int i = 0 ; i < 4  ; i ++)
    for(int j = 0 ; j < 4  ; j ++)
      tmpMat(i,j) = (double)transformation(i,j);
  tmp.matrix() = tmpMat;
  return tmp;
}

void Trajectory_EstimationNodeHandler::detectLoopClosure(int currentSceneInd)
{
  FrameData currentFrame = Frames[currentSceneInd];
  cv::Mat currentFeature = currentFrame.getSceneFeatureDescriptors();
  int currentNodeId = currentFrame.getGraphNodeId();
  Pose_6D Pose = convertToPose(currentFrame.getRobotPose());
  int similerNodeId = searchForSimilerScene(currentFeature);
  if(similerNodeId > 0)
    mapOptimizer->addEdge(Pose ,currentNodeId ,similerNodeId);
}
void Trajectory_EstimationNodeHandler::addToMap(Pose_6D newPose, int previousSceneInd, int currentSceneInd)
{
  int newNodeId;
  mapOptimizer->addPoseToGraph(newPose, newNodeId);
  Frames[currentSceneInd].setGraphNodeId( newNodeId);
  int prevNodeId = Frames[previousSceneInd].getGraphNodeId();
  mapOptimizer->addEdge(newPose ,newNodeId ,prevNodeId);
  if(numberOfNode ==10)
  {
    numberOfNode = 0;
    updateGraph();
  }
}
int Trajectory_EstimationNodeHandler::searchForSimilerScene(cv::Mat pCurrentDescriptors)
{
  double threshold = 0.8;
  for(int i = Frames.size()-2 ; i >=0 ; i--)
  {
    std::vector<cv::DMatch> matches;
    cv::Mat previousSceneFeatureDescriptors = Frames[i].getSceneFeatureDescriptors();
    featureMatcher->matching2ImageFeatures(previousSceneFeatureDescriptors , pCurrentDescriptors,matches);
    if((matches.size() / pCurrentDescriptors.rows) > threshold)
    {
      return Frames[i].getGraphNodeId();
    }
  }
  return -1;
}
void Trajectory_EstimationNodeHandler::updateGraph()
{
  mapOptimizer->optimize();
}
void Trajectory_EstimationNodeHandler::map(TFMatrix robot_pose, int previousSceneInd, int currentSceneInd)
{
  Pose_6D newPose = convertToPose(robot_pose);
  addToMap(newPose , previousSceneInd , currentSceneInd);

  detectLoopClosure(currentSceneInd);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Transformation_Estimation_Node");

  std::vector<FrameData> Frames = readDataset();
  ROS_INFO(" size %i" , (int)Frames.size());

  std::unique_ptr<Trajectory_EstimationNodeHandler> nh(new Trajectory_EstimationNodeHandler(Frames));
  nh->process();
  return 0;
}
