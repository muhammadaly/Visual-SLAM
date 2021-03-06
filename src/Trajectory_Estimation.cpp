#include <memory>
#include <boost/scoped_ptr.hpp>

#include "ros/ros.h"
#include <eigen_conversions/eigen_msg.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

#include <definitions.h>
#include <framedata.h>

#include <Transformation_Estimation/PCL3DRANSACTransformationEstimator.h>
#include <Feature_Extraction/CVSURFFeatureExtractorAndDescriptor.h>
#include <Feature_Extraction/CVSIFTFeatureExtractorAndDescriptor.h>
#include <Feature_Extraction/CVORBFeatureExtractorAndDescriptor.h>
#include <Feature_Matching/cvflannfeaturematcher.h>
#include "Utilities/PCLUtilities.h"
#include "Utilities/LogFile.h"
#include "Utilities/TUMUtilities.h"
#include <Map_Optimization/g2omapoptimizer.h>
#include "robot_pose.h"

namespace visual_slam {
std::string machineName = "ivsystems";
std::string datasetName = "rgbd_dataset_freiburg1_xyz";
std::string datasetDIR = "/home/"+machineName+"/master_dataset/"+datasetName+"/";
std::string rgbImages = datasetDIR + "rgb/";
std::string depthImages = datasetDIR + "depth/";
std::string transformationMatrices = datasetDIR + "transformationMatricesPCL.txt";
std::string featureMatching = datasetDIR + "matching/";
std::string frames_matching = datasetDIR + "matching_frames.txt";
std::string ground_truth_filename = "/home/"+machineName+"/ros_workspaces/master/src/tum-tools-fork/data/rgbdslam/freiburg1_xyz-rgbdslam.txt";
std::string result_filename = datasetDIR + "result.txt";
static int numberOfEstimationFailure = 0;
static int numberOfLoopClosure = 0;

visual_slam::PointCloudT::Ptr GeneratePointCloud(cv::Mat pImage)
{
  visual_slam::PointCloudT::Ptr cloud (new visual_slam::PointCloudT);

  float Z , factor = 5000;
  for(int rowInd = 0 ; rowInd < pImage.rows ; rowInd++)
    for(int colInd = 0 ; colInd < pImage.cols ; colInd++)
    {
      Z = pImage.at<u_int16_t>(rowInd , colInd) / factor;
      visual_slam::PointT p;
      p.x = ((colInd - visual_slam::cx) * Z )/visual_slam::fx;
      p.y = ((rowInd - visual_slam::cy) * Z )/visual_slam::fy;
      p.z = Z;

      cloud->points.push_back(p);
    }
  return cloud;
}
void visualizePointCloud(visual_slam::PointCloudT::ConstPtr secondPC , visual_slam::PointCloudT::ConstPtr firstPC, visual_slam::PointCloudT::ConstPtr aligned)
{
  pcl::visualization::PCLVisualizer visu("Point Cloud Visualizer");
  visu.addPointCloud(secondPC,visual_slam::ColorHandlerT(secondPC, 0.0, 0.0, 255.0), "secondPC");
  visu.addPointCloud(firstPC,visual_slam::ColorHandlerT(firstPC, 0.0, 255.0, 0.0), "firstPC");
  visu.addPointCloud(aligned,visual_slam::ColorHandlerT(aligned, 255.0, 255.0, 255.0), "aligned");
  visu.spin();
}
void visualizePointCloud(visual_slam::PointCloudT::ConstPtr secondPC , visual_slam::PointCloudT::ConstPtr firstPC)
{
  pcl::visualization::PCLVisualizer visu("Point Cloud Visualizer");
  visu.addPointCloud(secondPC,visual_slam::ColorHandlerT(secondPC, 255.0, 255.0, 255.0), "secondPC");
  visu.addPointCloud(firstPC,visual_slam::ColorHandlerT(firstPC, 0.0, 255.0, 0.0), "firstPC");
  visu.spin();
}
void visualizePointCloud(visual_slam::PointCloudT::ConstPtr secondPC)
{
  pcl::visualization::PCLVisualizer visu("Point Cloud Visualizer");
  visu.addPointCloud(secondPC,visual_slam::ColorHandlerT(secondPC, 0.0, 255.0, 0.0), "secondPC");
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
        frames.push_back(FrameData(image , Dimage, (depthImages+depthFiles[i]).c_str()));
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

class Trajectory_EstimationNodeHandler
{
public:
  Trajectory_EstimationNodeHandler(std::vector<FrameData>);
  bool estimateTransformBetween2Scenes(int previousFrameId, int currentFrameId, visual_slam::TFMatrix& transformation);
  void process();
  void addToMap(visual_slam::Pose_6D newPose,visual_slam::Pose_6D newTransformation, int previousSceneInd, int currentSceneInd);
  void detectLoopClosure(int);

  void publishOnTF(visual_slam::Robot_Pose);
  void publishOdometry(visual_slam::Robot_Pose);
  void publishPose(visual_slam::Robot_Pose);
  void publishFullPath(visual_slam::Robot_Pose);

  void initializeGraphMap();
  void map(visual_slam::TFMatrix robot_pose, visual_slam::TFMatrix lastTransformation,int previousSceneInd, int currentSceneInd);

private:
  ros::NodeHandle _node;

  std::unique_ptr<TransformationEstimator> tfEstimator;
  std::unique_ptr<FeatureExtractorAndDescriptor> featureExtractorAndDescriptor;
  std::unique_ptr<FeatureMatcher> featureMatcher;
  PCLUtilities pclUTL;
  //  std::unique_ptr<LogFile> log;
  std::unique_ptr<TUMUtilities> utility;

  visual_slam::PointCloudT::Ptr accumilatedPC;

  ros::Publisher odom_pub ;
  ros::Publisher pose_pub;
  ros::Publisher path_pub;
  ros::Publisher scene_pub;
  tf::TransformBroadcaster br;
  std::vector<geometry_msgs::PoseStamped> fullPath;

  std::unique_ptr<G2OMapOptimizer> mapOptimizer;
  std::vector<FrameData> Frames;
  void updateGraph();
  void publishOptomizedPath();
  int searchForSimilerScene(int);
  visual_slam::Pose_6D convertToPose(visual_slam::TFMatrix);
  void testingTransformation(visual_slam::PointCloudT::ConstPtr,visual_slam::PointCloudT::ConstPtr , visual_slam::TFMatrix&);
  void accumilatePC(visual_slam::PointCloudT::Ptr newPC, visual_slam::TFMatrix& transformation);
};

}

visual_slam::Trajectory_EstimationNodeHandler::Trajectory_EstimationNodeHandler(std::vector<FrameData> pFrames)
{
  Frames = pFrames;
  tfEstimator = std::unique_ptr<PCL3DRANSACTransformationEstimator>(new PCL3DRANSACTransformationEstimator);
  featureExtractorAndDescriptor = std::unique_ptr<CVSIFTFeatureExtractorAndDescriptor>(new CVSIFTFeatureExtractorAndDescriptor);
  featureMatcher = std::unique_ptr<CVFLANNFeatureMatcher>(new CVFLANNFeatureMatcher);
  odom_pub = _node.advertise<nav_msgs::Odometry>("odom", 50);
  pose_pub = _node.advertise<geometry_msgs::PoseStamped>("robot_pose",50);
  path_pub = _node.advertise<nav_msgs::Path>("robot_path",50);
  fullPath.clear();

  accumilatedPC = visual_slam::PointCloudT::Ptr(new visual_slam::PointCloudT);

  //  log = std::unique_ptr<LogFile>(new LogFile(datasetDIR));
  utility = std::unique_ptr<TUMUtilities>( new TUMUtilities(ground_truth_filename , result_filename));

  mapOptimizer = std::unique_ptr<G2OMapOptimizer>(new G2OMapOptimizer);
  initializeGraphMap();
}
void visual_slam::Trajectory_EstimationNodeHandler::accumilatePC(visual_slam::PointCloudT::Ptr newPC, visual_slam::TFMatrix& transformation)
{
  visual_slam::PointCloudT::Ptr transformed_cloud(new visual_slam::PointCloudT);
  pcl::transformPointCloud (*newPC, *transformed_cloud, transformation);
  *accumilatedPC += *transformed_cloud;
}

void visual_slam::Trajectory_EstimationNodeHandler::process()
{
  visual_slam::Robot_Pose robot_pose(0.0,0.0,0.0,0.0,0.0,0.0);
  int scenes_start = 0,
      scenes_end = Frames.size()-1;
  int first_scn = scenes_start,
      second_scn = first_scn+1;
  int numberOfFramesThreshold = 10;
  bool done = false;
  while(first_scn != scenes_end)
  {
    if(second_scn - first_scn > numberOfFramesThreshold)
    {
      first_scn ++;
      second_scn = first_scn+1;
      ROS_INFO("Estimation failed");
      numberOfEstimationFailure++;
    }
    else
    {
      ROS_INFO("Start estimate transformation between ( %i , %i )" , first_scn , second_scn);
      visual_slam::TFMatrix tf = visual_slam::TFMatrix::Zero();
      done = estimateTransformBetween2Scenes(first_scn,second_scn,tf);
      if(done)
      {
//        ROS_INFO("Estimation Successed");
        visual_slam::RobotPose6D PoseVec = robot_pose.getRobotPose();
        ROS_INFO("%f %f %f %f %f %f", PoseVec(0), PoseVec(1), PoseVec(2), PoseVec(3), PoseVec(4), PoseVec(5));
        visual_slam::TFMatrix Inverse_transformation = robot_pose.getInverseTransformation(tf);

        robot_pose *= Inverse_transformation ;

        publishOnTF(robot_pose);
        publishOdometry(robot_pose);
//        ROS_INFO("Start Mapping");
//        map(robot_pose_matrix, tf, first_scn, second_scn);
//        ROS_INFO("Mapping finished");
        first_scn = second_scn;
        second_scn = first_scn + 1;
      }
      else
      {
        second_scn ++;
      }
      ROS_INFO("---------------------------------------------------------------");
    }
  }
  visual_slam::visualizePointCloud(accumilatedPC);
  updateGraph();
  std::vector<visual_slam::Pose_6D> v = mapOptimizer->getPoses();
  ROS_INFO("Number of loop closure detected %i" , numberOfLoopClosure );
  ROS_INFO("Number of loop transformation estimation failure %i" , numberOfEstimationFailure);
  utility->writingResults(v);
}
bool visual_slam::Trajectory_EstimationNodeHandler::estimateTransformBetween2Scenes(int previousFrameId, int currentFrameId, visual_slam::TFMatrix& transformation)
{
  std::vector<cv::KeyPoint> tPreviousKeypoints , tCurrentKeypoints ;
  bool done = false;
  std::vector<cv::DMatch> matches;
  cv::Mat tPreviousDescriptors,tCurrentDescriptors;
  visual_slam::PointCloudT::Ptr tCurrentKeypointsPC (new visual_slam::PointCloudT),tPreviousKeypointsPC (new visual_slam::PointCloudT) , alignedPC(new visual_slam::PointCloudT);
  visual_slam::FeatureCloudT::Ptr tCurrentDescriptorsPC (new visual_slam::FeatureCloudT),tPreviousDescriptorsPC (new visual_slam::FeatureCloudT);

  featureExtractorAndDescriptor->computeDescriptors(Frames[previousFrameId] , tPreviousKeypoints , tPreviousDescriptors);
  featureExtractorAndDescriptor->computeDescriptors(Frames[currentFrameId] , tCurrentKeypoints , tCurrentDescriptors);
  Frames[previousFrameId].setDescriptors(tPreviousDescriptors);
  Frames[currentFrameId].setDescriptors(tCurrentDescriptors);
  featureMatcher->matching2ImageFeatures(tPreviousDescriptors , tCurrentDescriptors,matches);
  pclUTL.getKeypointsAndDescriptors(matches,tPreviousKeypoints,tCurrentKeypoints,
                                    tPreviousDescriptors,tCurrentDescriptors,Frames[previousFrameId],Frames[currentFrameId],
                                    tPreviousKeypointsPC,tCurrentKeypointsPC,
                                    tPreviousDescriptorsPC,tCurrentDescriptorsPC);
  done = tfEstimator->estimateTransformation(tPreviousKeypointsPC,tPreviousDescriptorsPC,tCurrentKeypointsPC,tCurrentDescriptorsPC,transformation,alignedPC);

//  if(done)
//  {
//    visual_slam::PointCloudT::Ptr newPC = visual_slam::GeneratePointCloud(Frames[currentFrameId].getDepthMatrix());
    //accumilatePC(newPC,transformation);
//    testingTransformation(tPreviousKeypointsPC , tCurrentKeypointsPC , transformation);
//  }
  return done;
}
void visual_slam::Trajectory_EstimationNodeHandler::testingTransformation(visual_slam::PointCloudT::ConstPtr tPreviousKeypointsPC, visual_slam::PointCloudT::ConstPtr tCurrentKeypointsPC,visual_slam::TFMatrix & transformation)
{
  visual_slam::PointCloudT::Ptr transformed_cloud (new visual_slam::PointCloudT);
  pcl::transformPointCloud(*tPreviousKeypointsPC, *transformed_cloud, transformation);
  visualizePointCloud( tPreviousKeypointsPC, tCurrentKeypointsPC, transformed_cloud);
}
void visual_slam::Trajectory_EstimationNodeHandler::publishOnTF(visual_slam::Robot_Pose robot_pose)
{
  visual_slam::RobotPose6D PoseVec = robot_pose.getRobotPose();
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(PoseVec(3), PoseVec(4), PoseVec(5));

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = "base_link";
  odom_trans.child_frame_id = "odom";

  odom_trans.transform.translation.x = PoseVec(0);
  odom_trans.transform.translation.y = PoseVec(1);
  odom_trans.transform.translation.z = PoseVec(2);
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  br.sendTransform(odom_trans);
}
void visual_slam::Trajectory_EstimationNodeHandler::publishOdometry(visual_slam::Robot_Pose robot_pose)
{
  visual_slam::RobotPose6D PoseVec = robot_pose.getRobotPose();

  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "base_link";

  //set the position
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(PoseVec(3), PoseVec(4), PoseVec(5));
  odom.pose.pose.position.x = PoseVec(0);
  odom.pose.pose.position.y = PoseVec(1);
  odom.pose.pose.position.z = PoseVec(3);
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "odom";
  //odom.twist.twist.linear.x = vx;
  //odom.twist.twist.linear.y = vy;
  //odom.twist.twist.angular.z = vth;

  //publish the message
  odom_pub.publish(odom);
}
void visual_slam::Trajectory_EstimationNodeHandler::publishPose(visual_slam::Robot_Pose robot_pose)
{
  visual_slam::RobotPose6D PoseVec = robot_pose.getRobotPose();

  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "base_link";
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(PoseVec(3), PoseVec(4), PoseVec(5));

  msg.pose.position.x = PoseVec(0);
  msg.pose.position.y = PoseVec(1);
  msg.pose.position.z = PoseVec(3);
  msg.pose.orientation= odom_quat;

  pose_pub.publish(msg);

}
void visual_slam::Trajectory_EstimationNodeHandler::publishFullPath(visual_slam::Robot_Pose robot_pose)
{
  visual_slam::RobotPose6D PoseVec = robot_pose.getRobotPose();
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(PoseVec(3), PoseVec(4), PoseVec(5));

  geometry_msgs::PoseStamped msg;
  msg.pose.position.x = PoseVec(0);
  msg.pose.position.y = PoseVec(1);
  msg.pose.position.z = PoseVec(3);
  msg.pose.orientation= odom_quat;
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
void visual_slam::Trajectory_EstimationNodeHandler::initializeGraphMap()
{
  int initialNodeId , initialSceneInd = 0;
  visual_slam::TFMatrix initialize_tf = visual_slam::TFMatrix::Zero();
  visual_slam::Pose_6D initial_Pose = convertToPose(initialize_tf);
  mapOptimizer->addPoseToGraph(initial_Pose, initialNodeId);
  Frames[initialSceneInd].setGraphNodeId(initialNodeId);
}
visual_slam::Pose_6D  visual_slam::Trajectory_EstimationNodeHandler::convertToPose(visual_slam::TFMatrix transformation)
{
  visual_slam::Pose_6D tmp;
  Eigen::Matrix<double, 4, 4> tmpMat ;
  for(int i = 0 ; i < 4  ; i ++)
    for(int j = 0 ; j < 4  ; j ++)
      tmpMat(i,j) = (double)transformation(i,j);
  tmp.matrix() = tmpMat;
  return tmp;
}

void visual_slam::Trajectory_EstimationNodeHandler::detectLoopClosure(int currentSceneInd)
{
  int similerInd = searchForSimilerScene(currentSceneInd);
  if(similerInd > 0)
  {
    visual_slam::TFMatrix t;
    estimateTransformBetween2Scenes(similerInd, currentSceneInd, t);
    visual_slam::Pose_6D Pose = convertToPose(t);

    int similerNodeId = Frames[similerInd].getGraphNodeId();
    int currentNodeId = Frames[currentSceneInd].getGraphNodeId();
    mapOptimizer->addEdge(Pose ,currentNodeId ,similerNodeId);
    ROS_INFO("Loop detected -- ");
    numberOfLoopClosure ++;
  }
  ROS_INFO("Loop closure finished -- ");
}
void visual_slam::Trajectory_EstimationNodeHandler::addToMap(visual_slam::Pose_6D newPose,visual_slam::Pose_6D newTransformation, int previousSceneInd, int currentSceneInd)
{
  int newNodeId;
  mapOptimizer->addPoseToGraph(newPose, newNodeId);
  Frames[currentSceneInd].setGraphNodeId( newNodeId);
  ROS_INFO("New Node : %i" , newNodeId);
  int prevNodeId = Frames[previousSceneInd].getGraphNodeId();
  mapOptimizer->addEdge(newTransformation ,newNodeId ,prevNodeId);
  ROS_INFO("New Edge : ( %i , %i )" , newNodeId , prevNodeId);
}
int visual_slam::Trajectory_EstimationNodeHandler::searchForSimilerScene(int currentSceneInd)
{
  ROS_INFO("Start search for similer to %i scene -- " , currentSceneInd);
  cv::Mat currentSceneFeatureDescriptors = Frames[currentSceneInd].getDescriptors();
  double threshold = 0.95;
  for(int i = currentSceneInd -1 ; i >=0 ; i--)
  {
    ROS_INFO("compare %i to %i -- " , currentSceneInd , i);
    std::vector<cv::DMatch> matches;
    cv::Mat previousSceneFeatureDescriptors = Frames[i].getDescriptors();
    featureMatcher->matching2ImageFeatures(previousSceneFeatureDescriptors , currentSceneFeatureDescriptors,matches);
    ROS_INFO("number of matches : %i" , (int)matches.size());
    if((matches.size() / currentSceneFeatureDescriptors.rows) > threshold)
    {
      ROS_INFO("current scene similer to %i " , Frames[i].getGraphNodeId());
      return i;
    }
  }
  return -1;
}
void visual_slam::Trajectory_EstimationNodeHandler::updateGraph()
{
  ROS_INFO("Graph updating stared --- ");
  mapOptimizer->optimize();
  ROS_INFO("Graph updated --- ");
}
void visual_slam::Trajectory_EstimationNodeHandler::map(visual_slam::TFMatrix robot_pose,visual_slam::TFMatrix lastTransformation, int previousSceneInd, int currentSceneInd)
{
  visual_slam::Pose_6D newPose = convertToPose(robot_pose);
  visual_slam::Pose_6D newTransformation = convertToPose(lastTransformation);
  addToMap(newPose ,newTransformation, previousSceneInd , currentSceneInd);
  detectLoopClosure(currentSceneInd);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Trajectory_Estimation");
  ROS_INFO("Start reading dataset ---");
  std::vector<visual_slam::FrameData> Frames = visual_slam::readDataset();
  ROS_INFO("Reading dataset finished with size %i" , (int)Frames.size());

  std::unique_ptr<visual_slam::Trajectory_EstimationNodeHandler> nh(new visual_slam::Trajectory_EstimationNodeHandler(Frames));
  nh->process();
  return 0;
}
