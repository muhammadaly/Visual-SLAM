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

class MappingNodeHandler
{
public:
  MappingNodeHandler();
  void OdometryCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void SceneDataCallback(const visual_slam_msgs::scene::ConstPtr& msg);

private:
  ros::NodeHandle node;
  ros::Subscriber odom_sub, scene_sub;
  std::unique_ptr<G2OMapOptimizer> mapOptimizer;
  std::unique_ptr<FeatureMatcher> featureMatcher;
  Pose_6D prevPose;
  int prevNodeId;
  short numberOfNode;
  std::vector<int> NodeIds;
  std::vector<std::pair<cv::Mat,int>> FeatureMap;

  void updateGraph();
  void publishOptomizedPath();
  void detectLoopClosure(cv::Mat currentFeature, Pose_6D Pose, int nodeId);
  int addToMap(Pose_6D newPose);
  int searchForSimilerScene(cv::Mat);
};

MappingNodeHandler::MappingNodeHandler()
{
  odom_sub = node.subscribe("robot_pose",50,&MappingNodeHandler::OdometryCallback , this);
  scene_sub = node.subscribe("scene_data",50,&MappingNodeHandler::SceneDataCallback,this);
  mapOptimizer = std::unique_ptr<G2OMapOptimizer>(new G2OMapOptimizer);
  featureMatcher = std::unique_ptr<CVFLANNFeatureMatcher>(new CVFLANNFeatureMatcher);
  prevPose = Pose_6D::Identity();
  mapOptimizer->addPoseToGraph(prevPose, prevNodeId);
  numberOfNode = 0;
  NodeIds.push_back(prevNodeId);
}


void MappingNodeHandler::OdometryCallback(const geometry_msgs::Pose::ConstPtr &msg)
{

}

void MappingNodeHandler::SceneDataCallback(const visual_slam_msgs::scene::ConstPtr &msg)
{
  // add new node
  Pose_6D newPose ;
  tf::poseMsgToEigen(msg->pose, newPose);

  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg->features, sensor_msgs::image_encodings::BGR8);
  cv::Mat currentSceneFeaturesDes = cv_ptr->image;

  addToMap(newPose);
  detectLoopClosure(currentSceneFeaturesDes,prevPose,prevNodeId);
}

void MappingNodeHandler::detectLoopClosure(cv::Mat currentFeature, Pose_6D Pose, int nodeId)
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
int MappingNodeHandler::addToMap(Pose_6D newPose)
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

int MappingNodeHandler::searchForSimilerScene(cv::Mat pCurrentDescriptors)
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
void MappingNodeHandler::updateGraph()
{
  mapOptimizer->optimize();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "Mapping");

  MappingNodeHandler handler;

  ros::spin();

    return 0;
}

