#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>

#include <visual_slam/definitions.h>
#include <visual_slam/Map_Optimization/g2omapoptimizer.h>
class MappingNodeHandler
{
public:
  MappingNodeHandler();
  void OdometryCallback(const geometry_msgs::Pose::ConstPtr& msg);

private:
  ros::NodeHandle node;
  ros::Subscriber odom_sub;
  std::unique_ptr<G2OMapOptimizer> mapOptimizer;
  Pose_6D prevPose;
  int prevNodeId;
  short numberOfNode;
  std::vector<int> NodeIds;

  void updateGraph();
  void publishOptomizedPath();
};

MappingNodeHandler::MappingNodeHandler()
{
  odom_sub = node.subscribe("robot_pose",50,&MappingNodeHandler::OdometryCallback , this);
  mapOptimizer = std::unique_ptr<G2OMapOptimizer>(new G2OMapOptimizer);
  prevPose = Pose_6D::Identity();
  mapOptimizer->addPoseToGraph(prevPose, prevNodeId);
  numberOfNode = 0;
  NodeIds.push_back(prevNodeId);
}

void MappingNodeHandler::OdometryCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
  Pose_6D newPose ;
  tf::poseMsgToEigen(*msg, newPose);
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

