#include <memory>
#include <boost/scoped_ptr.hpp>

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <visual_slam/Transformation_Estimation/TransformationEstimator.h>
#include <visual_slam/Transformation_Estimation/PCL3DRANSACTransformationEstimator.h>

void visualizePointCloud(PointCloudT::ConstPtr cloud , PointCloudT::ConstPtr adjusted_cloud)
{
    pcl::visualization::PCLVisualizer visu("Point Cloud Visualizer");
    visu.addPointCloud(cloud,ColorHandlerT(cloud, 0.0, 0.0, 255.0), "cloud");
    visu.addPointCloud(adjusted_cloud,ColorHandlerT(adjusted_cloud, 0.0, 255.0, 0.0), "adjusted_cloud");
    visu.spin();
}

class Transformation_EstimatorNodeHandler {
public:
    Transformation_EstimatorNodeHandler();
    void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
private:
    ros::NodeHandle _node;
    ros::Subscriber pointcloud_sub ;

    PointCloudT::Ptr previousPC , currentPC ;

    std::unique_ptr<TransformationEstimator> tfEstimator;
    Eigen::Matrix4f estimateTransform();
};


Transformation_EstimatorNodeHandler::Transformation_EstimatorNodeHandler()
{
    pointcloud_sub = _node.subscribe<sensor_msgs::PointCloud2> ("/cloud_in", 1, &Transformation_EstimatorNodeHandler::PointCloudCallback, this);
    tfEstimator = std::unique_ptr<PCL3DRANSACTransformationEstimator>(new PCL3DRANSACTransformationEstimator);
}

void Transformation_EstimatorNodeHandler::PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    if(!previousPC)
    {
        previousPC = PointCloudT::Ptr(new PointCloudT);
        pcl::fromROSMsg(*cloud_msg, *previousPC);
        printf("First one !");
    }
    else
    {
        currentPC = PointCloudT::Ptr(new PointCloudT);
        pcl::fromROSMsg(*cloud_msg, *currentPC);
        printf("New one !");
        estimateTransform();
        previousPC = currentPC;
        visualizePointCloud(previousPC , currentPC);
    }
}

Eigen::Matrix4f Transformation_EstimatorNodeHandler::estimateTransform()
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Transformation_Estimation_Node");

    boost::scoped_ptr<Transformation_EstimatorNodeHandler> nh(new Transformation_EstimatorNodeHandler);

    ros::spin();

    return 0;
}
