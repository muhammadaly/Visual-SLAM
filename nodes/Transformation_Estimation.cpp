#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

static const int ORBFeatureVectorLength = 32;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::Histogram<ORBFeatureVectorLength> FeatureT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> ColorHandlerT;

void visualizePointCloud(PointCloudT::ConstPtr cloud , PointCloudT::ConstPtr adjusted_cloud)
{
    pcl::visualization::PCLVisualizer visu("Point Cloud Visualizer");
    visu.addPointCloud(cloud,ColorHandlerT(cloud, 0.0, 0.0, 255.0), "cloud");
    visu.addPointCloud(adjusted_cloud,ColorHandlerT(adjusted_cloud, 0.0, 255.0, 0.0), "adjusted_cloud");
    visu.spin();
}

class Transformation_Estimator {
public:
    Transformation_Estimator();
    void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
private:
    ros::NodeHandle _node;
    ros::Subscriber pointcloud_sub ;

    PointCloudT::Ptr previousPC , currentPC ;

    Eigen::Matrix4f estimateTransform();
};


Transformation_Estimator::Transformation_Estimator()
{
    pointcloud_sub = _node.subscribe<sensor_msgs::PointCloud2> ("/scan_pc_f", 1, &Transformation_Estimator::PointCloudCallback, this);
}

void Transformation_Estimator::PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    if(!previousPC)
    {
        previousPC = PointCloudT::Ptr(new PointCloudT);
        pcl::fromROSMsg(*cloud_msg, *previousPC);
    }
    else
    {
        currentPC = PointCloudT::Ptr(new PointCloudT);
        pcl::fromROSMsg(*cloud_msg, *currentPC);
        estimateTransform();
        previousPC = currentPC;
        visualizePointCloud(previousPC , currentPC);
    }
}

Eigen::Matrix4f Transformation_Estimator::estimateTransform()
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Transformation_Estimation_Node");

    Transformation_Estimator estimator;

    ros::spin();

    return 0;
}
