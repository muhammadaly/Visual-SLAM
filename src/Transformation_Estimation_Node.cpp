#include <math.h>
#include <memory>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>


#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

static const int ORBFeatureVectorLength = 32;
static const int BRISKFeatureVectorLength = 64;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::Histogram<BRISKFeatureVectorLength> FeatureT;
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
        previousPC = currentPC;
        visualizePointCloud(previousPC , currentPC);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Transformation_Estimation_Node");

    Transformation_Estimator estimator;

    ros::spin();

    return 0;
}
