#include "myHeader.h"
#include <pcl_conversions/pcl_conversions.h>

visual_slam::myHeader::myHeader()
{
  seq = 0;
}

visual_slam::myHeader::myHeader(pcl::PCLHeader h) {
  std_msgs::Header rh = pcl_conversions::fromPCL(h);
  fromRosHeader(rh);
}

visual_slam::myHeader::myHeader(std_msgs::Header rh){
  fromRosHeader(rh);
}

visual_slam::myHeader::myHeader(uint32_t aseq, ros::Time astamp, std::string aframe_id)
{
  seq = aseq;
  stamp = astamp;
  frame_id = aframe_id;
}

std_msgs::Header visual_slam::myHeader::toRosHeader()
{
  std_msgs::Header rh;
  rh.seq = seq;
  rh.stamp = stamp;
  rh.frame_id = frame_id;
  return rh;
}


void visual_slam::myHeader::fromRosHeader(std_msgs::Header rh){
  seq = rh.seq;
  stamp = rh.stamp;
  frame_id = rh.frame_id;
}


visual_slam::myHeader::operator pcl::PCLHeader()
{
  return pcl_conversions::toPCL(this->toRosHeader());
}
visual_slam::myHeader::operator std_msgs::Header()
{
  return this->toRosHeader();
}
