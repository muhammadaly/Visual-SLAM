#include "ROSUtilities.h"

visual_slam::ROSUtilities::ROSUtilities()
{

}
bool visual_slam::ROSUtilities::asyncFrameDrop(ros::Time depth, ros::Time rgb)
{
  long rgb_timediff = abs(static_cast<long>(rgb.nsec) - static_cast<long>(depth.nsec));
  if(rgb_timediff > 33333333){
     ROS_DEBUG("Depth image time: %d - %d", depth.sec,   depth.nsec);
     ROS_DEBUG("RGB   image time: %d - %d", rgb.sec, rgb.nsec);
     ROS_INFO("Depth and RGB image off more than 1/30sec: %li (nsec)", rgb_timediff);
     if(ParameterServer::instance()->get<bool>("drop_async_frames")){
       ROS_WARN("Asynchronous frames ignored. See parameters if you want to keep async frames.");
       return true;
     }
  } else {
     ROS_DEBUG("Depth image time: %d - %d", depth.sec,   depth.nsec);
     ROS_DEBUG("RGB   image time: %d - %d", rgb.sec, rgb.nsec);
  }
  return false;
}
