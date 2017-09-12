#ifndef ITERATIVECLOSESTPOINT_H
#define ITERATIVECLOSESTPOINT_H

namespace visual_slam {
class IterativeClosestPoint
{
  IterativeClosestPoint();
  void calculateError(visual_slam::PointCloudT::ConstPtr,visual_slam::PointCloudT::ConstPtr,double&);
};
}
#endif // ITERATIVECLOSESTPOINT_H
