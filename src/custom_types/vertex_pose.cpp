
#include "visual_slam/custom_types/vertex_pose.h"
#include "visual_slam/custom_types/draw_functions.hpp"
#include "g2o/core/factory.h"
#include "g2o/stuff/opengl_wrapper.h"

#include <iostream>
#include "g2o/core/cache.h"

const static double DEFAULT_COORDINATE_SIZE = 1.5;

  VertexPose::VertexPose() :
    g2o::BaseVertex<6, Eigen::Isometry3d>(),
    _numOplusCalls(0)
  {
    setToOriginImpl();
    updateCache();
  }

  bool VertexPose::read(std::istream& is)
  {
    std::vector<double> est(7);
    for (int i=0; i<7; i++)
      is  >> est[i];

    Eigen::Isometry3d t;
    t= Eigen::Quaterniond(est[6], est[3], est[4], est[5]).toRotationMatrix();
    t.translation() = Eigen::Vector3d(est[0], est[1], est[2]);
    setEstimate(t);
    return true;
  }

  bool VertexPose::write(std::ostream& os) const
  {
    Eigen::Quaterniond q(_estimate.matrix().topLeftCorner<3,3>()); // extract rotation part
    q.normalize();
    std::vector<double> est(7);
    est[3] = q.x();
    est[4] = q.y();
    est[5] = q.z();
    est[6] = q.w();
    est[0] = _estimate.translation()[0];
    est[1] = _estimate.translation()[1];
    est[2] = _estimate.translation()[2];

    for (int i=0; i<7; i++)
      os << est[i] << " ";
    return os.good();
  }
