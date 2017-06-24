/*
 * edge_pose_pose.cpp
 *
 *  Created on: Jan 19, 2013
 *      Author: ross kidson
 */

// for point transformations
#include "custom_types/edge_pose_pose.h"
#include "custom_types/draw_functions.hpp"
#include "custom_types/math_functions.hpp"

bool visual_slam::EdgePosePose::write(std::ostream& os) const
{
  Eigen::Quaterniond q(measurement().matrix().topLeftCorner<3,3>()); // extract rotation part
  q.normalize();
  std::vector<double> est(7);
  est[0] = measurement().translation()[0];
  est[1] = measurement().translation()[1];
  est[2] = measurement().translation()[2];
  est[3] = q.x();
  est[4] = q.y();
  est[5] = q.z();
  est[6] = q.w();

  for (int i=0; i<7; i++)
    os << est[i] << " ";

  //information matrix
  for (int i=0; i<information().rows(); i++)
    for (int j=0; j<information().cols(); j++)
      os <<  information()(i,j) << " ";

  return os.good();
}

bool visual_slam::EdgePosePose::read(std::istream& is)
{
  std::vector<double> est(7);
  for (int i=0; i<7; i++)
    is  >> est[i];

  Eigen::Isometry3d t;
  t= Eigen::Quaterniond(est[6], est[3], est[4], est[5]).toRotationMatrix();
  t.translation() = Eigen::Vector3d(est[0], est[1], est[2]);
  setMeasurement(t);

  if (is.bad()) {
    return false;
  }
  for ( int i=0; i<information().rows() && is.good(); i++)
    for (int j=0; j<information().cols() && is.good(); j++)
      is >> information()(i,j);

  if (is.bad()) {
    //  we overwrite the information matrix with the Identity
    information().setIdentity();
  }

  return true;
}

void visual_slam::EdgePosePose::computeError()
{
  const VertexPose * w_T_a
      = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose * w_T_b
      = static_cast<const VertexPose*>(_vertices[1]);

  _error = fromIsometry(w_T_a->estimate() * _measurement * w_T_b->estimate().inverse());
}
