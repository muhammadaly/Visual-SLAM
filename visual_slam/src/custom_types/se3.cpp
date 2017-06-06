#include "custom_types/se3.h"

SE3 SE3::operator *(const SE3 &tr2) const
{
  SE3 result(*this);
  result._t += _R*tr2._t;
  result._R.angle()+= tr2._R.angle();
  result._R.angle()=normalize_theta(result._R.angle());
  return result;
}

SE3 &SE3::operator *=(const SE3 &tr2)
{
  _t+=_R*tr2._t;
  _R.angle()+=tr2._R.angle();
  _R.angle()=normalize_theta(_R.angle());
  return *this;
}

Eigen::Vector3d SE3::operator *(const Eigen::Vector3d &v) const
{
  return _t+_R*v;
}

SE3::SE3(double Tx, double Ty, double Tz, double Rw, double Rroll, double Rpitch, double Ryaw)
{
  _R = AngleAxisd(Ryaw, Vector3f::UnitZ())
    * AngleAxisd(Rpitch, Vector3f::UnitY())
    * AngleAxisd(Rroll, Vector3f::UnitZ());
  _t = Eigen::Vector3d(Tx,Ty,Tz);
}
double SE3::normalize_theta(double theta)
{
  if (theta >= -M_PI && theta < M_PI)
    return theta;

  double multiplier = floor(theta / (2*M_PI));
  theta = theta - multiplier*2*M_PI;
  if (theta >= M_PI)
    theta -= 2*M_PI;
  if (theta < -M_PI)
    theta += 2*M_PI;

  return theta;
}
