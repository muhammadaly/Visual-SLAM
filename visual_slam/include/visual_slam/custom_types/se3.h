#ifndef SE3_H
#define SE3_H
#include "g2o/stuff/misc.h"
#include "g2o/stuff/macros.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class SE3 {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    SE3();
    SE3(double Tx, double Ty, double Tz,
        double Rw, double Rroll, double Rpitch, double Ryaw);

    const Eigen::Vector3d& translation() const {return _t;}
    Eigen::Vector3d& translation() {return _t;}

    const Eigen::Matrix<double, 3, 3>& rotation() const {return _R;}
    Eigen::Matrix<double, 3, 3>& rotation() {return _R;}

    SE3 operator * (const SE3& tr2) const;
    SE3& operator *= (const SE3& tr2);
    Eigen::Vector3d operator * (const Eigen::Vector3d& v) const ;

    SE3 inverse() const{
      SE3 ret;
      ret._R=_R.inverse();
      ret._R.angle()=normalize_theta(ret._R.angle());
      ret._t=ret._R*(Eigen::Vector2d(-1 * _t));
      return ret;
    }

    double operator [](int i) const {
      assert (i>=0 && i<3);
      if (i<2)
        return _t(i);
      return _R.angle();
    }

    double& operator [](int i) {
      assert (i>=0 && i<3);
      if (i<2)
        return _t(i);
      return _R.angle();
    }

    void fromVector (const Eigen::Vector3d& v){
      *this=SE2(v[0], v[1], v[2]);
    }

    Eigen::Vector3d toVector() const {
      Eigen::Vector3d ret;
      for (int i=0; i<3; i++){
        ret(i)=(*this)[i];
      }
      return ret;
    }

  private:
    Eigen::Matrix3d _R;
    Eigen::Vector3d _t;

    double normalize_theta(double theta);
};

#endif // SE3_H

