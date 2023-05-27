#ifndef MathUtil_H
#define MathUtil_H
#include <cmath>
#include "pinocchio/math/fwd.hpp"

namespace MathUtil
{
  const double PI = pinocchio::PI<double>();
  // copy from choreonoid
  Eigen::Matrix3d rotFromRpy(double r, double p, double y)
  {
    const double cr = std::cos(r);
    const double sr = std::sin(r);
    const double cp = std::cos(p);
    const double sp = std::sin(p);
    const double cy = std::cos(y);
    const double sy = std::sin(y);

    Eigen::Matrix3d R;
    R << cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy,
      cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy,
      -sp  , sr*cp           , cr*cp;

    return R;
  }


  Eigen::Vector3d rpyFromRot(const Eigen::Matrix3d& R)
  {
    double roll, pitch, yaw;

    if((std::fabs(R(0,0)) < std::fabs(R(2,0))) && (std::fabs(R(1,0)) < std::fabs(R(2,0)))) {
      // std::cos(p) is nearly = 0
      double sp = -R(2,0);
      if (sp < -1.0) {
        sp = -1.0;
      } else if (sp > 1.0) {
        sp = 1.0;
      }
      pitch = std::asin(sp); // -pi/2< p < pi/2

      roll = std::atan2(sp * R(0,1) + R(1,2),  // -cp*cp*sr*cy
                        sp * R(0,2) - R(1,1)); // -cp*cp*cr*cy

      if (R(0,0) > 0.0) { // cy > 0
        (roll < 0.0) ? (roll += PI) : (roll -= PI);
      }
      const double sr = std::sin(roll);
      const double cr = std::cos(roll);
      if(sp > 0.0){
        yaw = std::atan2(sr * R(1,1) + cr * R(1,2), //sy*sp
                         sr * R(0,1) + cr * R(0,2));//cy*sp
      } else {
        yaw = std::atan2(-sr * R(1,1) - cr * R(1,2),
                         -sr * R(0,1) - cr * R(0,2));
      }
    } else {
      yaw = std::atan2(R(1,0), R(0,0));
      const double sa = std::sin(yaw);
      const double ca = std::cos(yaw);
      pitch = std::atan2(-R(2,0), ca * R(0,0) + sa * R(1,0));
      roll = std::atan2(sa * R(0,2) - ca * R(1,2), -sa * R(0,1) + ca * R(1,1));
    }
    return Eigen::Vector3d(roll, pitch, yaw);
  }
}
#endif // MathUtil_H
