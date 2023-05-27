#include "pinocchio/algorithm/kinematics.hpp"
#include "MathUtil.h"

namespace mathutil
{
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
        (roll < 0.0) ? (roll += M_PI) : (roll -= M_PI);
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

  Eigen::Matrix3d orientCoordToAxis(const Eigen::Matrix3d& m, const Eigen::Vector3d& axis, const Eigen::Vector3d& localaxis){
    // axisとlocalaxisはノルムが1, mは回転行列でなければならない.
    // axisとlocalaxisがピッタリ180反対向きの場合、回転方向が定まらないので不安定
    Eigen::AngleAxisd m_ = Eigen::AngleAxisd(m); // Eigen::Matrix3dの空間で積算していると数値誤差によってだんたん回転行列ではなくなってくるので
    Eigen::Vector3d localaxisdir = m_ * localaxis;
    Eigen::Vector3d cross = localaxisdir.cross(axis);
    double dot = std::min(1.0,std::max(-1.0,localaxisdir.dot(axis))); // acosは定義域外のときnanを返す
    if(cross.norm()==0){
      if(dot == -1) return Eigen::Matrix3d(-m);
      else return Eigen::Matrix3d(m_);
    }else{
      double angle = std::acos(dot); // 0~pi
      Eigen::Vector3d axis = cross.normalized(); // include sign
      return Eigen::Matrix3d(Eigen::AngleAxisd(angle, axis) * m_);
    }
  }
  Eigen::Transform<double, 3, Eigen::AffineCompact> orientCoordToAxis(const Eigen::Transform<double, 3, Eigen::AffineCompact>& m, const Eigen::Vector3d& axis, const Eigen::Vector3d& localaxis){
    Eigen::Transform<double, 3, Eigen::AffineCompact> ret = m;
    ret.linear() = mathutil::orientCoordToAxis(ret.linear(), axis, localaxis);
    return ret;
  }

  Eigen::AngleAxisd slerp(const Eigen::AngleAxisd& M1, const Eigen::AngleAxisd& M2, double r){
    // 0 <= r <= 1
    Eigen::AngleAxisd trans = Eigen::AngleAxisd(M1.inverse() * M2);
    return Eigen::AngleAxisd(M1 * Eigen::AngleAxisd(trans.angle() * r, trans.axis()));
  }

  Eigen::Transform<double, 3, Eigen::AffineCompact> calcMidCoords(const std::vector<Eigen::Transform<double, 3, Eigen::AffineCompact>>& coords, const std::vector<double>& weights){
    // coordsとweightsのサイズは同じでなければならない
    double sumWeight = 0.0;
    Eigen::Transform<double, 3, Eigen::AffineCompact> midCoords = Eigen::Transform<double, 3, Eigen::AffineCompact>::Identity();

    for(int i=0;i<coords.size();i++){
      if(weights[i]<=0) continue;
      midCoords.translation() = ((midCoords.translation()*sumWeight + coords[i].translation()*weights[i])/(sumWeight+weights[i])).eval();
      midCoords.linear() = mathutil::slerp(Eigen::AngleAxisd(midCoords.linear()), Eigen::AngleAxisd(coords[i].linear()),(weights[i]/(sumWeight+weights[i]))).toRotationMatrix();
      //midCoords.linear() = Eigen::Quaterniond(midCoords.linear()).slerp(weights[i]/(sumWeight+weights[i]),Eigen::Quaterniond(coords[i].linear())).toRotationMatrix(); // quaternionのslerpは、90度回転した姿勢で不自然な遠回り補間をするので使ってはならない
      sumWeight += weights[i];
    }
    return midCoords;
  }
}
