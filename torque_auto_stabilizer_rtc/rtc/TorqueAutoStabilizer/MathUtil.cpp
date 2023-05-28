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
  pinocchio::SE3 orientCoordToAxis(const pinocchio::SE3& m, const Eigen::Vector3d& axis, const Eigen::Vector3d& localaxis){
    pinocchio::SE3 ret = m;
    ret.rotation() = mathutil::orientCoordToAxis(ret.rotation(), axis, localaxis);
    return ret;
  }

  Eigen::AngleAxisd slerp(const Eigen::AngleAxisd& M1, const Eigen::AngleAxisd& M2, double r){
    // 0 <= r <= 1
    Eigen::AngleAxisd trans = Eigen::AngleAxisd(M1.inverse() * M2);
    return Eigen::AngleAxisd(M1 * Eigen::AngleAxisd(trans.angle() * r, trans.axis()));
  }

  pinocchio::SE3 calcMidCoords(const std::vector<pinocchio::SE3>& coords, const std::vector<double>& weights){
    // coordsとweightsのサイズは同じでなければならない
    double sumWeight = 0.0;
    pinocchio::SE3 midCoords;

    for(int i=0;i<coords.size();i++){
      if(weights[i]<=0) continue;
      midCoords.translation() = ((midCoords.translation()*sumWeight + coords[i].translation()*weights[i])/(sumWeight+weights[i])).eval();
      midCoords.rotation() = mathutil::slerp(Eigen::AngleAxisd(midCoords.rotation()), Eigen::AngleAxisd(coords[i].rotation()),(weights[i]/(sumWeight+weights[i]))).toRotationMatrix();
      //midCoords.rotation() = Eigen::Quaterniond(midCoords.rotation()).slerp(weights[i]/(sumWeight+weights[i]),Eigen::Quaterniond(coords[i].rotation())).toRotationMatrix(); // quaternionのslerpは、90度回転した姿勢で不自然な遠回り補間をするので使ってはならない
      sumWeight += weights[i];
    }
    return midCoords;
  }

  inline Eigen::Matrix3d hat(const Eigen::Vector3d& x) {
    Eigen::Matrix3d M;
    M <<  0.0, -x(2),   x(1),
      x(2),   0.0,  -x(0),
      -x(1),  x(0),   0.0;
    return M;
  }

  // Smooth Attitude Interpolation
  // https://github.com/scipy/scipy/files/2932755/attitude_interpolation.pdf
  template<>
  void TwoPointInterpolatorBase<Eigen::Matrix3d,Eigen::Vector3d>::getImpl(Eigen::Matrix3d& x, Eigen::Vector3d& v, Eigen::Vector3d& a, double t) {
    Eigen::Vector3d theta, dtheta, ddtheta;
    this->calcPolynomial(theta,dtheta,ddtheta,t);
    double th = theta.norm();
    Eigen::Matrix3d thetaX = hat(theta);

    Eigen::Matrix3d Ainv = Eigen::Matrix3d::Identity();
    if(th>1e-10){// 0除算がダメなのは勿論だが、小さすぎてもオーバーフローする恐れ
      Ainv =
        Eigen::Matrix3d::Identity()
        - (1-std::cos(th))/std::pow(th,2) * thetaX
        + (th - std::sin(th))/std::pow(th,3) * thetaX * thetaX;
    }
    Eigen::Vector3d dAinv_dtheta = Eigen::Vector3d::Zero();
    if(th>1e-10){// 0除算がダメなのは勿論だが、小さすぎてもオーバーフローする恐れ
      dAinv_dtheta =
        - (th*std::sin(th)+2*(std::cos(th)-1))/std::pow(th,4) * theta.dot(dtheta) * theta.cross(dtheta)
        - (2*th+th*std::cos(th)-3*std::sin(th))/std::pow(th,5) * theta.dot(dtheta) * theta.cross(theta.cross(dtheta))
        + (th-std::sin(th))/std::pow(th,3) * dtheta.cross(theta.cross(dtheta));
    }

    // 単純に3x3行列の空間でRを積算していると、だんだん数値誤差によって回転行列でなくなってしまう
    if(th>1e-10) x = Eigen::Matrix3d(Eigen::AngleAxisd(this->startx_) * Eigen::AngleAxisd(th,theta.normalized()));
    else x = this->startx_;
    v = Ainv * dtheta;
    a = Ainv * ddtheta + dAinv_dtheta;
  }

  template<>
  void TwoPointInterpolatorBase<Eigen::Matrix3d,Eigen::Vector3d>::setGoalImpl(const Eigen::Matrix3d& startx, const Eigen::Vector3d& startv, const Eigen::Vector3d& starta, const Eigen::Matrix3d& goalx, const Eigen::Vector3d& goalv, const Eigen::Vector3d& goala, double t) {
    Eigen::Vector3d starttheta = Eigen::Vector3d::Zero();
    Eigen::AngleAxisd angleaxis(startx.transpose()*Eigen::AngleAxisd(goalx)); // 単純に3x3行列の空間でRを積算していると、だんだん数値誤差によって回転行列でなくなってしまう
    Eigen::Vector3d goaltheta = angleaxis.angle() * angleaxis.axis();
    double th = goaltheta.norm();
    Eigen::Matrix3d thetaX = hat(goaltheta);
    Eigen::Vector3d startdtheta = startv;
    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    if(th > 1e-10){ // 0除算がダメなのは勿論だが、小さすぎてもオーバーフローする恐れ
      A =
        Eigen::Matrix3d::Identity()
        + 0.5*thetaX
        + (1-th/2*std::cos(th/2)/std::sin(th/2))/std::pow(th,2) * thetaX * thetaX;
    }
    Eigen::Vector3d goaldtheta = A*goalv;
    double dth = goaldtheta.norm();
    Eigen::Matrix3d dthetaX = hat(goaldtheta);
    Eigen::Vector3d startddtheta = starta;
    Eigen::Matrix3d dA = 0.5*thetaX;
    if(th > 1e-10){ // 0除算がダメなのは勿論だが、小さすぎてもオーバーフローする恐れ
      dA =
        0.5*dthetaX
        + (dth*std::cos(th/2)/std::sin(th/2)/(2*std::pow(th,2)) + dth/(4*th*std::pow(std::sin(th/2),2)) - 2*dth/std::pow(th,3)) * thetaX * thetaX
        + (1-th/2*std::cos(th/2)/std::sin(th/2))/std::pow(th,2) * (dthetaX*thetaX + thetaX*dthetaX);
    }
    Eigen::Vector3d goalddtheta = dA * goalv + A * goala;

    this->calcCoeff(starttheta, startdtheta, startddtheta, goaltheta, goaldtheta, goalddtheta, t);
  }

}
