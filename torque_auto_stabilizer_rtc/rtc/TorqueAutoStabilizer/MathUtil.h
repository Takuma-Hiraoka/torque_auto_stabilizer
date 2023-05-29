#ifndef MathUtil_H
#define MathUtil_H
#include <cmath>
#include <string>
#include "pinocchio/spatial/se3.hpp"

namespace mathutil
{
  // copy from choreonoid
  Eigen::Matrix3d rotFromRpy(double r, double p, double y);

  Eigen::Vector3d rpyFromRot(const Eigen::Matrix3d& R);

  Eigen::Matrix3d orientCoordToAxis(const Eigen::Matrix3d& m, const Eigen::Vector3d& axis, const Eigen::Vector3d& localaxis = Eigen::Vector3d::UnitZ());
  pinocchio::SE3 orientCoordToAxis(const pinocchio::SE3& m, const Eigen::Vector3d& axis, const Eigen::Vector3d& localaxis = Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd slerp(const Eigen::AngleAxisd& M1, const Eigen::AngleAxisd& M2, double r);

  // coordsとweightsのサイズは同じでなければならない
  Eigen::Matrix3d calcMidRot(const std::vector<Eigen::Matrix3d>& coords, const std::vector<double>& weights);

  pinocchio::SE3 calcMidCoords(const std::vector<pinocchio::SE3>& coords, const std::vector<double>& weights);

  template<typename T>
  inline T clamp(const T& value, const T& limit_value) {
    return std::max(-limit_value, std::min(limit_value, value));
  }
  template<typename T>
  inline T clamp(const T& value, const T& llimit_value, const T& ulimit_value) {
    return std::max(llimit_value, std::min(ulimit_value, value));
  }
  template<typename Derived>
  inline typename Derived::PlainObject clampMatrix(const Eigen::MatrixBase<Derived>& value, const Eigen::MatrixBase<Derived>& limit_value) {
    return value.array().max(-limit_value.array()).min(limit_value.array());
  }
  template<typename Derived>
  inline typename Derived::PlainObject clampMatrix(const Eigen::MatrixBase<Derived>& value, const Eigen::MatrixBase<Derived>& llimit_value, const Eigen::MatrixBase<Derived>& ulimit_value) {
    return value.array().max(llimit_value.array()).min(ulimit_value.array());
  }

  Eigen::Matrix3d cross(const Eigen::Vector3d& m);

  // Z成分は無視する
  bool isIntersect (Eigen::Vector3d& r, const Eigen::Vector3d& a0, const Eigen::Vector3d& a1, const Eigen::Vector3d& b0, const Eigen::Vector3d& b1);
  // Z成分は無視する.(0が入る)
  std::vector<Eigen::Vector3d> calcConvexHull(const std::vector<Eigen::Vector3d>& vertices);
  // Z成分は無視する. P, Qは半時計回りの凸包. (P,QのZ成分が0なら、RのZ成分にも0が入る)
  std::vector<Eigen::Vector3d> calcIntersectConvexHull(const std::vector<Eigen::Vector3d>& P, const std::vector<Eigen::Vector3d>& Q);

  // Z成分は無視する. hullは半時計回りの凸包.
  bool isInsideHull(const Eigen::Vector3d& p, const std::vector<Eigen::Vector3d>& hull);

  // Z成分は無視する. hullは半時計回りの凸包. (返り値のZ成分はhullの値が入る)
  Eigen::Vector3d calcNearestPointOfHull(const Eigen::Vector3d& p_, const std::vector<Eigen::Vector3d>& hull);

  // Z成分は無視する. P, Qは半時計回りの凸包. (返り値のZ成分はhullの値が入る). 点同士がが最近傍の場合、p,qのサイズは1になる. 線分同士が最近傍の場合、p,qのサイズが2になり線分の端点が入る.  PQが重なっている場合の挙動は、定義されない
  double calcNearestPointOfTwoHull(const std::vector<Eigen::Vector3d>& P, const std::vector<Eigen::Vector3d>& Q, std::vector<Eigen::Vector3d>& p, std::vector<Eigen::Vector3d>& q);

  // originから見て、pがverticesの内部に入るようにする. pの高さのXY平面で考える
  Eigen::Vector3d calcInsidePointOfPolygon3D(const Eigen::Vector3d& p, const std::vector<Eigen::Vector3d>& vertices, const Eigen::Vector3d& origin);

  // dir方向に最も遠いverticesと、その距離を返す. dirのノルムは1.
  double findExtreams(const std::vector<Eigen::Vector3d>& vertices, const Eigen::Vector3d& dir, std::vector<Eigen::Vector3d>& ret);

  // copy from https://github.com/Naoki-Hiraoka/cpp_filters/blob/master/include/cpp_filters/FirstOrderLowPassFilter.h
  template <class T> class FirstOrderLowPassFilter
  {
  private:
    T prev_value;
    double cutoff_freq, dt, const_param;
  public:
    FirstOrderLowPassFilter (const double _cutoff_freq, const T init_value) : FirstOrderLowPassFilter(_cutoff_freq, 1.0, init_value)
    {
    };
    FirstOrderLowPassFilter (const double _cutoff_freq, const double _dt, const T init_value) : prev_value(init_value), dt(_dt)
    {
      setCutOffFreq(_cutoff_freq);
    };
    T passFilter (const T& value, const double _dt)
    {
      if ( _dt != dt ){
        dt = _dt;
        setCutOffFreq(cutoff_freq);
      }
      prev_value = 1.0/(1+const_param) * prev_value + const_param/(1+const_param) * value;
      return prev_value;
    };
    T passFilter (const T& value)
    {
      prev_value = 1.0/(1+const_param) * prev_value + const_param/(1+const_param) * value;
      return prev_value;
    };
    void reset (const T& value) { prev_value = value; };
    void setCutOffFreq (const double f)
    {
      cutoff_freq = f;
      const_param = 2 * M_PI * cutoff_freq * dt;
    };
    double getCutOffFreq () const { return cutoff_freq; };
    T value () const { return prev_value; };
  };

  typedef enum {LINEAR, HOFFARBIB,QUINTICSPLINE,CUBICSPLINE} interpolation_mode;

  template<typename T1, typename T2> class TwoPointInterpolatorBase
  {
    // interpolator class is to interpolate from current value to goal value considering position, velocities, and accelerations.
    //   Two status : empty or not
    //                Interpolator interpolates based on remaining time (remain_t) and pushes value to queue (q, dq, ddq).
    //                Users can get interpolated results from queue (q, dq, ddq).
    //                If remain_t <= 0 and queue is empty, interpolator is "empty", otherwise "not empty".
    //                This is related with isEmpty() function.
    //   Setting goal value : setGoal(), go(), and load()
    //   Getting current value : get()
    //   Resetting current value : set()
    //   Interpolate : interpolate()
  public:
    TwoPointInterpolatorBase(const T1& init_x, const T2& init_v, const T2& init_a, interpolation_mode imode=HOFFARBIB) :
      imode_(imode)
    {
      this->reset(init_x,init_v,init_a);
    }
    void interpolate(double dt){
      if(dt == 0.0 || current_time_ == goal_time_) return;
      current_time_ += dt;
      if (current_time_ < 0.0) current_time_ = 0.0;
      if (current_time_ > goal_time_) current_time_ = goal_time_;
      this->getImpl(currentx_,currentv_,currenta_,current_time_);
    }
    // Getter function.
    T1 value() const {
      return currentx_;
    }
    void value(T1& x) const {
      x = currentx_;
    }
    void value(T1& x, T2& v) const {
      x = currentx_;
      v = currentv_;
    }
    void value(T1& x, T2& v, T2& a) const {
      x = currentx_;
      v = currentv_;
      a = currenta_;
    }
    void get(T1& x, double dt=0.0) { // deprecated
      T2 v, a;
      get(x, v, a, dt);
    }
    void get(T1& x, T2& v, double dt=0.0) { // deprecated
      T2 a;
      get(x, v, a, dt);
    }
    void get(T1& x, T2& v, T2& a, double dt=0.0) { // deprecated
      interpolate(dt);
      value(x,v,a);
    }
    // Reset current value.
    void reset(const T1& x) {
      this->reset(x,this->a0_*0,this->a0_*0);
    }
    void reset(const T1& x, const T2& v) {
      this->reset(x,v,this->a0_*0);
    }
    void reset(const T1& x, const T2& v, const T2& a)
    {
      this->goal_time_ = 0.0;
      this->current_time_ = 0.0;
      this->startx_ = x;
      this->startv_ = v;
      this->starta_ = a;
      this->currentx_ = x;
      this->currentv_ = v;
      this->currenta_ = a;
      this->goalx_ = x;
      this->goalv_ = v;
      this->goala_ = a;
      this->a0_= v * 0.0;
      this->a1_= v * 0.0;
      this->a2_= v * 0.0;
      this->a3_= v * 0.0;
      this->a4_= v * 0.0;
      this->a5_= v * 0.0;
      this->resetImpl(x,v,a);
    }
    // Stop to current value
    void clear() {
      T1 x;
      T2 v, a;
      this->get(x,v,a,0.0);
      this->reset(x,v*0,a*0);
    }
    bool isEmpty() const {
      return current_time_ == goal_time_;
    }
    double remain_time() const {
      return goal_time_ - current_time_;
    }
    double current_time() const {
      return current_time_;
    }
    double goal_time() const{
      return goal_time_;
    }
    bool setInterpolationMode (interpolation_mode i_mode){
      if (i_mode != LINEAR && i_mode != HOFFARBIB &&
          i_mode != QUINTICSPLINE && i_mode != CUBICSPLINE) return false;
      imode_ = i_mode;
      return true;
    };
    // Set goal
    void setGoal(const T1& goalx, double t) {
      this->setGoal(goalx,this->a0_*0,this->a0_*0,t);
    }
    void setGoal(const T1& goalx, const T2& goalv, double t) {
      this->setGoal(goalx,goalv,this->a0_*0,t);
    }
    void setGoal(const T1& goalx, const T2& goalv, const T2& goala, double t) {
      if(t == 0.0) {
        this->reset(goalx,goalv,goala);
        return;
      }

      T1 x;
      T2 v, a;
      this->get(x,v,a,0.0);
      this->reset(x,v,a);
      this->goal_time_ = t;
      this->goalx_ = goalx;
      this->goalv_ = goalv;
      this->goala_ = goala;

      this->setGoalImpl(x,v,a,goalx,goalv,goala,t);
    }
    T1 getGoal() const {
      return goalx_;
    }
    void getGoal(T1& x) const {
      x = goalx_;
    }
    void getGoal(T1& x, T2& v) const {
      x = goalx_;
      v = goalv_;
    }
    void getGoal(T1& x, T2& v, T2& a) const { // current_time = goal_time時のvalue()値と、getGoal()値は、数値誤差によって僅かに異なることに注意
      x = goalx_;
      v = goalv_;
      a = goala_;
    }
    std::string& name() { return name_; };
    const std::string& name() const { return name_; };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  protected:
    void getImpl(T1& x, T2& v, T2& a, double t);
    void resetImpl(const T1& x, const T2& v, const T2& a){}
    void setGoalImpl(const T1& startx, const T2& startv, const T2& starta, const T1& goalx, const T2& goalv, const T2& goala, double t);
    void calcCoeff(const T2& startx, const T2& startv, const T2& starta, const T2& goalx, const T2& goalv, const T2& goala, double t){
      T2 A,B,C;
      switch(imode_){
      case LINEAR:
        a0_=startx;
        a1_=(goalx-startx)/t;
        a2_*=0;
        a3_*=0;
        a4_*=0;
        a5_*=0;
        break;
      case HOFFARBIB:
        A=(goalx-(startx+startv*t+(starta/2.0)*t*t))/(t*t*t);
        B=(goalv-(startv+starta*t))/(t*t);
        C=(goala-starta)/t;

        a0_=startx;
        a1_=startv;
        a2_=starta/2.0;
        a3_=10*A-4*B+0.5*C;
        a4_=(-15*A+7*B-C)/t;
        a5_=(6*A-3*B+0.5*C)/(t*t);
        break;
      case QUINTICSPLINE:
        a0_=startx;
        a1_=startv;
        a2_=0.5*starta;
        a3_=(-20*startx + 20*goalx - 3*starta*t*t + goala*t*t -
             12*startv*t - 8*goalv*t) / (2*t*t*t);
        a4_=(30*startx - 30*goalx + 3*starta*t*t - 2*goala*t*t +
             16*startv*t + 14*goalv*t) / (2*t*t*t*t);
        a5_=(-12*startx + 12*goalx - starta*t*t + goala*t*t -
             6*startv*t - 6*goalv*t) / (2*t*t*t*t*t);
        break;
      case CUBICSPLINE:
        a0_=startx;
        a1_=startv;
        a2_=(-3*startx + 3*goalx - 2*startv*t - goalv*t) / (t*t);
        a3_=( 2*startx - 2*goalx + startv*t + goalv*t) / (t*t*t);
        a4_*=0;
        a5_*=0;
        break;
      }
    }
    void calcPolynomial(T2& x, T2& v, T2& a, double t){
      x=a0_+a1_*t+a2_*t*t+a3_*t*t*t+a4_*t*t*t*t+a5_*t*t*t*t*t;
      v=a1_+2*a2_*t+3*a3_*t*t+4*a4_*t*t*t+5*a5_*t*t*t*t;
      a=2*a2_+6*a3_*t+12*a4_*t*t+20*a5_*t*t*t;
    }
    // Current interpolation mode
    interpolation_mode imode_;
    double goal_time_;
    double current_time_;
    // Coefficients for interpolation polynomials.
    T2 a0_, a1_, a2_, a3_, a4_, a5_;
    T1 startx_;
    T2 startv_, starta_;
    T1 currentx_;
    T2 currentv_, currenta_;
    T1 goalx_;
    T2 goalv_, goala_;
    // Interpolator name
    std::string name_;
  };


  // for Euclid
  template<typename T1, typename T2>
  void TwoPointInterpolatorBase<T1,T2>::getImpl(T1& x, T2& v, T2& a, double t) {
    this->calcPolynomial(x,v,a,t);
  }
  template<typename T1, typename T2>
  void TwoPointInterpolatorBase<T1,T2>::setGoalImpl(const T1& startx, const T2& startv, const T2& starta, const T1& goalx, const T2& goalv, const T2& goala, double t) {
    this->calcCoeff(startx, startv, starta, goalx, goalv, goala, t);
  }
  template<typename T> using TwoPointInterpolator = TwoPointInterpolatorBase<T,T>;


  // for SO3. v and a are represented in local frame.
  template<>
  void TwoPointInterpolatorBase<Eigen::Matrix3d,Eigen::Vector3d>::getImpl(Eigen::Matrix3d& x, Eigen::Vector3d& v, Eigen::Vector3d& a, double t);
  template<>
  void TwoPointInterpolatorBase<Eigen::Matrix3d,Eigen::Vector3d>::setGoalImpl(const Eigen::Matrix3d& startx, const Eigen::Vector3d& startv, const Eigen::Vector3d& starta, const Eigen::Matrix3d& goalx, const Eigen::Vector3d& goalv, const Eigen::Vector3d& goala, double t);
  using TwoPointInterpolatorSO3 = TwoPointInterpolatorBase<Eigen::Matrix3d,Eigen::Vector3d>;

  // for Eigen::Transform<double, 3, Eigen::AffineCompact>
  class TwoPointInterpolatorSE3 {
    using Position = pinocchio::SE3;
  public:
    TwoPointInterpolatorSE3(const Position& init_x, const Eigen::Matrix<double, 6, 1>& init_v, const Eigen::Matrix<double, 6, 1>& init_a, interpolation_mode imode=HOFFARBIB) :
      p(init_x.translation(),init_v.head<3>(), init_a.head<3>(), imode),
      R(init_x.rotation(),init_v.tail<3>(), init_a.tail<3>(), imode) {}
    void interpolate(double dt){
      p.interpolate(dt);
      R.interpolate(dt);
    }
    Position value() const {
      Position ret;
      ret.translation() = p.value();
      ret.rotation() = R.value();
      return ret;
    }
    void value(Position& x) const {
      Eigen::Vector3d p_x;
      Eigen::Matrix3d R_x;
      p.value(p_x);
      R.value(R_x);
      x.translation() = p_x;
      x.rotation() = R_x;
    }
    void value(Position& x, Eigen::Matrix<double, 6, 1>& v) const {
      Eigen::Vector3d p_x, p_v, R_v;
      Eigen::Matrix3d R_x;
      p.value(p_x,p_v);
      R.value(R_x,R_v);
      x.translation() = p_x;
      v.head<3>() = p_v;
      x.rotation() = R_x;
      v.tail<3>() = R_v;
    }
    void value(Position& x, Eigen::Matrix<double, 6, 1>& v, Eigen::Matrix<double, 6, 1>& a) const {
      Eigen::Vector3d p_x, p_v, p_a, R_v, R_a;
      Eigen::Matrix3d R_x;
      p.value(p_x,p_v,p_a);
      R.value(R_x,R_v,R_a);
      x.translation() = p_x;
      v.head<3>() = p_v;
      a.head<3>() = p_a;
      x.rotation() = R_x;
      v.tail<3>() = R_v;
      a.tail<3>() = R_a;
    }
    // Reset current value.
    void reset(const Position& x) {
      p.reset(x.translation());
      R.reset(x.rotation());
    }
    void reset(const Position& x, const Eigen::Matrix<double, 6, 1>& v) {
      p.reset(x.translation(),v.head<3>());
      R.reset(x.rotation(),v.tail<3>());
    }
    void reset(const Position& x, const Eigen::Matrix<double, 6, 1>& v, const Eigen::Matrix<double, 6, 1>& a) {
      p.reset(x.translation(),v.head<3>(),a.head<3>());
      R.reset(x.rotation(),v.tail<3>(),a.tail<3>());
    }
    void clear() {
      p.clear();
      R.clear();
    }
    bool isEmpty() const {
      return p.isEmpty();
    }
    double remain_time() const {
      return p.remain_time();
    }
    bool setInterpolationMode (interpolation_mode i_mode){
      return p.setInterpolationMode(i_mode) && R.setInterpolationMode(i_mode);
    };
    void setGoal(const Position& goalx, double t) {
      p.setGoal(goalx.translation(),t);
      R.setGoal(goalx.rotation(),t);
    }
    void setGoal(const Position& goalx, const Eigen::Matrix<double, 6, 1>& goalv, double t) {
      p.setGoal(goalx.translation(),goalv.head<3>(),t);
      R.setGoal(goalx.rotation(),goalv.tail<3>(),t);
    }
    void setGoal(const Position& goalx, const Eigen::Matrix<double, 6, 1>& goalv, const Eigen::Matrix<double, 6, 1>& goala, double t) {
      p.setGoal(goalx.translation(),goalv.head<3>(),goala.head<3>(),t);
      R.setGoal(goalx.rotation(),goalv.tail<3>(),goala.tail<3>(),t);
    }
    Position getGoal() const {
      Position ret;
      ret.translation() = p.getGoal();
      ret.rotation() = R.getGoal();
      return ret;
    }
    void getGoal(Position& x) const {
      Eigen::Vector3d p_x;
      Eigen::Matrix3d R_x;
      p.getGoal(p_x);
      R.getGoal(R_x);
      x.translation() = p_x;
      x.rotation() = R_x;
    }
    void getGoal(Position& x, Eigen::Matrix<double, 6, 1>& v) const {
      Eigen::Vector3d p_x, p_v, R_v;
      Eigen::Matrix3d R_x;
      p.getGoal(p_x,p_v);
      R.getGoal(R_x,R_v);
      x.translation() = p_x;
      v.head<3>() = p_v;
      x.rotation() = R_x;
      v.tail<3>() = R_v;
    }
    void getGoal(Position& x, Eigen::Matrix<double, 6, 1>& v, Eigen::Matrix<double, 6, 1>& a) const {
      Eigen::Vector3d p_x, p_v, p_a, R_v, R_a;
      Eigen::Matrix3d R_x;
      p.getGoal(p_x,p_v,p_a);
      R.getGoal(R_x,R_v,R_a);
      x.translation() = p_x;
      v.head<3>() = p_v;
      a.head<3>() = p_a;
      x.rotation() = R_x;
      v.tail<3>() = R_v;
      a.tail<3>() = R_a;
    }
    std::string& name() { return p.name(); };
    const std::string& name() const { return p.name(); };
  protected:
    TwoPointInterpolator<Eigen::Vector3d> p;
    TwoPointInterpolatorSO3 R;
  };
}
#endif // MathUtil_H
