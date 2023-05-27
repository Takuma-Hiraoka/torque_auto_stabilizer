#ifndef MathUtil_H
#define MathUtil_H
#include <cmath>

namespace mathutil
{
  // copy from choreonoid
  Eigen::Matrix3d rotFromRpy(double r, double p, double y);

  Eigen::Vector3d rpyFromRot(const Eigen::Matrix3d& R);

  Eigen::Matrix3d orientCoordToAxis(const Eigen::Matrix3d& m, const Eigen::Vector3d& axis, const Eigen::Vector3d& localaxis = Eigen::Vector3d::UnitZ());
  Eigen::Transform<double, 3, Eigen::AffineCompact> orientCoordToAxis(const Eigen::Transform<double, 3, Eigen::AffineCompact>& m, const Eigen::Vector3d& axis, const Eigen::Vector3d& localaxis = Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd slerp(const Eigen::AngleAxisd& M1, const Eigen::AngleAxisd& M2, double r);

  Eigen::Transform<double, 3, Eigen::AffineCompact> calcMidCoords(const std::vector<Eigen::Transform<double, 3, Eigen::AffineCompact>>& coords, const std::vector<double>& weights);

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
}
#endif // MathUtil_H
