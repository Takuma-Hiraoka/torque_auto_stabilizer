#ifndef ACTTOGENFRAMECONVERTER_H
#define ACTTOGENFRAMECONVERTER_H

#include "GaitParam.h"
#include <cnoid/ForceSensor>


class ActToGenFrameConverter{
public:
  // ActToGenFrameConverterだけでつかうパラメータ
  Eigen::Vector3d rpyOffset = Eigen::Vector3d::Zero(); // [roll, pitch, yaw]. rootLink Frame. Actual robotのrootLinkの姿勢に加えるオフセット. IMUの取り付け位置のオフセットを考慮するためのものではない(それはモデルファイルを変えれば良い). 全身のキャリブのずれなど次第に出てくるなにかしらのずれをごますためのもの. 本来このようなパラメータは必要ないのが望ましいが、実用上は確かに必要.

  std::vector<std::string> eeForceSensor; // constant. 要素数と順序はGaitParam.eeNameと同じ. actualのForceSensorの値を座標変換したものがEndEffectorが受けている力とみなされる. eeForceSensorが""ならば受けている力は常に0とみなされる. eeForceSensorが""で無いならばrobot->findDevice<cnoid::ForceSensor>(eeForceSensor)がnullptrでは無いことを約束するので、毎回nullptrかをチェックしなくても良い

protected:
  // 内部で変更されるパラメータ. startAutoStabilizer時にリセットされる
  mutable bool isInitial = true;

public:
  // startAutoStabilizer時に呼ばれる
  void reset() {
    isInitial = true;
  }

public:
  /*
    支持脚のfootOrigin座標系が一致するように、actual frametとgenerate frameを対応付ける
   */

  // actual frameで表現されたactRobotPos/Velをgenerate frameに投影しactRobotとし、各種actual値をgenerate frameに変換する
  bool convertFrame(const GaitParam& gaitParam, double dt, // input
                    cnoid::BodyPtr& actRobot, std::vector<cnoid::Position>& o_actEEPose, std::vector<Eigen::Vector6d>& o_actEEWrench, mathutil::FirstOrderLowPassFilter<Eigen::Vector3d>& o_actCogVel, mathutil::FirstOrderLowPassFilter<cnoid::Vector6>& o_actRootVel, Eigen::Vector3d o_actCog, std::vector<bool>& prevOriginLeg) const; // output
};

#endif
