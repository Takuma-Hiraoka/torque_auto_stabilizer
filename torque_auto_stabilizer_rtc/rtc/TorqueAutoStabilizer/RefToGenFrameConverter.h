#ifndef REFTOGENFRAMECONVERTER_H
#define REFTOGENFRAMECONVERTER_H

#include "GaitParam.h"

class RefToGenFrameConverter {
public:
  // RefToGenFrameConverterだけでつかうパラメータ
  std::vector<mathutil::TwoPointInterpolator<double> > refFootOriginWeight = std::vector<mathutil::TwoPointInterpolator<double> >(NUM_LEGS,mathutil::TwoPointInterpolator<double>(1.0,0.0,0.0,mathutil::HOFFARBIB)); // 要素数2. 0: rleg. 1: lleg. 0~1. Reference座標系のfootOriginを計算するときに用いるweight. このfootOriginからの相対位置で、GaitGeneratorに管理されていないEndEffectorのReference位置が解釈される. interpolatorによって連続的に変化する. 全てのLegのrefFootOriginWeightが同時に0になることはない.
  mathutil::TwoPointInterpolator<double> solveFKMode = mathutil::TwoPointInterpolator<double>(1.0,0.0,0.0,mathutil::HOFFARBIB); // 0~1. 1ならエンドエフェクタ位置姿勢をrefRobotRawのFKから求める. 0なら、refEEPoseRawから求める. startAutoStabilizerした直後は必ず1

public:
  // startAutoStabilizer時に呼ばれる
  void reset() {
    for(int i=0;i<refFootOriginWeight.size();i++) refFootOriginWeight[i].reset(refFootOriginWeight[i].getGoal());
    solveFKMode.reset(1.0);
  }
  // 内部の補間器をdtだけ進める
  void update(double dt){
    for(int i=0;i<refFootOriginWeight.size();i++) refFootOriginWeight[i].interpolate(dt);
    solveFKMode.interpolate(dt);
  }
public:
  /*
    次の2つの座標系が一致するようにreference frameとgenerate frameを対応付ける
    - refRobotRawの、refFootOriginWeightとdefaultTranslatePosとcopOffset.value()に基づいて求めた足裏中間座標 (イメージとしては静止状態の目標ZMP位置にdefaultTranslatePosを作用させたもの)
    - 位置XYはactRobotの重心位置. 位置ZはactRobotの重心位置 - dz. 姿勢はfootMidCoords.
   */

  // 変換のためにactの重心位置を使うため、

  // 重心位置をもとに反映させるが、姿勢はactの足の平均を取るのではなく着地した際の姿勢へ緩やかに遷移する．そのためのfootMidCoords．

  // startAutoStabilizer直後の初回に、他の処理よりも先に呼ぶ必要がある.
  // ロボット内座標系を初期化する．
  // 原点にfootMidCoordsを置く．
  // footMidCoordsとrefRobotのfootOrigin座標系が一致するようにrefRobotを変換し、その両足先をfootstepNodeのはじめにする。
  bool initFootCoords(const GaitParam& gaitParam, const pinocchio::Model& model,// input
                      mathutil::TwoPointInterpolatorSE3& o_footMidCoords, pinocchio::Data& refRobot) const; // output

  // reference frameで表現されたrefRobotPosをgenerate frameに投影しrefRobotとし、各種reference値をgenerate frameに変換する
  bool convertFrame(const GaitParam& gaitParam, const pinocchio::Model& model, double dt, pinocchio::Data& actRobot, // input
                    pinocchio::Data& refRobot, std::vector<pinocchio::SE3>& o_refEEPose, std::vector<Eigen::Vector6d>& o_refEEWrench, double& o_refdz, mathutil::TwoPointInterpolatorSE3& o_footMidCoords) const; // output
protected:
  // 現在のFootStepNodesListから、genRobotのfootMidCoordsを求める (gaitParam.footMidCoords)
  void calcFootMidCoords(const GaitParam& gaitParam, double dt, mathutil::TwoPointInterpolatorSE3& footMidCoords) const;
  // refRobotRawをrefRobotに変換する.
  void convertRefRobotRaw(const GaitParam& gaitParam, const pinocchio::Model& model, const pinocchio::SE3& genFootMidCoords, pinocchio::Data& refRobot, std::vector<pinocchio::SE3>& refEEPoseFK, double& refdz) const;
  // refEEPoseRawを変換する.
  void convertRefEEPoseRaw(const GaitParam& gaitParam, const pinocchio::Model& model, const pinocchio::SE3& genFootMidCoords, std::vector<pinocchio::SE3>& refEEPoseWithOutFK) const;

  // refFootOriginWeightとdefaultTranslatePosとcopOffset.value() に基づいて両足中間座標を求める
  pinocchio::SE3 calcRefFootMidCoords(const pinocchio::SE3& rleg_, const pinocchio::SE3& lleg_, const GaitParam& gaitParam) const;
};

#endif
