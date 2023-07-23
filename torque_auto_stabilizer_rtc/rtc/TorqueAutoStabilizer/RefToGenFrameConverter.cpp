#include "RefToGenFrameConverter.h"
#include "CnoidBodyUtil.h"

bool RefToGenFrameConverter::initFootCoords(const GaitParam& gaitParam, // input
                      mathutil::TwoPointInterpolatorSE3& o_footMidCoords, cnoid::BodyPtr& refRobot) const{ // output

  // ロボット内座標系を初期化する．
  // 原点にfootMidCoordsを置く．
  // footMidCoordsとrefRobotのfootOrigin座標系が一致するようにrefRobotを変換する

  refRobot->rootLink()->T() = gaitParam.refRobotRaw->rootLink()->T();
  refRobot->rootLink()->v() = gaitParam.refRobotRaw->rootLink()->v();
  refRobot->rootLink()->w() = gaitParam.refRobotRaw->rootLink()->w();
  for(int i=0;i<refRobot->numJoints();i++){
    refRobot->joint(i)->q() = gaitParam.refRobotRaw->joint(i)->q();
    refRobot->joint(i)->dq() = gaitParam.refRobotRaw->joint(i)->dq();
    refRobot->joint(i)->u() = gaitParam.refRobotRaw->joint(i)->u();
  }
  refRobot->calcForwardKinematics();
  cnoid::Position rleg = refRobot->link(gaitParam.eeParentLink[RLEG])->T()*gaitParam.eeLocalT[RLEG];
  cnoid::Position lleg = refRobot->link(gaitParam.eeParentLink[LLEG])->T()*gaitParam.eeLocalT[LLEG];
  cnoid::Position refFootMidCoords = this->calcRefFootMidCoords(rleg, lleg, gaitParam);
  cnoid::Position footMidCoords = cnoid::Position::Identity();
  cnoidbodyutil::moveCoords(refRobot, footMidCoords, refFootMidCoords);

  refRobot->calcForwardKinematics(true);
  refRobot->calcCenterOfMass();

  o_footMidCoords.reset(footMidCoords);
  return true;
}

bool RefToGenFrameConverter::convertFrame(const GaitParam& gaitParam, double dt,// input
                    cnoid::BodyPtr& refRobot, std::vector<cnoid::Position>& o_refEEPose, std::vector<cnoid::Vector6>& o_refEEWrench, double& o_refdz, mathutil::TwoPointInterpolatorSE3& o_footMidCoords) const{ // output

    /*
    次の2つの座標系が一致するようにreference frameとgenerate frameを対応付ける
    - refRobotRawの、refFootOriginWeightとdefaultTranslatePosとcopOffset.value()に基づいて求めた足裏中間座標 (イメージとしては静止状態の目標ZMP位置にdefaultTranslatePosを作用させたもの)
    - 位置XYはactRobotの重心位置. 位置ZはactRobotの重心位置 - dz. 姿勢はfootMidCoords.
   */

  // 現在のFootStepNodesListから、footMidCoordsを求める
  mathutil::TwoPointInterpolatorSE3 footMidCoords = gaitParam.footMidCoords; //generate frame. gaitParam.footMidCoords. 両足の中間
  this->calcFootMidCoords(gaitParam, dt, footMidCoords); // 1周期前のfootStepNodesListを使っているが、footMidCoordsは不連続に変化するものではないのでよい
  cnoid::Position genFootMidCoords;  //generate frame. 実際に対応づけに使用する
  genFootMidCoords.linear() = footMidCoords.value().linear();
  genFootMidCoords.translation() = gaitParam.actCog - gaitParam.l; // 1周期前のlを使っているtが、lは不連続に変化するものではないので良い

  // refRobotPosのrefFootMidCoordsを求めてrefRobotに変換する
  double refdz;
  std::vector<cnoid::Position> refEEPoseFK(gaitParam.eeName.size());
  this->convertRefRobotRaw(gaitParam, genFootMidCoords,
                           refRobot, refEEPoseFK, refdz);

  // refEEPoseを求める
  std::vector<cnoid::Position> refEEPose(gaitParam.eeName.size());
  for(int i=0;i<gaitParam.eeName.size();i++){
    refEEPose[i] = refEEPoseFK[i];
  }

  // refEEWrenchを計算
  std::vector<Eigen::Vector6d> refEEWrench(gaitParam.eeName.size());
  for(int i=0;i<gaitParam.eeName.size();i++){
    refEEWrench[i].head<3>() = footMidCoords.value().linear() * gaitParam.refEEWrenchOrigin[i].head<3>();
    refEEWrench[i].tail<3>() = footMidCoords.value().linear() * gaitParam.refEEWrenchOrigin[i].tail<3>();
  }

  o_refEEPose = refEEPose;
  o_refEEWrench = refEEWrench;
  o_refdz = refdz;
  o_footMidCoords = footMidCoords;

  return true;
}

// 現在のFootStepNodesListから、genRobotのfootMidCoordsを求める (gaitParam.footMidCoords)
void RefToGenFrameConverter::calcFootMidCoords(const GaitParam& gaitParam, double dt, mathutil::TwoPointInterpolatorSE3& footMidCoords) const {
  //footMidCoordsを進める
  cnoid::Position rleg = gaitParam.footStepNodesList[0].dstCoords[RLEG];
  rleg.translation() += rleg.linear() * gaitParam.copOffset[RLEG].value();
  cnoid::Position lleg = gaitParam.footStepNodesList[0].dstCoords[LLEG];
  lleg.translation() += lleg.linear() * gaitParam.copOffset[LLEG].value();
  cnoid::Position midCoords = mathutil::calcMidCoords(std::vector<cnoid::Position>{rleg, lleg}, std::vector<double>{1.0, 1.0});
  rleg = mathutil::orientCoordToAxis(rleg, cnoid::Vector3::UnitZ());
  lleg = mathutil::orientCoordToAxis(lleg, cnoid::Vector3::UnitZ());
  midCoords = mathutil::orientCoordToAxis(midCoords, cnoid::Vector3::UnitZ());
  rleg.translation() -= rleg.linear() * gaitParam.defaultTranslatePos[RLEG].value();
  lleg.translation() -= lleg.linear() * gaitParam.defaultTranslatePos[LLEG].value();
  if(gaitParam.footStepNodesList[0].isSupportPhase[RLEG] && gaitParam.footStepNodesList[0].isSupportPhase[LLEG]){ // 両足支持
    footMidCoords.setGoal(midCoords, gaitParam.footStepNodesList[0].remainTime);
  }else if(gaitParam.footStepNodesList[0].isSupportPhase[RLEG] && !gaitParam.footStepNodesList[0].isSupportPhase[LLEG]){ // 右足支持
    if(gaitParam.footStepNodesList.size() > 1 &&
       (gaitParam.footStepNodesList[1].isSupportPhase[RLEG] && gaitParam.footStepNodesList[1].isSupportPhase[LLEG]) // 次が両足支持
       ){
      cnoid::Position rleg = gaitParam.footStepNodesList[1].dstCoords[RLEG];
      rleg.translation() += rleg.linear() * gaitParam.copOffset[RLEG].value();
      cnoid::Position lleg = gaitParam.footStepNodesList[1].dstCoords[LLEG];
      lleg.translation() += lleg.linear() * gaitParam.copOffset[LLEG].value();
      cnoid::Position midCoords = mathutil::calcMidCoords(std::vector<cnoid::Position>{rleg, lleg}, std::vector<double>{1.0, 1.0});
      midCoords = mathutil::orientCoordToAxis(midCoords, cnoid::Vector3::UnitZ());
      footMidCoords.setGoal(midCoords, gaitParam.footStepNodesList[0].remainTime + gaitParam.footStepNodesList[1].remainTime);
    }else{
      footMidCoords.setGoal(rleg, gaitParam.footStepNodesList[0].remainTime);
    }
  }else if(!gaitParam.footStepNodesList[0].isSupportPhase[RLEG] && gaitParam.footStepNodesList[0].isSupportPhase[LLEG]){ // 左足支持
    if(gaitParam.footStepNodesList.size() > 1 &&
       (gaitParam.footStepNodesList[1].isSupportPhase[RLEG] && gaitParam.footStepNodesList[1].isSupportPhase[LLEG]) // 次が両足支持
       ){
      cnoid::Position rleg = gaitParam.footStepNodesList[1].dstCoords[RLEG];
      rleg.translation() += rleg.linear() * gaitParam.copOffset[RLEG].value();
      cnoid::Position lleg = gaitParam.footStepNodesList[1].dstCoords[LLEG];
      lleg.translation() += lleg.linear() * gaitParam.copOffset[LLEG].value();
      cnoid::Position midCoords = mathutil::calcMidCoords(std::vector<cnoid::Position>{rleg, lleg}, std::vector<double>{1.0, 1.0});
      midCoords = mathutil::orientCoordToAxis(midCoords, cnoid::Vector3::UnitZ());
      footMidCoords.setGoal(midCoords, gaitParam.footStepNodesList[0].remainTime + gaitParam.footStepNodesList[1].remainTime);
    }else{
      footMidCoords.setGoal(lleg, gaitParam.footStepNodesList[0].remainTime);
    }
  }else{ // 滞空期 TODO
    if(gaitParam.footStepNodesList[1].isSupportPhase[RLEG] && !gaitParam.footStepNodesList[1].isSupportPhase[LLEG]) { // 次が右足支持
      footMidCoords.setGoal(rleg, gaitParam.footStepNodesList[0].remainTime);
    }else if(!gaitParam.footStepNodesList[1].isSupportPhase[RLEG] && gaitParam.footStepNodesList[1].isSupportPhase[LLEG]) { // 次が左足支持
      footMidCoords.setGoal(lleg, gaitParam.footStepNodesList[0].remainTime);
    }else{
      footMidCoords.setGoal(midCoords, gaitParam.footStepNodesList[0].remainTime);
    }
  }

  footMidCoords.interpolate(dt);
}

// refRobotRawをrefRobotに変換する.
void RefToGenFrameConverter::convertRefRobotRaw(const GaitParam& gaitParam, const cnoid::Position& genFootMidCoords, cnoid::BodyPtr& refRobot, std::vector<cnoid::Position>& refEEPoseFK, double& refdz) const{
  cnoidbodyutil::copyRobotState(gaitParam.refRobotRaw, refRobot);
  cnoid::Position rleg = refRobot->link(gaitParam.eeParentLink[RLEG])->T()*gaitParam.eeLocalT[RLEG];
  cnoid::Position lleg = refRobot->link(gaitParam.eeParentLink[LLEG])->T()*gaitParam.eeLocalT[LLEG];
  cnoid::Position refFootMidCoords = this->calcRefFootMidCoords(rleg, lleg, gaitParam);
  refdz = (refFootMidCoords.inverse() * refRobot->centerOfMass())[2]; // ref重心高さ

  cnoidbodyutil::moveCoords(refRobot, genFootMidCoords, refFootMidCoords);
  refRobot->calcForwardKinematics();
  refRobot->calcCenterOfMass();

  for(int i=0;i<gaitParam.eeName.size();i++){
    refEEPoseFK[i] = refRobot->link(gaitParam.eeParentLink[i])->T() * gaitParam.eeLocalT[i];
  }
}

cnoid::Position RefToGenFrameConverter::calcRefFootMidCoords(const cnoid::Position& rleg_, const cnoid::Position& lleg_, const GaitParam& gaitParam) const {
  cnoid::Position rleg = rleg_;
  rleg.translation() += rleg.linear() * gaitParam.copOffset[RLEG].value();
  cnoid::Position lleg = lleg_;
  lleg.translation() += lleg.linear() * gaitParam.copOffset[LLEG].value();

  cnoid::Position bothmidcoords = mathutil::calcMidCoords(std::vector<cnoid::Position>{rleg, lleg},
                                                          std::vector<double>{1.0, 1.0});
  cnoid::Position rlegmidcoords = rleg;
  rlegmidcoords.translation() -= rlegmidcoords.linear() * gaitParam.defaultTranslatePos[RLEG].value();
  cnoid::Position llegmidcoords = lleg;
  llegmidcoords.translation() -= llegmidcoords.linear() * gaitParam.defaultTranslatePos[LLEG].value();

  double bothweight = std::min(this->refFootOriginWeight[RLEG].value(), this->refFootOriginWeight[LLEG].value());
  double rlegweight = this->refFootOriginWeight[RLEG].value() - bothweight;
  double llegweight = this->refFootOriginWeight[LLEG].value() - bothweight;
  return mathutil::calcMidCoords(std::vector<cnoid::Position>{bothmidcoords, rlegmidcoords, llegmidcoords},
                                 std::vector<double>{bothweight, rlegweight, llegweight});
}
