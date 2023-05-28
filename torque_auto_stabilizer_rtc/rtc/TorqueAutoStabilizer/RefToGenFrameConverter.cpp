#include "RefToGenFrameConverter.h"
#include "pinocchio/algorithm/center-of-mass.hpp"

bool RefToGenFrameConverter::initFootCoords(const GaitParam& gaitParam, const pinocchio::Model& model, // input
                    mathutil::TwoPointInterpolatorSE3& o_footMidCoords, pinocchio::Data& refRobot, std::vector<GaitParam::FootStepNodes>& o_footStepNodeList) const{ // output

  // ロボット内座標系を初期化する．
  // 原点にfootMidCoordsを置く．
  // footMidCoordsとrefRobotのfootOrigin座標系が一致するようにrefRobotを変換し、その両足先をfootstepNodeのはじめにする。

  pinocchio::forwardKinematics(model,refRobot,gaitParam.refRobotPos);
  pinocchio::SE3 footMidCoords;
  pinocchio::SE3 refrleg = refRobot.oMi[model.getJointId(gaitParam.eeParentLink[RLEG])]*gaitParam.eeLocalT[RLEG];
  pinocchio::SE3 reflleg = refRobot.oMi[model.getJointId(gaitParam.eeParentLink[LLEG])]*gaitParam.eeLocalT[LLEG];
  pinocchio::SE3 refFootMidCoords = mathutil::calcMidCoords(std::vector<pinocchio::SE3>{refrleg, reflleg},
                                                            std::vector<double>{1.0, 1.0});
  pinocchio::SE3 refFootOriginCoords = mathutil::orientCoordToAxis(refFootMidCoords, Eigen::Vector3d::UnitZ());

  pinocchio::SE3 transform = footMidCoords*refFootMidCoords.inverse();
  Eigen::Vector3d pos = gaitParam.refRobotPos.segment(0,3);
  Eigen::Quaterniond q;
  q.coeffs() = gaitParam.refRobotPos.segment(3,4);
  pinocchio::SE3 rootT = transform * pinocchio::SE3(q,pos);
  Eigen::VectorXd posForFootStepNode = gaitParam.refRobotPos;
  posForFootStepNode.segment(0,3) = rootT.translation();
  Eigen::Quaterniond qTransform(rootT.rotation());
  posForFootStepNode.segment(3,4) = qTransform.coeffs();
  pinocchio::forwardKinematics(model,refRobot,posForFootStepNode);

  std::vector<GaitParam::FootStepNodes> footStepNodesList(1);
  pinocchio::SE3 rlegCoords = refRobot.oMi[model.getJointId(gaitParam.eeParentLink[RLEG])]*gaitParam.eeLocalT[RLEG];
  pinocchio::SE3 llegCoords = refRobot.oMi[model.getJointId(gaitParam.eeParentLink[LLEG])]*gaitParam.eeLocalT[LLEG];
  footStepNodesList[0].dstCoords = {rlegCoords, llegCoords};
  footStepNodesList[0].isSupportPhase = {true,true}; //TODO
  footStepNodesList[0].remainTime = 0.0;
  if(footStepNodesList[0].isSupportPhase[RLEG] && !footStepNodesList[0].isSupportPhase[LLEG]) footStepNodesList[0].endRefZmpState = GaitParam::FootStepNodes::refZmpState_enum::RLEG;
  else if(!footStepNodesList[0].isSupportPhase[RLEG] && footStepNodesList[0].isSupportPhase[LLEG]) footStepNodesList[0].endRefZmpState = GaitParam::FootStepNodes::refZmpState_enum::LLEG;
  else footStepNodesList[0].endRefZmpState = GaitParam::FootStepNodes::refZmpState_enum::MIDDLE;

  o_footMidCoords.reset(footMidCoords);
  o_footStepNodeList = footStepNodesList;
  return true;
}

bool RefToGenFrameConverter::convertFrame(const GaitParam& gaitParam, const pinocchio::Model& model, double dt, pinocchio::Data& actRobot, // input
                                          pinocchio::Data& refRobot, std::vector<pinocchio::SE3>& o_refEEPose, std::vector<Eigen::Vector6d>& o_refEEWrench, double& o_refdz, mathutil::TwoPointInterpolatorSE3& o_footMidCoords) const{ // output

    /*
    次の2つの座標系が一致するようにreference frameとgenerate frameを対応付ける
    - refRobotRawの、refFootOriginWeightとdefaultTranslatePosとcopOffset.value()に基づいて求めた足裏中間座標 (イメージとしては静止状態の目標ZMP位置にdefaultTranslatePosを作用させたもの)
    - 位置XYはactRobotの重心位置. 位置ZはactRobotの重心位置 - dz. 姿勢はfootMidCoords.
   */

  // 現在のFootStepNodesListから、footMidCoordsを求める
  mathutil::TwoPointInterpolatorSE3 footMidCoords = gaitParam.footMidCoords; //generate frame. gaitParam.footMidCoords. 両足の中間
  this->calcFootMidCoords(gaitParam, dt, footMidCoords); // 1周期前のfootstepNodesListを使っているが、footMidCoordsは不連続に変化するものではないのでよい
  pinocchio::SE3 genFootMidCoords;  //generate frame. 実際に対応づけに使用する
  genFootMidCoords.rotation() = footMidCoords.value().rotation();
  genFootMidCoords.translation() = pinocchio::centerOfMass(model, actRobot, false) - gaitParam.l; // 1周期前のlを使っているが、lは不連続に変化するものではないので良い

  // refRobotPosのrefFootMidCoordsを求めてrefRobotに変換する
  double refdz;
  std::vector<pinocchio::SE3> refEEPoseFK(gaitParam.eeName.size());
  this->convertRefRobotRaw(gaitParam, model, genFootMidCoords,
                           refRobot, refEEPoseFK, refdz);

  // refEEPosePosのrefFootMidCoordsを求めて変換する
  std::vector<pinocchio::SE3> refEEPoseWithOutFK(gaitParam.eeName.size());
  this->convertRefEEPoseRaw(gaitParam, model, genFootMidCoords,
                            refEEPoseWithOutFK);

  // refEEPoseを求める
  std::vector<pinocchio::SE3> refEEPose(gaitParam.eeName.size());
  for(int i=0;i<gaitParam.eeName.size();i++){
    refEEPose[i] = mathutil::calcMidCoords(std::vector<pinocchio::SE3>{refEEPoseFK[i], refEEPoseWithOutFK[i]},
                                           std::vector<double>{this->solveFKMode.value(), 1.0 - this->solveFKMode.value()});
  }

  // refEEWrenchを計算
  std::vector<Eigen::Vector6d> refEEWrench(gaitParam.eeName.size());
  for(int i=0;i<gaitParam.eeName.size();i++){
    refEEWrench[i].head<3>() = footMidCoords.value().rotation() * gaitParam.refEEWrenchOrigin[i].head<3>();
    refEEWrench[i].tail<3>() = footMidCoords.value().rotation() * gaitParam.refEEWrenchOrigin[i].tail<3>();
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
  pinocchio::SE3 rleg = gaitParam.footStepNodesList[0].dstCoords[RLEG];
  rleg.translation() += rleg.rotation() * gaitParam.copOffset[RLEG].value();
  pinocchio::SE3 lleg = gaitParam.footStepNodesList[0].dstCoords[LLEG];
  lleg.translation() += lleg.rotation() * gaitParam.copOffset[LLEG].value();
  pinocchio::SE3 midCoords = mathutil::calcMidCoords(std::vector<pinocchio::SE3>{rleg, lleg}, std::vector<double>{1.0, 1.0});
  rleg = mathutil::orientCoordToAxis(rleg, Eigen::Vector3d::UnitZ());
  lleg = mathutil::orientCoordToAxis(lleg, Eigen::Vector3d::UnitZ());
  midCoords = mathutil::orientCoordToAxis(midCoords, Eigen::Vector3d::UnitZ());
  rleg.translation() -= rleg.rotation() * gaitParam.defaultTranslatePos[RLEG].value();
  lleg.translation() -= lleg.rotation() * gaitParam.defaultTranslatePos[LLEG].value();
  if(gaitParam.footStepNodesList[0].isSupportPhase[RLEG] && gaitParam.footStepNodesList[0].isSupportPhase[LLEG]){ // 両足支持
    footMidCoords.setGoal(midCoords, gaitParam.footStepNodesList[0].remainTime);
  }else if(gaitParam.footStepNodesList[0].isSupportPhase[RLEG] && !gaitParam.footStepNodesList[0].isSupportPhase[LLEG]){ // 右足支持
    if(gaitParam.footStepNodesList.size() > 1 &&
       (gaitParam.footStepNodesList[1].isSupportPhase[RLEG] && gaitParam.footStepNodesList[1].isSupportPhase[LLEG]) // 次が両足支持
       ){
      pinocchio::SE3 rleg = gaitParam.footStepNodesList[1].dstCoords[RLEG];
      rleg.translation() += rleg.rotation() * gaitParam.copOffset[RLEG].value();
      pinocchio::SE3 lleg = gaitParam.footStepNodesList[1].dstCoords[LLEG];
      lleg.translation() += lleg.rotation() * gaitParam.copOffset[LLEG].value();
      pinocchio::SE3 midCoords = mathutil::calcMidCoords(std::vector<pinocchio::SE3>{rleg, lleg}, std::vector<double>{1.0, 1.0});
      midCoords = mathutil::orientCoordToAxis(midCoords, Eigen::Vector3d::UnitZ());
      footMidCoords.setGoal(midCoords, gaitParam.footStepNodesList[0].remainTime + gaitParam.footStepNodesList[1].remainTime);
    }else{
      footMidCoords.setGoal(rleg, gaitParam.footStepNodesList[0].remainTime);
    }
  }else if(!gaitParam.footStepNodesList[0].isSupportPhase[RLEG] && gaitParam.footStepNodesList[0].isSupportPhase[LLEG]){ // 左足支持
    if(gaitParam.footStepNodesList.size() > 1 &&
       (gaitParam.footStepNodesList[1].isSupportPhase[RLEG] && gaitParam.footStepNodesList[1].isSupportPhase[LLEG]) // 次が両足支持
       ){
      pinocchio::SE3 rleg = gaitParam.footStepNodesList[1].dstCoords[RLEG];
      rleg.translation() += rleg.rotation() * gaitParam.copOffset[RLEG].value();
      pinocchio::SE3 lleg = gaitParam.footStepNodesList[1].dstCoords[LLEG];
      lleg.translation() += lleg.rotation() * gaitParam.copOffset[LLEG].value();
      pinocchio::SE3 midCoords = mathutil::calcMidCoords(std::vector<pinocchio::SE3>{rleg, lleg}, std::vector<double>{1.0, 1.0});
      midCoords = mathutil::orientCoordToAxis(midCoords, Eigen::Vector3d::UnitZ());
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
void RefToGenFrameConverter::convertRefRobotRaw(const GaitParam& gaitParam, const pinocchio::Model& model, const pinocchio::SE3& genFootMidCoords, pinocchio::Data& refRobot, std::vector<pinocchio::SE3>& refEEPoseFK, double& refdz) const{
  pinocchio::forwardKinematics(model,refRobot,gaitParam.refRobotPos);
  pinocchio::SE3 refrleg = refRobot.oMi[model.getJointId(gaitParam.eeParentLink[RLEG])]*gaitParam.eeLocalT[RLEG];
  pinocchio::SE3 reflleg = refRobot.oMi[model.getJointId(gaitParam.eeParentLink[LLEG])]*gaitParam.eeLocalT[LLEG];
  pinocchio::SE3 refFootMidCoords = mathutil::calcMidCoords(std::vector<pinocchio::SE3>{refrleg, reflleg},
                                                            std::vector<double>{1.0, 1.0});
  pinocchio::SE3 refFootOriginCoords = mathutil::orientCoordToAxis(refFootMidCoords, Eigen::Vector3d::UnitZ());

  pinocchio::SE3 transform = genFootMidCoords*refFootMidCoords.inverse();
  Eigen::Vector3d pos = gaitParam.refRobotPos.segment(0,3);
  Eigen::Quaterniond q;
  q.coeffs() = gaitParam.refRobotPos.segment(3,4);
  pinocchio::SE3 rootT = transform * pinocchio::SE3(q,pos);
  Eigen::VectorXd posForRefRobot = gaitParam.refRobotPos;
  posForRefRobot.segment(0,3) = rootT.translation();
  Eigen::Quaterniond qTransform(rootT.rotation());
  posForRefRobot.segment(3,4) = qTransform.coeffs();
  pinocchio::forwardKinematics(model,refRobot,posForRefRobot);

  refdz = (genFootMidCoords.inverse() * pinocchio::SE3(Eigen::Matrix3d::Identity(), pinocchio::centerOfMass(model, refRobot, false))).translation()[2]; // ref重心高さ

  for(int i=0;i<gaitParam.eeName.size();i++){
    refEEPoseFK[i] = refRobot.oMi[model.getJointId(gaitParam.eeParentLink[i])]*gaitParam.eeLocalT[i];
  }
}

// refEEPoseRawを変換する.
void RefToGenFrameConverter::convertRefEEPoseRaw(const GaitParam& gaitParam, const pinocchio::Model& model, const pinocchio::SE3& genFootMidCoords, std::vector<pinocchio::SE3>& refEEPoseWithOutFK) const{
  pinocchio::SE3 refFootMidCoords = this->calcRefFootMidCoords(gaitParam.refEEPoseRaw[RLEG].value(), gaitParam.refEEPoseRaw[LLEG].value(), gaitParam);
  refFootMidCoords = mathutil::orientCoordToAxis(refFootMidCoords, Eigen::Vector3d::UnitZ()); // 足裏を水平になるように傾け直さずに、もとの傾きをそのまま使うことに相当
  pinocchio::SE3 transform = genFootMidCoords * refFootMidCoords.inverse();
  for(int i=0;i<gaitParam.eeName.size();i++){
    refEEPoseWithOutFK[i] = transform * gaitParam.refEEPoseRaw[i].value();
  }
}

pinocchio::SE3 RefToGenFrameConverter::calcRefFootMidCoords(const pinocchio::SE3& rleg_, const pinocchio::SE3& lleg_, const GaitParam& gaitParam) const {
  pinocchio::SE3 rleg = rleg_;
  rleg.translation() += rleg.rotation() * gaitParam.copOffset[RLEG].value();
  pinocchio::SE3 lleg = lleg_;
  lleg.translation() += lleg.rotation() * gaitParam.copOffset[LLEG].value();

  pinocchio::SE3 bothmidcoords = mathutil::calcMidCoords(std::vector<pinocchio::SE3>{rleg, lleg},
                                                          std::vector<double>{1.0, 1.0});
  pinocchio::SE3 rlegmidcoords = rleg;
  rlegmidcoords.translation() -= rlegmidcoords.rotation() * gaitParam.defaultTranslatePos[RLEG].value();
  pinocchio::SE3 llegmidcoords = lleg;
  llegmidcoords.translation() -= llegmidcoords.rotation() * gaitParam.defaultTranslatePos[LLEG].value();

  double bothweight = std::min(this->refFootOriginWeight[RLEG].value(), this->refFootOriginWeight[LLEG].value());
  double rlegweight = this->refFootOriginWeight[RLEG].value() - bothweight;
  double llegweight = this->refFootOriginWeight[LLEG].value() - bothweight;
  return mathutil::calcMidCoords(std::vector<pinocchio::SE3>{bothmidcoords, rlegmidcoords, llegmidcoords},
                                 std::vector<double>{bothweight, rlegweight, llegweight});
}
