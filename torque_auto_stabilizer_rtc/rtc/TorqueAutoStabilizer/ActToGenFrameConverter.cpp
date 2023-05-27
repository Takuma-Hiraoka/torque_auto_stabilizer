#include "ActToGenFrameConverter.h"
#include "pinocchio/algorithm/center-of-mass.hpp"

bool ActToGenFrameConverter::convertFrame(const GaitParam& gaitParam, const pinocchio::Model& model, double dt, // input
                                          pinocchio::Data& actRobot, std::vector<pinocchio::SE3>& o_actEEPose, std::vector<Eigen::Vector6d>& o_actFSensorWrench, mathutil::FirstOrderLowPassFilter<Eigen::Vector3d>& o_actCogVel) const {
  Eigen::Vector3d actCogPrev = pinocchio::centerOfMass(model, actRobot, false); // 各リンクの重心は使わない．

  {
    // FootOrigin座標系を用いてactRobotPos/Velをgenerate frameに投影しactRobotとする
    Eigen::VectorXd actRobotPosOffset = gaitParam.actRobotPos;
    Eigen::Quaterniond qOrig;
    qOrig.coeffs() = actRobotPosOffset.segment(3,4);
    Eigen::Quaterniond qOffset = Eigen::AngleAxisd(qOrig) * Eigen::AngleAxisd(mathutil::rotFromRpy(this->rpyOffset[0], this->rpyOffset[1], this->rpyOffset[2])); // rpyOffsetを適用
    actRobotPosOffset.segment(3,4) = qOffset.coeffs();
    pinocchio::forwardKinematics(model,actRobot,actRobotPosOffset);
    //TODO
    double rlegweight = 1.0;//gaitParam.footstepNodesList[0].isSupportPhase[RLEG]? 1.0 : 0.0;
    double llegweight = 0.0;//gaitParam.footstepNodesList[0].isSupportPhase[LLEG]? 1.0 : 0.0;
    pinocchio::SE3 actrleg = actRobot.oMi[model.getJointId(gaitParam.eeParentLink[RLEG])]*gaitParam.eeLocalT[RLEG];
    pinocchio::SE3 actlleg = actRobot.oMi[model.getJointId(gaitParam.eeParentLink[LLEG])]*gaitParam.eeLocalT[LLEG];
    pinocchio::SE3 actFootMidCoords = mathutil::calcMidCoords(std::vector<pinocchio::SE3>{actrleg, actlleg},
                                                               std::vector<double>{rlegweight, llegweight});
    pinocchio::SE3 actFootOriginCoords = mathutil::orientCoordToAxis(actFootMidCoords, Eigen::Vector3d::UnitZ());
    pinocchio::SE3 genFootMidCoords = mathutil::calcMidCoords(std::vector<pinocchio::SE3>{gaitParam.eeTargetPose[RLEG], gaitParam.eeTargetPose[LLEG]},
                                                               std::vector<double>{rlegweight, llegweight});  // 1周期前のeeTargetPoseを使っているが、eeTargetPoseは不連続に変化するものではないのでよい
    pinocchio::SE3 genFootOriginCoords = mathutil::orientCoordToAxis(genFootMidCoords, Eigen::Vector3d::UnitZ());
    pinocchio::SE3 transform = genFootMidCoords*actFootMidCoords.inverse();
    Eigen::Vector3d pos = actRobotPosOffset.segment(0,3);
    pinocchio::SE3 rootT = transform * pinocchio::SE3(qOffset,pos);
    actRobotPosOffset.segment(0,3) = rootT.translation();
    Eigen::Quaterniond qTransform(rootT.rotation());
    actRobotPosOffset.segment(3,4) = qTransform.coeffs();
    pinocchio::forwardKinematics(model,actRobot,actRobotPosOffset);
  }

  std::vector<pinocchio::SE3> actEEPose(gaitParam.eeName.size());
  std::vector<Eigen::Vector6d> actFSensorWrench(gaitParam.eeName.size(), Eigen::Vector6d::Zero());
  {
    // 各エンドエフェクタのactualの位置・力を計算
    for(int i=0;i<gaitParam.eeName.size(); i++){
      actEEPose[i] = actRobot.oMi[model.getJointId(gaitParam.eeParentLink[i])]*gaitParam.eeLocalT[i];
      pinocchio::SE3 senPose = actRobot.oMi[model.getJointId(gaitParam.fsensorParentLink[i])]*gaitParam.fsensorLocalT[i];
      pinocchio::SE3 eeTosenPose = actEEPose[i].inverse() * senPose;
      Eigen::Vector6d eefF; // endeffector frame. endeffector origin.
      eefF.head<3>() = eeTosenPose.rotation() * gaitParam.actFSensorWrenchOrigin[i].head<3>();
      eefF.tail<3>() = eeTosenPose.rotation() * gaitParam.actFSensorWrenchOrigin[i].tail<3>() + eeTosenPose.translation().cross(eefF.head<3>());
      actFSensorWrench[i].head<3>() = actEEPose[i].rotation() * eefF.head<3>();
      actFSensorWrench[i].tail<3>() = actEEPose[i].rotation() * eefF.tail<3>();

    }
  }

  Eigen::Vector3d actCogVel;
  {
    // actCogを計算
    bool genContactState_changed = false;
    for(int i=0;i<NUM_LEGS;i++){
      // TODO
      //if(gaitParam.footstepNodesList[0].isSupportPhase[i] != gaitParam.prevSupportPhase[i]) genContactState_changed = true;
    }
    if(genContactState_changed){
      //座標系が飛んでいるので、gaitParam.actCogVel は前回の周期の値をそのままつかう
      actCogVel = gaitParam.actCogVel.value();
    }else{
      actCogVel = (pinocchio::centerOfMass(model, actRobot, false) - actCogPrev) / dt;
    }
  }

  o_actEEPose = actEEPose;
  o_actFSensorWrench = actFSensorWrench;
  if(this->isInitial){
    o_actCogVel.reset(Eigen::Vector3d::Zero());
  } else {
    o_actCogVel.passFilter(actCogVel, dt);
  }

  this->isInitial = false;

  return true;
}
