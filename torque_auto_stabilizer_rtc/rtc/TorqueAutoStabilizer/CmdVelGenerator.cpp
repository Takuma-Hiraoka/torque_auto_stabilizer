#include "CmdVelGenerator.h"
#include "MathUtil.h"

bool CmdVelGenerator::calcCmdVel(const GaitParam& gaitParam,
                                 cnoid::Vector3& o_cmdVel) const{

  cnoid::Vector3 cmdVel = this->refCmdVel;

  if(this->isGraspLessManipMode) {
    cnoid::Vector3 graspLessCmdVel = cnoid::Vector3::Zero();
    this->calcVelFromHandError(gaitParam,
                               graspLessCmdVel);
    cmdVel += graspLessCmdVel;
  }

  o_cmdVel = cmdVel;
  return true;
}


void CmdVelGenerator::calcVelFromHandError(const GaitParam& gaitParam,
                                           cnoid::Vector3& o_graspLessCmdVel) const{
  cnoid::Vector3 graspLessCmdVel = cnoid::Vector3::Zero(); // footPos frame. footPos origin

  cnoid::Position footPos; // generate frame. 次の一歩の支持脚のCoords-defaultTranslatePos. Z軸は鉛直. Z=0.0
  if(gaitParam.footstepNodesList.size() > 1 &&
     (gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && gaitParam.footstepNodesList[0].isSupportPhase[LLEG])){ // 現在両足支持期.
    if(!gaitParam.footstepNodesList[1].isSupportPhase[RLEG] && gaitParam.footstepNodesList[1].isSupportPhase[LLEG]){ // 次右脚をswing
      footPos = mathutil::orientCoordToAxis(gaitParam.footstepNodesList[0].dstCoords[LLEG], cnoid::Vector3::UnitZ());
      footPos.translation() -= footPos.linear() * gaitParam.defaultTranslatePos[LLEG].value();
    }else if(gaitParam.footstepNodesList[1].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[1].isSupportPhase[LLEG]){ // 次左脚をswing
      footPos = mathutil::orientCoordToAxis(gaitParam.footstepNodesList[0].dstCoords[RLEG], cnoid::Vector3::UnitZ());
      footPos.translation() -= footPos.linear() * gaitParam.defaultTranslatePos[RLEG].value();
    }else{
      o_graspLessCmdVel = graspLessCmdVel;
      return;
    }
  }else if(gaitParam.footstepNodesList.size() > 2 &&
           (gaitParam.footstepNodesList[1].isSupportPhase[RLEG] && gaitParam.footstepNodesList[1].isSupportPhase[LLEG])){ // 次両足支持期
    if(!gaitParam.footstepNodesList[2].isSupportPhase[RLEG] && gaitParam.footstepNodesList[2].isSupportPhase[LLEG]){ // 次の次右脚をswing
      footPos = mathutil::orientCoordToAxis(gaitParam.footstepNodesList[1].dstCoords[LLEG], cnoid::Vector3::UnitZ());
      footPos.translation() -= footPos.linear() * gaitParam.defaultTranslatePos[LLEG].value();
    }else if(gaitParam.footstepNodesList[2].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[2].isSupportPhase[LLEG]){ // 次の次左脚をswing
      footPos = mathutil::orientCoordToAxis(gaitParam.footstepNodesList[1].dstCoords[RLEG], cnoid::Vector3::UnitZ());
      footPos.translation() -= footPos.linear() * gaitParam.defaultTranslatePos[RLEG].value();
    }else{
      o_graspLessCmdVel = graspLessCmdVel;
      return;
    }
  }else{
    o_graspLessCmdVel = graspLessCmdVel;
    return;
  }
  footPos.translation()[2] = 0.0;

  o_graspLessCmdVel = graspLessCmdVel;
  return;
}
