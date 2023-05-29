#include "LegCoordsGenerator.h"

void LegCoordsGenerator::initLegCoords(const GaitParam& gaitParam,
                                       std::vector<footguidedcontroller::LinearTrajectory<Eigen::Vector3d> >& o_refZmpTraj, std::vector<mathutil::TwoPointInterpolatorSE3>& o_genCoords) const{
  std::vector<footguidedcontroller::LinearTrajectory<Eigen::Vector3d> > refZmpTraj;
  std::vector<mathutil::TwoPointInterpolatorSE3> genCoords;

  pinocchio::SE3 rlegCoords = gaitParam.footStepNodesList[0].dstCoords[RLEG];
  pinocchio::SE3 llegCoords = gaitParam.footStepNodesList[0].dstCoords[LLEG];

  genCoords.emplace_back(rlegCoords, Eigen::Vector6d::Zero(), Eigen::Vector6d::Zero(), mathutil::HOFFARBIB);
  genCoords.emplace_back(llegCoords, Eigen::Vector6d::Zero(), Eigen::Vector6d::Zero(), mathutil::HOFFARBIB);

  Eigen::Vector3d zmp;
  if(gaitParam.footStepNodesList[0].isSupportPhase[RLEG] && gaitParam.footStepNodesList[0].isSupportPhase[LLEG]){
    zmp = 0.5 * (rlegCoords.translation() + rlegCoords.rotation()*gaitParam.copOffset[RLEG].value()) + 0.5 * (llegCoords.translation() + llegCoords.rotation()*gaitParam.copOffset[LLEG].value());
  }else if(gaitParam.footStepNodesList[0].isSupportPhase[RLEG]){
    zmp = rlegCoords.translation() + rlegCoords.rotation()*gaitParam.copOffset[RLEG].value();
  }else{
    zmp = llegCoords.translation() + llegCoords.rotation()*gaitParam.copOffset[LLEG].value();
  }
  refZmpTraj.emplace_back(zmp,zmp,0.0);

  o_refZmpTraj = refZmpTraj;
  o_genCoords = genCoords;
}

void LegCoordsGenerator::calcLegCoords(const GaitParam& gaitParam, const pinocchio::Model& model, double dt,
                                       std::vector<footguidedcontroller::LinearTrajectory<Eigen::Vector3d> >& o_refZmpTraj, std::vector<mathutil::TwoPointInterpolatorSE3>& o_genCoords, std::vector<GaitParam::SwingState_enum>& o_swingState) const{
  // swing期は、remainTime - supportTime - delayTimeOffset後にdstCoordsに到達するようなantececdent軌道を生成し(genCoords.getGoal()の値)、その軌道にdelayTimeOffset遅れで滑らかに追従するような軌道(genCoords.value()の値)を生成する.
  //   rectangle以外の軌道タイプや跳躍についてはひとまず考えない TODO
  //   srcCoordsとdstCoordsを結ぶ軌道を生成する. srcCoordsの高さ+[0]とdstCoordsの高さ+[1]の高い方(heightとおく)に上げるようなrectangle軌道を生成する
  // support期は、現FootStepNodesの終了時にdstCoordsに到達するような軌道を線形補間によって生成する.

  // refZmpTrajを更新し進める
  std::vector<footguidedcontroller::LinearTrajectory<Eigen::Vector3d> > refZmpTraj = gaitParam.refZmpTraj;
  {
    Eigen::Vector3d refZmp = refZmpTraj[0].getStart(); // for文中の現在のrefzmp
    refZmpTraj.clear();
    // footStepNodesListのサイズが1, footStepNodesList[0].remainTimeが0のときに、copOffsetのパラメータが滑らかに変更になる場合がある. それに対応できるように
    for(int i=0;i<gaitParam.footStepNodesList.size();i++){
      Eigen::Vector3d zmpGoalPos;

      if(gaitParam.footStepNodesList[i].endRefZmpState == GaitParam::FootStepNodes::refZmpState_enum::RLEG){
        pinocchio::SE3 rlegGoalCoords = gaitParam.footStepNodesList[i].dstCoords[RLEG]; // このfootStepNode終了時にdstCoordsに行くように線形補間
        zmpGoalPos = rlegGoalCoords.translation() + rlegGoalCoords.rotation()*gaitParam.copOffset[RLEG].value();
      }else if(gaitParam.footStepNodesList[i].endRefZmpState == GaitParam::FootStepNodes::refZmpState_enum::LLEG){
        pinocchio::SE3 llegGoalCoords = gaitParam.footStepNodesList[i].dstCoords[LLEG]; // このfootStepNode終了時にdstCoordsに行くように線形補間
        zmpGoalPos = llegGoalCoords.translation() + llegGoalCoords.rotation()*gaitParam.copOffset[LLEG].value();
      }else{ //gaitParam.footStepNodesList[i].endRefZmpState == GaitParam::FootStepNodes::refZmpState_enum::MIDDLE
        pinocchio::SE3 rlegGoalCoords = gaitParam.footStepNodesList[i].dstCoords[RLEG];
        pinocchio::SE3 llegGoalCoords = gaitParam.footStepNodesList[i].dstCoords[LLEG];
        Eigen::Vector3d rlegCOP = rlegGoalCoords.translation() + rlegGoalCoords.rotation()*gaitParam.copOffset[RLEG].value();
        Eigen::Vector3d llegCOP = llegGoalCoords.translation() + llegGoalCoords.rotation()*gaitParam.copOffset[LLEG].value();
        zmpGoalPos = 0.5 * rlegCOP + 0.5 * llegCOP;
      }
      refZmpTraj.push_back(footguidedcontroller::LinearTrajectory<Eigen::Vector3d>(refZmp,zmpGoalPos,std::max(gaitParam.footStepNodesList[i].remainTime, dt)));
      refZmp = zmpGoalPos;

      if(i >= this->previewStepNum - 1){
        // 片足支持期で終わるのではなく、両足支持期のrefZmpの位置まで予見した方が性能が良い
        if(/*NOT*/!(((gaitParam.footStepNodesList[i].isSupportPhase[RLEG] && !gaitParam.footStepNodesList[i].isSupportPhase[LLEG]) || (!gaitParam.footStepNodesList[i].isSupportPhase[RLEG] && gaitParam.footStepNodesList[i].isSupportPhase[LLEG])) && // 片足支持期
                    ((!(i==gaitParam.footStepNodesList.size()-1)) && gaitParam.footStepNodesList[i+1].isSupportPhase[RLEG] && gaitParam.footStepNodesList[i+1].isSupportPhase[LLEG])) // 次が両足支持期
           ) break;
      }
    }

    // footGuidedBalanceTime[s]に満たない場合、満たないぶんだけ末尾に加える. そうしないと終端条件が厳しすぎる. 一方で、常に末尾にfootGuidedBalanceTime[s]だけ加えると、終端条件がゆるすぎて重心を動かすのが遅すぎる?
    double totalTime = 0;
    for(int i=0;i<refZmpTraj.size();i++) totalTime += refZmpTraj[i].getTime();
    if(totalTime < this->footGuidedBalanceTime){
      refZmpTraj.push_back(footguidedcontroller::LinearTrajectory<Eigen::Vector3d>(refZmp,refZmp, std::max(this->footGuidedBalanceTime - totalTime, dt)));
    }

    // dtだけ進める
    if(refZmpTraj[0].getTime() <= dt){
      if(refZmpTraj.size() > 1) refZmpTraj.erase(refZmpTraj.begin());
      else refZmpTraj[0] = footguidedcontroller::LinearTrajectory<Eigen::Vector3d>(refZmpTraj[0].getGoal(),refZmpTraj[0].getGoal(),0.0);
    }else{
      refZmpTraj[0] = footguidedcontroller::LinearTrajectory<Eigen::Vector3d>(refZmpTraj[0].getStart()+refZmpTraj[0].getSlope()*dt,refZmpTraj[0].getGoal(),refZmpTraj[0].getTime()-dt);
    }
  }

  // genCoordsを進める
  std::vector<mathutil::TwoPointInterpolatorSE3> genCoords = gaitParam.genCoords;
  std::vector<GaitParam::SwingState_enum> swingState = gaitParam.swingState;
  for(int i=0;i<NUM_LEGS;i++){
    if(gaitParam.footStepNodesList[0].stopCurrentPosition[i]){ // for early touch down. 今の位置に止める
      genCoords[i].reset(gaitParam.actRobot.oMi[model.getJointId(gaitParam.eeParentLink[i])]*gaitParam.eeLocalT[i]);
      continue;
    }
    if(gaitParam.footStepNodesList[0].isSupportPhase[i]) { // 支持脚．どうせタスク空間のPDゲインは0なので気にしない．
      genCoords[i].reset(gaitParam.actRobot.oMi[model.getJointId(gaitParam.eeParentLink[i])]*gaitParam.eeLocalT[i]);
    }else{ // 遊脚
      double height = std::max(gaitParam.srcCoords[i].translation()[2] + gaitParam.footStepNodesList[0].stepHeight[i][0],
                               gaitParam.footStepNodesList[0].dstCoords[i].translation()[2] + gaitParam.footStepNodesList[0].stepHeight[i][1]); // 足上げ高さ. generate frame
      pinocchio::SE3 srcCoords = gaitParam.srcCoords[i];
      pinocchio::SE3 dstCoords = gaitParam.footStepNodesList[0].dstCoords[i];
      dstCoords.translation()[2] += gaitParam.footStepNodesList[0].goalOffset[i];
      pinocchio::SE3 antecedentCoords = genCoords[i].getGoal(); // 今のantecedent軌道の位置

      // phase transition
      if(swingState[i] == GaitParam::LIFT_PHASE){
        if(gaitParam.footStepNodesList[0].remainTime <= this->delayTimeOffset) swingState[i] = GaitParam::DOWN_PHASE;
        else if(antecedentCoords.translation()[2] >= height - 1e-3) swingState[i] = GaitParam::SWING_PHASE;
      }else if(swingState[i] == GaitParam::SWING_PHASE){
        if(gaitParam.footStepNodesList[0].remainTime <= this->delayTimeOffset) swingState[i] = GaitParam::DOWN_PHASE;
        else if(antecedentCoords.translation()[2] < dstCoords.translation()[2] - 1e-3) swingState[i] = GaitParam::LIFT_PHASE;
      }else{
        // 一度DOWN_PHASEになったら別のPHASEになることはない
      }

      if(swingState[i] == GaitParam::LIFT_PHASE){
        Eigen::Vector3d viaPos0 = antecedentCoords.translation(); viaPos0[2] = height;
        double length0 = (viaPos0 - antecedentCoords.translation()).norm();
        Eigen::Vector3d viaPos1 = dstCoords.translation(); viaPos1[2] = height;
        double length1 = (viaPos1 - viaPos0).norm();
        double length2 = (dstCoords.translation() - viaPos1).norm();
        double totalLength = length0 + length1 + length2 * this->finalDistanceWeight;
        double ratio = std::min(dt / (gaitParam.footStepNodesList[0].remainTime - this->delayTimeOffset), 1.0); // LIFT_PHASEのとき必ずgaitParam.footStepNodesList[0].remainTime - this->delayTimeOffset>0
        double dp = ratio * totalLength;
        Eigen::Vector3d goal;
        if(dp < length0){
          Eigen::Vector3d dir = ((viaPos0 - antecedentCoords.translation()).norm() > 0) ? (viaPos0 - antecedentCoords.translation()).normalized() : Eigen::Vector3d::Zero();
            goal = antecedentCoords.translation() + dp * dir;
        }else{
          dp -= length0;
          if(dp < length1){
            Eigen::Vector3d dir = ((viaPos1 - viaPos0).norm() > 0) ? (viaPos1 - viaPos0).normalized() : Eigen::Vector3d::Zero();
            goal = viaPos0 + dp * dir;
          }else{
            dp -= length1; dp /= this->finalDistanceWeight;
            Eigen::Vector3d dir = ((dstCoords.translation() - viaPos1).norm() > 0) ? (dstCoords.translation() - viaPos1).normalized() : Eigen::Vector3d::Zero();
            goal = viaPos1 + dp * dir;
          }
        }
	// TODO 補間はdpの段階でやっているのでgenCoords = nextCoordsでもよいかも？
	// 或いはdelayTimeOffsetはもっと短くてよいかも？
        pinocchio::SE3 nextCoords;
        nextCoords.translation() = goal;
        nextCoords.rotation() = mathutil::calcMidRot(std::vector<Eigen::Matrix3d>{antecedentCoords.rotation(),dstCoords.rotation()},
                                                   std::vector<double>{std::max(0.0,gaitParam.footStepNodesList[0].remainTime - this->delayTimeOffset - dt), dt}); // dstCoordsについたときにdstCoordsの傾きになるように線形補間
        Eigen::Vector6d goalVel = (Eigen::Vector6d() << 0.0, 0.0, -gaitParam.footStepNodesList[0].touchVel[i], 0.0, 0.0, 0.0).finished(); // pはgenerate frame. RはgoalCoords frame.
        genCoords[i].setGoal(nextCoords, goalVel, this->delayTimeOffset);
        genCoords[i].interpolate(dt);
      }else if(swingState[i] == GaitParam::SWING_PHASE ||
               swingState[i] == GaitParam::DOWN_PHASE){
        if(gaitParam.footStepNodesList[0].remainTime <= this->delayTimeOffset){
          Eigen::Vector6d goalVel = (Eigen::Vector6d() << 0.0, 0.0, -gaitParam.footStepNodesList[0].touchVel[i], 0.0, 0.0, 0.0).finished(); // pはgenerate frame. RはgoalCoords frame.
          genCoords[i].setGoal(dstCoords, goalVel, gaitParam.footStepNodesList[0].remainTime);
          genCoords[i].interpolate(dt);
        }else{
          Eigen::Vector3d viaPos1 = dstCoords.translation(); viaPos1[2] = antecedentCoords.translation()[2];
          double length1 = (viaPos1 - antecedentCoords.translation()).norm();
          double length2 = (dstCoords.translation() - viaPos1).norm();
          double totalLength = length1 + length2 * this->finalDistanceWeight;
          double ratio = std::min(dt / (gaitParam.footStepNodesList[0].remainTime - this->delayTimeOffset), 1.0); // 必ずgaitParam.footStepNodesList[0].remainTime - this->delayTimeOffset>0
          Eigen::Vector3d goal;
          double dp = ratio * totalLength;
          if(dp < length1){
            Eigen::Vector3d dir = ((viaPos1 - antecedentCoords.translation()).norm() > 0) ? (viaPos1 - antecedentCoords.translation()).normalized() : Eigen::Vector3d::Zero();
            goal = antecedentCoords.translation() + dp * dir;
          }else{
            dp -= length1; dp /= this->finalDistanceWeight;
            Eigen::Vector3d dir = ((dstCoords.translation() - viaPos1).norm() > 0) ? (dstCoords.translation() - viaPos1).normalized() : Eigen::Vector3d::Zero();
            goal = viaPos1 + dp * dir;
          }
          pinocchio::SE3 nextCoords;
          nextCoords.translation() = goal;
          nextCoords.rotation() = mathutil::calcMidRot(std::vector<Eigen::Matrix3d>{antecedentCoords.rotation(),dstCoords.rotation()},
                                                     std::vector<double>{std::max(0.0,gaitParam.footStepNodesList[0].remainTime - this->delayTimeOffset - dt), dt}); // dstCoordsについたときにdstCoordsの傾きになるように線形補間
          Eigen::Vector6d goalVel = (Eigen::Vector6d() << 0.0, 0.0, -gaitParam.footStepNodesList[0].touchVel[i], 0.0, 0.0, 0.0).finished(); // pはgenerate frame. RはgoalCoords frame.
          genCoords[i].setGoal(nextCoords, goalVel, this->delayTimeOffset);
          genCoords[i].interpolate(dt);
        }
      }
    }
  }

  o_refZmpTraj = refZmpTraj;
  o_genCoords = genCoords;
  o_swingState = swingState;
}
