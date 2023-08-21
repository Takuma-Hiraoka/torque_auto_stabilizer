#include "Stabilizer.h"
#include "MathUtil.h"
#include <cnoid/Jacobian>
#include <cnoid/EigenUtil>
#include <cnoid/src/Body/InverseDynamics.h>

void Stabilizer::initStabilizerOutput(const GaitParam& gaitParam,
                                      cnoid::Vector3& o_stTargetZmp, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoPGainPercentage, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoDGainPercentage) const{
  o_stTargetZmp = gaitParam.refZmpTraj[0].getStart();
  for(int i=0;i<o_stServoPGainPercentage.size();i++){
    o_stServoPGainPercentage[i].reset(100.0);
    o_stServoDGainPercentage[i].reset(100.0);
  }
}

bool Stabilizer::calcResolvedAccelationControl(const GaitParam& gaitParam, double dt, bool useActState, cnoid::BodyPtr& actRobotTqc, 
				     cnoid::Vector3& o_stTargetZmp, std::vector<cnoid::Vector6>& o_stEETargetWrench,
				     std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoPGainPercentage, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoDGainPercentage) const{
  // - 現在のactual重心位置から、目標ZMPを計算
  // - 目標位置姿勢を満たすように分解加速度制御. 重心が倒立振子で加速された場合のトルクを計算
  // - 目標のZMPを満たすように目標足裏反力を計算、仮想仕事の原理で足し込む

  // - 現在のactual重心位置から、目標ZMPを計算
  cnoid::Vector3 tgtForce; // generate frame
  cnoid::Vector3 tgtCogAcc; // generate frame
  this->calcZMP(gaitParam, dt, useActState, // input
                o_stTargetZmp, tgtForce, tgtCogAcc); // output

  this->calcTorque(dt, gaitParam, useActState, actRobotTqc, tgtCogAcc,
		   o_stServoPGainPercentage, o_stServoDGainPercentage);

  // 目標ZMPを満たすように目標EndEffector反力を計算
  this->calcWrench(gaitParam, o_stTargetZmp, tgtForce, useActState,// input
                   actRobotTqc, o_stEETargetWrench); // output

  // 出力トルク制限
  // ラチェッティングトルクはこれよりも小さい
  for(int i=0;i<actRobotTqc->numJoints();i++){
    mathutil::clamp(actRobotTqc->joint(i)->u(), this->torque_limit[i]);
  }

  return true;
};

bool Stabilizer::calcZMP(const GaitParam& gaitParam, double dt, bool useActState,
                         cnoid::Vector3& o_tgtZmp, cnoid::Vector3& o_tgtForce, cnoid::Vector3& o_tgtCogAcc) const{
  cnoid::Vector3 cog = useActState ? gaitParam.actCog : gaitParam.genCog;
  cnoid::Vector3 cogVel = useActState ? gaitParam.actCogVel.value() : gaitParam.genCogVel;
  cnoid::Vector3 DCM = cog + cogVel / gaitParam.omega;
  const std::vector<cnoid::Position>& EEPose = useActState ? gaitParam.actEEPose : gaitParam.abcEETargetPose;

  cnoid::Vector3 tgtZmp;
  if(gaitParam.footstepNodesList[0].isSupportPhase[RLEG] || gaitParam.footstepNodesList[0].isSupportPhase[LLEG]){
    tgtZmp = footguidedcontroller::calcFootGuidedControl(gaitParam.omega,gaitParam.l,DCM,gaitParam.refZmpTraj);
    if(tgtZmp[2] >= gaitParam.actCog[2]) tgtZmp = gaitParam.actCog - cnoid::Vector3(gaitParam.l[0],gaitParam.l[1], 0.0); // 下向きの力は受けられないので
    else{
      // truncate zmp inside polygon. actual robotの関節角度を用いて計算する
      std::vector<cnoid::Vector3> vertices; // generate frame. 支持点の集合
      for(int i=0;i<NUM_LEGS;i++){
        if(!gaitParam.footstepNodesList[0].isSupportPhase[i]) continue;
        for(int j=0;j<gaitParam.legHull[i].size();j++){
          cnoid::Vector3 p = EEPose[i]*gaitParam.legHull[i][j];
          if(p[2] > gaitParam.actCog[2] - 1e-2) p[2] = gaitParam.actCog[2] - 1e-2; // 重心よりも支持点が高いと射影が破綻するので 
          vertices.push_back(p);
        }
      }
      tgtZmp = mathutil::calcInsidePointOfPolygon3D(tgtZmp,vertices,gaitParam.actCog - cnoid::Vector3(gaitParam.l[0],gaitParam.l[1], 0.0));
      // TODO. 角運動量オフセット.
    }
  }else{ // 跳躍期
    tgtZmp = cog - cnoid::Vector3(gaitParam.l[0],gaitParam.l[1], 0.0);
  }
  cnoid::Vector3 tgtCog,tgtCogVel,tgtCogAcc,tgtForce;
  footguidedcontroller::updateState(gaitParam.omega,gaitParam.l,cog,cogVel,tgtZmp,gaitParam.genRobot->mass(),dt,
                                      tgtCog, tgtCogVel, tgtCogAcc, tgtForce);

  // tgtForceにrefEEWrenchのXY成分を足す TODO

  o_tgtZmp = tgtZmp;
  o_tgtForce = tgtForce;
  o_tgtCogAcc = tgtCogAcc;
  return true;
}

bool Stabilizer::calcWrench(const GaitParam& gaitParam, const cnoid::Vector3& tgtZmp/*generate座標系*/, const cnoid::Vector3& tgtForce/*generate座標系 ロボットが受ける力*/, bool useActState, cnoid::BodyPtr& actRobotTqc, 
                            std::vector<cnoid::Vector6>& o_tgtEEWrench) const{
  std::vector<cnoid::Vector6> tgtEEWrench(gaitParam.eeName.size(), cnoid::Vector6::Zero()); /* 要素数EndEffector数. generate frame. EndEffector origin*/

  for(int i = 0;i<gaitParam.eeName.size();i++){
    tgtEEWrench[i] = gaitParam.refEEWrench[i];
  }

  /*
    legは、legから受けるwrenchの和がtgtZmp, tgtForceを満たすように.
    非Support期のlegには分配せずゼロを入れる. 全てのlegが非Support期なら分配計算すら行わない
    actual robotの関節角度を用いて計算する
    各EEFのwrenchを、polygonの各頂点からのSPAN表現で考える.
    各頂点のfx, fy, fzの向きは、合力の向きと同じで、ノルムだけを変数とする. 合力がtgtForce, ZMPがtgtZmpになるように、ノルムの値を求める.
      - nzが反映できない、力の向きの冗長性を利用できない、摩擦係数を考慮できない、といった欠点がある. 二次元動歩行なので、まずは物理的・数学的厳密性や冗長性の利用よりもシンプルさ、ロバストさを優先する. そのあたりをこだわりたいなら三次元多点接触でやる.
      - 動歩行の途中の一歩で偶然actualの足が90度以上倒れた姿勢で地面につくことがあるので、そうなったときにても破綻しないことが重要.
    最後に、FACE表現に変換する.

    階層QPのタスクは次の通り
    変数: SPAN表現のノルム. 0~1
    1. ノルム>0. 合力がtgtForce.
    2. ZMPがtgtZmp
    3. 各脚の各頂点のノルムの重心がCOPOffsetと一致 (fzの値でスケールされてしまうので、alphaを用いて左右をそろえる)
    4. ノルムの2乗和の最小化 (3の中で微小な重みで一緒にやる)
  */
  // 計算時間は、tgtZmpが支持領域内に無いと遅くなるなので、事前に支持領域内に入るように修正しておくこと
  const std::vector<cnoid::Position>& EEPose = useActState ? gaitParam.actEEPose : gaitParam.abcEETargetPose;

  if(gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[0].isSupportPhase[LLEG]){
    if(gaitParam.isManualControlMode[LLEG].getGoal() == 0.0) tgtEEWrench[LLEG].setZero(); // Manual Control ModeであればrefEEWrenchをそのまま使う
    tgtEEWrench[RLEG].head<3>() = tgtForce;
    tgtEEWrench[RLEG].tail<3>() = (tgtZmp - EEPose[RLEG].translation()).cross(tgtForce);
  }else if(!gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && gaitParam.footstepNodesList[0].isSupportPhase[LLEG]){
    if(gaitParam.isManualControlMode[RLEG].getGoal() == 0.0) tgtEEWrench[RLEG].setZero(); // Manual Control ModeであればrefEEWrenchをそのまま使う
    tgtEEWrench[LLEG].head<3>() = tgtForce;
    tgtEEWrench[LLEG].tail<3>() = (tgtZmp - EEPose[LLEG].translation()).cross(tgtForce);
  }else if(!gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[0].isSupportPhase[LLEG]){
    if(gaitParam.isManualControlMode[RLEG].getGoal() == 0.0) tgtEEWrench[RLEG].setZero(); // Manual Control ModeであればrefEEWrenchをそのまま使う
    if(gaitParam.isManualControlMode[LLEG].getGoal() == 0.0) tgtEEWrench[LLEG].setZero(); // Manual Control ModeであればrefEEWrenchをそのまま使う
  }else if(tgtForce.norm() == 0){
    if(gaitParam.isManualControlMode[RLEG].getGoal() == 0.0) tgtEEWrench[RLEG].setZero(); // Manual Control ModeであればrefEEWrenchをそのまま使う
    if(gaitParam.isManualControlMode[LLEG].getGoal() == 0.0) tgtEEWrench[LLEG].setZero(); // Manual Control ModeであればrefEEWrenchをそのまま使う
  }else{
    int dim = gaitParam.legHull[RLEG].size() + gaitParam.legHull[LLEG].size();
    {
      // 1. ノルム>0. 合力がtgtForce.

      // 合力がtgtForce. (合計が1)
      this->constraintTask_->A() = Eigen::SparseMatrix<double,Eigen::RowMajor>(1,dim);
      for(int i=0;i<dim;i++) this->constraintTask_->A().insert(0,i) = 1.0;
      this->constraintTask_->b() = Eigen::VectorXd::Ones(1);
      this->constraintTask_->wa() = cnoid::VectorX::Ones(1);

      // 各値が0~1
      this->constraintTask_->C() = Eigen::SparseMatrix<double,Eigen::RowMajor>(dim,dim);
      for(int i=0;i<dim;i++) this->constraintTask_->C().insert(i,i) = 1.0;
      this->constraintTask_->dl() = Eigen::VectorXd::Zero(dim);
      this->constraintTask_->du() = Eigen::VectorXd::Ones(dim);
      this->constraintTask_->wc() = cnoid::VectorX::Ones(dim);

      this->constraintTask_->w() = cnoid::VectorX::Ones(dim) * 1e-6;
      this->constraintTask_->toSolve() = false;
      this->constraintTask_->settings().verbose = 0;
    }
    {
      // 2. ZMPがtgtZmp
      // tgtZmpまわりのトルクの和を求めて、tgtForceの向きの単位ベクトルとの外積が0なら良い
      //this->tgtZmpTask_->A() = Eigen::SparseMatrix<double,Eigen::RowMajor>(3,dim);
      cnoid::Vector3 tgtForceDir = tgtForce.normalized();
      int idx = 0;
      Eigen::SparseMatrix<double,Eigen::ColMajor> A_ColMajor(3,dim); // insert()する順序がColMajorなので、RowMajorのAに直接insertすると計算効率が著しく悪い(ミリ秒単位で時間がかかる).
      for(int i=0;i<NUM_LEGS;i++){
        for(int j=0;j<gaitParam.legHull[i].size();j++){
          cnoid::Vector3 pos = EEPose[i] * gaitParam.legHull[i][j];
          cnoid::Vector3 a = tgtForceDir.cross( (pos - tgtZmp).cross(tgtForce));
          for(int k=0;k<3;k++) A_ColMajor.insert(k,idx) = a[k];
          idx ++;
        }
      }
      this->tgtZmpTask_->A() = A_ColMajor;
      this->tgtZmpTask_->b() = Eigen::VectorXd::Zero(3);
      this->tgtZmpTask_->wa() = cnoid::VectorX::Ones(3);

      this->tgtZmpTask_->C() = Eigen::SparseMatrix<double,Eigen::RowMajor>(0,dim);
      this->tgtZmpTask_->dl() = Eigen::VectorXd::Zero(0);
      this->tgtZmpTask_->du() = Eigen::VectorXd::Ones(0);
      this->tgtZmpTask_->wc() = cnoid::VectorX::Ones(0);

      this->tgtZmpTask_->w() = cnoid::VectorX::Ones(dim) * 1e-6;
      this->tgtZmpTask_->toSolve() = false; // 常にtgtZmpが支持領域内にあるなら解く必要がないので高速化のためfalseにする. ない場合があるならtrueにする. calcWrenchでtgtZmpをtruncateしているのでfalseでよい
      this->tgtZmpTask_->settings().verbose = 0;
    }
    {
      // 3. 各脚の各頂点のノルムの重心がCOPOffsetと一致 (fzの値でスケールされてしまうので、alphaを用いて左右をそろえる)

      // 各EndEffectorとtgtZmpの距離を用いてalphaを求める
      std::vector<double> alpha(NUM_LEGS);
      {
        cnoid::Vector3 rleg2leg = EEPose[LLEG].translation() - EEPose[RLEG].translation();
        rleg2leg[2] = 0.0;
        if(rleg2leg.norm() == 0.0){
          alpha[RLEG] = alpha[LLEG] = 0.5;
        }else{
          cnoid::Vector3 rleg2legDir = rleg2leg.normalized();
          double rleg2llegDistance = rleg2leg.norm();
          double rleg2tgtZmpRatio = rleg2legDir.dot(tgtZmp - EEPose[RLEG].translation()) / rleg2llegDistance;
          alpha[RLEG] = mathutil::clamp(1.0 - rleg2tgtZmpRatio, 0.05, 1.0-0.05);
          alpha[LLEG] = mathutil::clamp(rleg2tgtZmpRatio, 0.05, 1.0-0.05);
        }
      }
      Eigen::SparseMatrix<double,Eigen::ColMajor> A_ColMajor(3*NUM_LEGS,dim); // insert()する順序がColMajorなので、RowMajorのAに直接insertすると計算効率が著しく悪い(ミリ秒単位で時間がかかる).
      int idx = 0;
      for(int i=0;i<NUM_LEGS;i++) {
        cnoid::Vector3 cop = EEPose[i].translation() + EEPose[i].linear() * gaitParam.copOffset[i].value();
        for(int j=0;j<gaitParam.legHull[i].size();j++){
          cnoid::Vector3 pos = EEPose[i].translation() + EEPose[i].linear() * gaitParam.legHull[i][j];
          cnoid::Vector3 a = (pos - cop) / alpha[i];
          for(int k=0;k<3;k++) A_ColMajor.insert(i*3+k,idx) = a[k];
          idx ++;
        }
      }
      this->copTask_->A() = A_ColMajor;
      this->copTask_->b() = Eigen::VectorXd::Zero(3*NUM_LEGS);
      this->copTask_->wa() = cnoid::VectorX::Ones(3*NUM_LEGS);

      this->copTask_->C() = Eigen::SparseMatrix<double,Eigen::RowMajor>(0,dim);
      this->copTask_->dl() = Eigen::VectorXd::Zero(0);
      this->copTask_->du() = Eigen::VectorXd::Ones(0);
      this->copTask_->wc() = cnoid::VectorX::Ones(0);

      this->copTask_->w() = cnoid::VectorX::Ones(dim) * 1e-6;
      this->copTask_->toSolve() = true;
      // this->copTask_->options().setToReliable();
      // this->copTask_->options().printLevel = qpOASES::PL_NONE; // PL_HIGH or PL_NONE
      this->copTask_->settings().check_termination = 5; // default 25. 高速化
      this->copTask_->settings().verbose = 0;
    }

    std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks{this->constraintTask_,this->tgtZmpTask_,this->copTask_};
    cnoid::VectorX result;
    if(!prioritized_qp_base::solve(tasks,
                                   result,
                                   0 // debuglevel
                                   )){
      // QP fail. 適当に1/2 tgtForceずつ分配してもよいが、QPがfailするのはだいたい転んでいるときなので、ゼロを入れたほうが安全
      tgtEEWrench[RLEG].setZero();
      tgtEEWrench[LLEG].setZero();
    }else{
      int idx = 0;
      for(int i=0;i<NUM_LEGS;i++){
        tgtEEWrench[i].setZero();
        for(int j=0;j<gaitParam.legHull[i].size();j++){
          tgtEEWrench[i].head<3>() += tgtForce * result[idx];
          tgtEEWrench[i].tail<3>() += (EEPose[i].linear() * gaitParam.legHull[i][j]).cross(tgtForce * result[idx]);
          idx ++;
        }
      }
    }
  }

  // tgtEEWrenchをトルクに直して足し込む
  {
    for(int i=0;i<gaitParam.eeName.size();i++){
      cnoid::JointPath jointPath(actRobotTqc->rootLink(), actRobotTqc->link(gaitParam.eeParentLink[i]));
      cnoid::MatrixXd J = cnoid::MatrixXd::Zero(6,jointPath.numJoints()); // generate frame. endeffector origin
      cnoid::setJacobian<0x3f,0,0,true>(jointPath,actRobotTqc->link(gaitParam.eeParentLink[i]),gaitParam.eeLocalT[i].translation(), // input
					J); // output
      cnoid::VectorX tau = - J.transpose() * tgtEEWrench[i];
      for(int j=0;j<jointPath.numJoints();j++){
	jointPath.joint(j)->u() += tau[j];
      }
    }
  }
    
  o_tgtEEWrench = tgtEEWrench;
  return true;
}

bool Stabilizer::calcTorque(double dt, const GaitParam& gaitParam, bool useActState, cnoid::BodyPtr& actRobotTqc, const cnoid::Vector3& targetCogAcc,
                            std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoPGainPercentage, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoDGainPercentage) const{

  if(!useActState){
    for(int i=0;i<actRobotTqc->numJoints();i++) actRobotTqc->joint(i)->u() = 0.0;
    for(int i=0;i<actRobotTqc->numJoints();i++){
      o_stServoPGainPercentage[i].interpolate(dt);
      o_stServoDGainPercentage[i].interpolate(dt);
    }
  } else {
    cnoid::VectorX tau_g = cnoid::VectorXd::Zero(actRobotTqc->numJoints()); // 重力
    cnoid::VectorX tau_ee = cnoid::VectorXd::Zero(actRobotTqc->numJoints()); // endEffector

    // 速度・加速度を考慮しない重力補償
    {
      actRobotTqc->rootLink()->T() = gaitParam.actRobot->rootLink()->T();
      actRobotTqc->rootLink()->v() = cnoid::Vector3::Zero();
      actRobotTqc->rootLink()->w() = cnoid::Vector3::Zero();
      actRobotTqc->rootLink()->dv() = cnoid::Vector3(0.0,0.0,gaitParam.g);
      actRobotTqc->rootLink()->dw() = cnoid::Vector3::Zero();
      for(int i=0;i<actRobotTqc->numJoints();i++){
	actRobotTqc->joint(i)->q() = gaitParam.actRobot->joint(i)->q();
	actRobotTqc->joint(i)->dq() = 0.0;
	actRobotTqc->joint(i)->ddq() = 0.0;
      }
      actRobotTqc->calcForwardKinematics(true, true);
      cnoid::calcInverseDynamics(actRobotTqc->rootLink()); // actRobotTqc->joint()->u()に書き込まれる
      for(int i=0;i<actRobotTqc->numJoints();i++){
	tau_g[i] = actRobotTqc->joint(i)->u();
	actRobotTqc->joint(i)->u() = 0.0;
      }
    }

    // actRobotTqcのq,dqにactualの値を入れる
    {
      actRobotTqc->rootLink()->T() = gaitParam.actRobot->rootLink()->T();
      actRobotTqc->rootLink()->v() = gaitParam.actRobot->rootLink()->v();
      actRobotTqc->rootLink()->w() = gaitParam.actRobot->rootLink()->w();
      actRobotTqc->rootLink()->dv().setZero();
      actRobotTqc->rootLink()->dw().setZero();
      for(int i=0;i<actRobotTqc->numJoints();i++){
	actRobotTqc->joint(i)->q() = gaitParam.actRobot->joint(i)->q();
	actRobotTqc->joint(i)->dq() = gaitParam.actRobot->joint(i)->dq();
	actRobotTqc->joint(i)->ddq() = 0.0;
	actRobotTqc->joint(i)->u() = 0.0;
      }
      for(int l=0;l<actRobotTqc->numLinks();l++) actRobotTqc->link(l)->F_ext().setZero();
      actRobotTqc->calcForwardKinematics(true,true);
      actRobotTqc->calcCenterOfMass();
    }


    // jointControllableの関節のみ、探索変数にする
    std::vector<cnoid::LinkPtr> variables; variables.reserve(1+actRobotTqc->numJoints());
    variables.push_back(actRobotTqc->rootLink());
    for(size_t i=0;i<actRobotTqc->numJoints();i++){
      if(gaitParam.jointControllable[i]) {
        variables.push_back(actRobotTqc->joint(i));
      }
    }

    std::vector<std::shared_ptr<aik_constraint::IKConstraint> > ikConstraint0;

    // joint limit
    for(size_t i=0;i<actRobotTqc->numJoints();i++){
      if(!gaitParam.jointControllable[i]) continue;
      this->aikJointLimitConstraint[i]->joint() = actRobotTqc->joint(i);
      this->aikJointLimitConstraint[i]->jointLimitTables() = gaitParam.jointLimitTables[i];
      this->aikJointLimitConstraint[i]->pgain() = 400;
      this->aikJointLimitConstraint[i]->dgain() = 100;
      this->aikJointLimitConstraint[i]->maxAccByVelError() = 20.0;
      this->aikJointLimitConstraint[i]->weight() = 0.1;
      ikConstraint0.push_back(this->aikJointLimitConstraint[i]);
    }

    std::vector<std::shared_ptr<aik_constraint::IKConstraint> > ikConstraint1;
    // self Collision TODO

    std::vector<std::shared_ptr<aik_constraint::IKConstraint> > ikConstraint2;
    std::vector<std::shared_ptr<aik_constraint::IKConstraint> > ikConstraint3;

    for(int i=0;i<gaitParam.eeName.size();i++){
      this->aikEEPositionConstraint[i]->A_link() = actRobotTqc->link(gaitParam.eeParentLink[i]);
      this->aikEEPositionConstraint[i]->A_localpos() = gaitParam.eeLocalT[i];
      this->aikEEPositionConstraint[i]->B_link() = nullptr;
      this->aikEEPositionConstraint[i]->eval_link() = actRobotTqc->link(gaitParam.eeParentLink[i]); // local 座標系でerrorやgainを評価
      this->aikEEPositionConstraint[i]->eval_localR() = gaitParam.eeLocalT[i].linear(); // local 座標系でerrorやgainを評価

      if(i < NUM_LEGS && 
         (gaitParam.footstepNodesList[0].isSupportPhase[i] || // 支持脚
          gaitParam.footstepNodesList[0].stopCurrentPosition[i])) { // 早付き
        // 加速させない
        this->aikEEPositionConstraint[i]->pgain().setZero();
        this->aikEEPositionConstraint[i]->B_localvel().setZero();
        this->aikEEPositionConstraint[i]->dgain() = this->ee_support_D[i];
        this->aikEEPositionConstraint[i]->ref_acc().setZero();
        this->aikEEPositionConstraint[i]->weight() = 3.0 * cnoid::Vector6::Ones();
        ikConstraint2.push_back(this->aikEEPositionConstraint[i]);
      }else if(i < NUM_LEGS &&
               gaitParam.isManualControlMode[i].getGoal() == 0.0){ // 遊脚
        if(gaitParam.swingState[i] == GaitParam::DOWN_PHASE){
          this->aikEEPositionConstraint[i]->pgain() = this->ee_landing_K[i];
          this->aikEEPositionConstraint[i]->dgain() = this->ee_landing_D[i];
        }else{
          this->aikEEPositionConstraint[i]->pgain() = this->ee_swing_K[i];
          this->aikEEPositionConstraint[i]->dgain() = this->ee_swing_D[i];
        }
        this->aikEEPositionConstraint[i]->B_localpos() = gaitParam.abcEETargetPose[i];
        this->aikEEPositionConstraint[i]->B_localvel() = gaitParam.abcEETargetVel[i];
        this->aikEEPositionConstraint[i]->ref_acc() = gaitParam.abcEETargetAcc[i];
        this->aikEEPositionConstraint[i]->weight() = 1.0 * cnoid::Vector6::Ones();
        ikConstraint2.push_back(this->aikEEPositionConstraint[i]);
      }else{ // maniulation arm/leg
        this->aikEEPositionConstraint[i]->pgain() = this->ee_K[i];
        this->aikEEPositionConstraint[i]->dgain() = this->ee_D[i];
        this->aikEEPositionConstraint[i]->B_localpos() = gaitParam.abcEETargetPose[i];
        this->aikEEPositionConstraint[i]->B_localvel() = gaitParam.abcEETargetVel[i];
        this->aikEEPositionConstraint[i]->ref_acc() = gaitParam.abcEETargetAcc[i];
        this->aikEEPositionConstraint[i]->weight() = 0.3 * cnoid::Vector6::Ones();
        ikConstraint3.push_back(this->aikEEPositionConstraint[i]); // low prioritiy
      }
    }

    {
      // task: COM to target
      this->aikComConstraint->A_robot() = actRobotTqc;
      this->aikComConstraint->pgain().setZero(); // footguidedで計算された加速をそのまま使う
      this->aikComConstraint->dgain().setZero(); // footguidedで計算された加速をそのまま使う
      this->aikComConstraint->ref_acc() = targetCogAcc; // footguidedで計算された加速をそのまま使う
      this->aikComConstraint->weight() << 3.0, 3.0, 0.3; // 0.1, wn=1e-4だと、wnに負けて不正確になる
      ikConstraint2.push_back(this->aikComConstraint);
    }
    {
      // root
      this->aikRootPositionConstraint->A_link() = actRobotTqc->rootLink();
      this->aikRootPositionConstraint->B_link() = nullptr;
      this->aikRootPositionConstraint->pgain().head<3>().setZero(); // 傾きのみ
      this->aikRootPositionConstraint->pgain().tail<3>() = this->root_K;
      this->aikRootPositionConstraint->dgain().head<3>().setZero(); // 傾きのみ
      this->aikRootPositionConstraint->dgain().tail<3>() = this->root_D;
      this->aikRootPositionConstraint->B_localpos() = gaitParam.refRobot->rootLink()->T();
      this->aikRootPositionConstraint->B_localvel().tail<3>() = gaitParam.refRobot->rootLink()->w();
      this->aikRootPositionConstraint->ref_acc().tail<3>() = gaitParam.refRobot->rootLink()->dw();
      this->aikRootPositionConstraint->eval_link() = actRobotTqc->rootLink(); // local 座標系でerrorやgainを評価
      this->aikRootPositionConstraint->weight().head<3>().setZero(); // 傾きのみ
      this->aikRootPositionConstraint->weight().tail<3>() = 0.3 * cnoid::Vector3::Ones();
      ikConstraint2.push_back(this->aikRootPositionConstraint);
    }

    std::vector<std::shared_ptr<aik_constraint::IKConstraint> > ikConstraint4;
    {
      // task: angular momentum to zero
      this->aikAngularMomentumConstraint->robot() = actRobotTqc;
      this->aikAngularMomentumConstraint->weight() << 0.1, 0.1, 0.1; // yaw旋回歩行するときのために、Zは小さく. rollとpitchも、joint angle taskの方を優先したほうがよい
      ikConstraint4.push_back(this->aikAngularMomentumConstraint);
    }
    {
      // task: joint angle to target
      for(int i=0;i<actRobotTqc->numJoints();i++){
        if(!gaitParam.jointControllable[i]) continue;
        this->aikRefJointAngleConstraint[i]->joint() = actRobotTqc->joint(i);
        this->aikRefJointAngleConstraint[i]->targetq() = gaitParam.refRobot->joint(i)->q();
        this->aikRefJointAngleConstraint[i]->targetdq() = gaitParam.refRobot->joint(i)->dq();
        this->aikRefJointAngleConstraint[i]->ref_acc() = gaitParam.refRobot->joint(i)->ddq();
        this->aikRefJointAngleConstraint[i]->pgain() = this->joint_K[i] * std::pow(this->aikdqWeight[i].value(), 2);
        this->aikRefJointAngleConstraint[i]->maxAccByPosError() = 3.0;
        this->aikRefJointAngleConstraint[i]->dgain() = this->joint_D[i] * this->aikdqWeight[i].value();
        this->aikRefJointAngleConstraint[i]->maxAccByVelError() = 10.0;
        this->aikRefJointAngleConstraint[i]->weight() = 0.3 * this->aikdqWeight[i].value();
        ikConstraint4.push_back(this->aikRefJointAngleConstraint[i]);
      }
    }

    int debugLevel = 0; // 0 or 1
    std::vector<std::vector<std::shared_ptr<aik_constraint::IKConstraint> > > constraints{ikConstraint0,ikConstraint1,ikConstraint2,ikConstraint3,ikConstraint4};
    for(size_t i=0;i<constraints.size();i++){
      for(size_t j=0;j<constraints[i].size();j++){
        constraints[i][j]->debugLevel() = debugLevel;
      }
    }

    prioritized_acc_inverse_kinematics_solver::IKParam param;
    param.debugLevel = debugLevel;
    param.wn = 1e-4;
    param.we = 1e-6;
    bool solved = prioritized_acc_inverse_kinematics_solver::solveAIK(variables,
                                                                      constraints,
                                                                      aikTasks,
                                                                      param);
	
    if(!solved){
      std::cerr << "fail" << std::endl; // TODO
    }else {
      actRobotTqc->calcForwardKinematics(true, true);
      actRobotTqc->calcCenterOfMass();
      cnoid::calcInverseDynamics(actRobotTqc->rootLink()); // actRobotTqc->joint()->u()に書き込まれる
      for(int i=0;i<actRobotTqc->numJoints();i++){
	tau_ee[i] = actRobotTqc->joint(i)->u();
	actRobotTqc->joint(i)->u() = 0.0;
      }
    }

    // 最終的な出力トルクを代入
    for(int i=0;i<actRobotTqc->numJoints();i++){
      actRobotTqc->joint(i)->u() = tau_g[i] +  tau_ee[i];
    }
    
    // std::cerr << "tau_g" << std::endl; 
    // for(int i=0;i<actRobotTqc->numJoints();i++){
    //   std::cerr << tau_g[i] << " ";
    // }
    // std::cerr << std::endl;

    // std::cerr << "tau_ee" << std::endl; 
    // for(int i=0;i<actRobotTqc->numJoints();i++){
    //   std::cerr << tau_ee[i] << " ";
    // }
    // std::cerr << std::endl;

    // std::cerr << "tau" << std::endl; 
    // for(int i=0;i<actRobotTqc->numJoints();i++){
    //   std::cerr << actRobotTqc->joint(i)->u() << " ";
    // }
    // std::cerr << std::endl;
    
    // Gain
    {
      for(int i=0;i<gaitParam.eeName.size();i++){
	cnoid::JointPath jointPath(actRobotTqc->rootLink(), actRobotTqc->link(gaitParam.eeParentLink[i]));
	if((i<NUM_LEGS) && (gaitParam.isManualControlMode[i].getGoal() == 1.0)){ // Manual Control on
	  double transitionTime = std::max(gaitParam.isManualControlMode[i].remain_time(), dt*2); // 現状, setGoal(*,dt)以下の時間でgoal指定するとwriteOutPortDataが破綻するのでテンポラリ
	  for(int j=0;j<jointPath.numJoints();j++){
	    if(o_stServoPGainPercentage[jointPath.joint(j)->jointId()].getGoal() != 100.0) o_stServoPGainPercentage[jointPath.joint(j)->jointId()].setGoal(100.0, transitionTime);
	    if(o_stServoDGainPercentage[jointPath.joint(j)->jointId()].getGoal() != 100.0) o_stServoDGainPercentage[jointPath.joint(j)->jointId()].setGoal(100.0, transitionTime);
	  }
	}else{ // Manual Control off
	  double transitionTime = std::max(0.1, dt*2); // 現状, setGoal(*,dt)以下の時間でgoal指定するとwriteOutPortDataが破綻するのでテンポラリ
	  for(int j=0;j<jointPath.numJoints();j++){
	    if(o_stServoPGainPercentage[jointPath.joint(j)->jointId()].getGoal() != 0.0) o_stServoPGainPercentage[jointPath.joint(j)->jointId()].setGoal(0.0, transitionTime);
	    if(o_stServoDGainPercentage[jointPath.joint(j)->jointId()].getGoal() != 0.0) {
	      o_stServoDGainPercentage[jointPath.joint(j)->jointId()].setGoal(0.0, transitionTime);
	      std::cerr << jointPath.joint(j)->jointId() << std::endl;}
	  }
	}
      }

    }
  
    for(int i=0;i<gaitParam.genRobot->numJoints();i++){
      o_stServoPGainPercentage[i].interpolate(dt);
      o_stServoDGainPercentage[i].interpolate(dt);
    }
  } // useActState
  
  return true;
}

