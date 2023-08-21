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
  // - 目標位置姿勢を満たすように分解加速度制御. 目標加速度を計算
  // - 目標のZMPと接触レンチを満たすように目標足裏反力を計算、仮想仕事の原理で足し込む

  // - 現在のactual重心位置から、目標ZMPを計算
  cnoid::Vector3 tgtForce; // generate frame
  cnoid::Vector3 tgtCogAcc; // generate frame
  this->calcZMP(gaitParam, dt, useActState, // input
                o_stTargetZmp, tgtForce, tgtCogAcc); // output

  // 目標位置姿勢を満たすような目標加速度を計算．actRobotTqcに書き込まれる
  this->calcTorque(dt, gaitParam, useActState, actRobotTqc, tgtCogAcc,
		   o_stServoPGainPercentage, o_stServoDGainPercentage);

  if(useActState)
    {
      // 目標ZMPを満たすように目標EndEffector反力を計算
      this->calcWrench(gaitParam,// input
		       actRobotTqc, o_stEETargetWrench); // output
    }

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

bool Stabilizer::calcWrench(const GaitParam& gaitParam, cnoid::BodyPtr& actRobotTqc, 
                            std::vector<cnoid::Vector6>& o_tgtEEWrench) const{
  std::vector<cnoid::Vector6> tgtEEWrench(gaitParam.eeName.size(), cnoid::Vector6::Zero()); /* 要素数EndEffector数. generate frame. EndEffector origin*/

  for(int i = 0;i<gaitParam.eeName.size();i++){
    tgtEEWrench[i] = gaitParam.refEEWrench[i];
  }

  actRobotTqc->rootLink()->dv() += cnoid::Vector3(0.0,0.0,gaitParam.g);
  actRobotTqc->calcForwardKinematics(true, true);
  actRobotTqc->calcCenterOfMass();

  cnoid::Vector6 tgtSupWrench = cnoid::Vector6::Zero(); // ルートリンクが支持脚から受ける必要がある外力. generate frame. cog origin.

  cnoid::Vector6 tgtSupWrench_o = cnoid::calcInverseDynamics(actRobotTqc->rootLink()); // actRobotTqc->joint()->u()に書き込まれる

  tgtSupWrench.head<3>() = tgtSupWrench_o.head<3>();
  tgtSupWrench.tail<3>() = tgtSupWrench_o.tail<3>();
  tgtSupWrench.tail<3>() += (- actRobotTqc->centerOfMass()).cross(tgtSupWrench_o.head<3>());

  // std::cerr << "tau" << std::endl; 
  // for(int i=0;i<actRobotTqc->numJoints();i++){
  //   std::cerr << actRobotTqc->joint(i)->u() << " ";
  // }
  // std::cerr << std::endl;

  std::vector<int> supportEE;
  if(gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[0].isSupportPhase[LLEG]){
    supportEE = {RLEG};
  }else if(!gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && gaitParam.footstepNodesList[0].isSupportPhase[LLEG]){
    supportEE = {LLEG};
  }else if(!gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[0].isSupportPhase[LLEG]){
    // 滞空期
  }else{
    supportEE = {RLEG, LLEG};
  }


  if(supportEE.size()>0){
    /*
      legは、legから受けるwrenchの和がtgtSupWrenchを満たすように.
      各EEFのwrenchを、EEF+copOffset frame/originの6軸表現で考える.

      階層QPのタスクは次の通り
      1. 接触力制約
      2. 和がtgtSupWrench (rot).
      3. 和がtgtSupWrench (trans)
      4. ノルムの2乗和の最小化 (fzは大きくて良い.)

      rotがtransより下の優先度になることにより、擬似的なhip strategyが実現されるようにも思える. しかし、重心位置はFootGuidedControlやmodifyFootStepsで制御できるが、身体の回転を制御する手段は乏しいので、回転しすぎて破綻しないようにrotを優先して満たしたほうが良い
    */

    const int dim = 6 * supportEE.size();
    {
      // 1. 接触力制約
      // 0 <  0  0  1  0  0  0 < 1e10
      // 0 <  1  0 mt  0  0  0 < 1e10
      // 0 < -1  0 mt  0  0  0 < 1e10
      // 0 <  0  1 mt  0  0  0 < 1e10
      // 0 <  0 -1 mt  0  0  0 < 1e10
      // 0 <  0  0  d r1 r2  0 < 1e10 ;; x legHull.size()
      // 0 <  0  0 mr  0  0  1 < 1e10
      // 0 <  0  0 mr  0  0 -1 < 1e10

      this->constraintTask_->A() = Eigen::SparseMatrix<double,Eigen::RowMajor>(0,dim);
      this->constraintTask_->b() = Eigen::VectorXd::Zero(0);
      this->constraintTask_->wa() = cnoid::VectorX::Ones(0);

      int constraintDim = 0;
      for(int i=0;i<supportEE.size();i++) constraintDim += 7+gaitParam.legHull[supportEE[i]].size();
      this->constraintTask_->C() = Eigen::SparseMatrix<double,Eigen::RowMajor>(constraintDim,dim);
      this->constraintTask_->dl() = Eigen::VectorXd::Zero(constraintDim);
      this->constraintTask_->du() = 1e10 * Eigen::VectorXd::Ones(constraintDim);
      this->constraintTask_->wc() = cnoid::VectorX::Ones(constraintDim);
      for(int i=0, idx=0;i<supportEE.size();i++){
        int leg = supportEE[i];
        this->constraintTask_->C().insert(idx,i*6+2) = 1.0; this->constraintTask_->dl()[idx] = 50.0; idx++;
        this->constraintTask_->C().insert(idx,i*6+0) = 1.0; this->constraintTask_->C().insert(idx,i*6+2) = gaitParam.muTrans[leg]; idx++;
        this->constraintTask_->C().insert(idx,i*6+0) = -1.0; this->constraintTask_->C().insert(idx,i*6+2) = gaitParam.muTrans[leg]; idx++;
        this->constraintTask_->C().insert(idx,i*6+1) = 1.0; this->constraintTask_->C().insert(idx,i*6+2) = gaitParam.muTrans[leg]; idx++;
        this->constraintTask_->C().insert(idx,i*6+1) = -1.0; this->constraintTask_->C().insert(idx,i*6+2) = gaitParam.muTrans[leg]; idx++;
        for(int j=0;j<gaitParam.legHull[leg].size();j++){
          cnoid::Vector3 v1 = gaitParam.legHull[leg][j] - gaitParam.copOffset[leg].value(); // EEF+copOffset frame/origin
          cnoid::Vector3 v2 = gaitParam.legHull[leg][(j+1<gaitParam.legHull[leg].size())?j+1:0] - gaitParam.copOffset[leg].value(); // EEF+copOffset frame/origin
          if(v1.head<2>() == v2.head<2>()) continue;
          cnoid::Vector3 r = cnoid::Vector3(v2[1]-v1[1],v1[0]-v2[0],0).normalized();
          double d = r.dot(v1);
          this->constraintTask_->C().insert(idx,i*6+2) = d; this->constraintTask_->C().insert(idx,i*6+3) = -r[1]; this->constraintTask_->C().insert(idx,i*6+4) = r[0]; idx++;
        }
        this->constraintTask_->C().insert(idx,i*6+5) = 1.0; this->constraintTask_->C().insert(idx,2) = gaitParam.muRot[leg]; idx++;
        this->constraintTask_->C().insert(idx,i*6+5) = -1.0; this->constraintTask_->C().insert(idx,2) = gaitParam.muRot[leg]; idx++;
      }

      this->constraintTask_->w() = cnoid::VectorX::Ones(dim) * 1e-6;
      this->constraintTask_->toSolve() = false;
      this->constraintTask_->settings().verbose = 0;
    }

    {
      // 2. 和がtgtSupWrench (rot)
      Eigen::SparseMatrix<double,Eigen::ColMajor> A_colMajor(3,dim); // insertする順番がcolMajorなので
      for(int i=0;i<supportEE.size();i++){
        int leg = supportEE[i];
        cnoid::Position eePose = gaitParam.actEEPose[leg]; eePose.translation() += eePose.linear() * gaitParam.copOffset[leg].value();
        cnoid::Matrix3 eeR = eePose.linear();
        cnoid::Matrix3 eepCross = mathutil::cross(eePose.translation() - actRobotTqc->centerOfMass()) * eeR;
        for(int j=0;j<3;j++) {
          for(int k=0;k<3;k++) A_colMajor.insert(k,i*6+j) = eepCross(k,j);
        }
        for(int j=0;j<3;j++) {
          for(int k=0;k<3;k++) A_colMajor.insert(k,i*6+3+j) = eeR(k,j);
        }
      }
      this->tgtTorqueTask_->A() = A_colMajor;
      this->tgtTorqueTask_->b() = tgtSupWrench.tail<3>();
      this->tgtTorqueTask_->wa() = cnoid::VectorX::Ones(3);

      this->tgtTorqueTask_->C() = Eigen::SparseMatrix<double,Eigen::RowMajor>(0,dim);
      this->tgtTorqueTask_->dl() = Eigen::VectorXd::Zero(0);
      this->tgtTorqueTask_->du() = Eigen::VectorXd::Ones(0);
      this->tgtTorqueTask_->wc() = cnoid::VectorX::Ones(0);

      this->tgtTorqueTask_->w() = cnoid::VectorX::Ones(dim) * 1e-6;
      this->tgtTorqueTask_->toSolve() = true;
      this->tgtTorqueTask_->settings().check_termination = 5; // default 25. 高速化
      this->tgtTorqueTask_->settings().verbose = 0;
    }

    {
      // 3. 和がtgtSupWrench (trans)
      Eigen::SparseMatrix<double,Eigen::ColMajor> A_colMajor(3,dim); // insertする順番がcolMajorなので
      for(int i=0;i<supportEE.size();i++){
        int leg = supportEE[i];
        cnoid::Matrix3 eeR = gaitParam.actEEPose[leg].linear();
        for(int j=0;j<3;j++) {
          for(int k=0;k<3;k++) A_colMajor.insert(k,i*6+j) = eeR(k,j);
        }
      }
      this->tgtForceTask_->A() = A_colMajor;
      this->tgtForceTask_->b() = tgtSupWrench.head<3>();
      this->tgtForceTask_->wa() = cnoid::VectorX::Ones(3);

      this->tgtForceTask_->C() = Eigen::SparseMatrix<double,Eigen::RowMajor>(0,dim);
      this->tgtForceTask_->dl() = Eigen::VectorXd::Zero(0);
      this->tgtForceTask_->du() = Eigen::VectorXd::Ones(0);
      this->tgtForceTask_->wc() = cnoid::VectorX::Ones(0);

      this->tgtForceTask_->w() = cnoid::VectorX::Ones(dim) * 1e-6;
      this->tgtForceTask_->toSolve() = true;
      this->tgtForceTask_->settings().check_termination = 5; // default 25. 高速化
      this->tgtForceTask_->settings().verbose = 0;
    }

    {
      // 4. ノルムの2乗和の最小化
      this->normTask_->A() = Eigen::SparseMatrix<double,Eigen::RowMajor>(0,dim);
      this->normTask_->b() = Eigen::VectorXd::Zero(0);
      this->normTask_->wa() = cnoid::VectorX::Ones(0);

      this->normTask_->C() = Eigen::SparseMatrix<double,Eigen::RowMajor>(0,dim);
      this->normTask_->dl() = Eigen::VectorXd::Zero(0);
      this->normTask_->du() = Eigen::VectorXd::Ones(0);
      this->normTask_->wc() = cnoid::VectorX::Ones(0);

      this->normTask_->w() = cnoid::VectorX::Ones(dim);
      for(int i=0;i<supportEE.size();i++){
        int leg = supportEE[i];
        this->normTask_->w()[i*6+0] = std::pow(1e2, 2.0);
        this->normTask_->w()[i*6+1] = std::pow(1e2, 2.0);
        this->normTask_->w()[i*6+2] = std::pow(1e0, 2.0);
        this->normTask_->w()[i*6+3] = std::pow(1e2, 2.0);
        this->normTask_->w()[i*6+4] = std::pow(1e2, 2.0);
        this->normTask_->w()[i*6+5] = std::pow(1e3, 2.0);
      }

      this->normTask_->toSolve() = true;
      this->normTask_->settings().check_termination = 5; // default 25. 高速化
      this->normTask_->settings().verbose = 0;
    }

    std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks{this->constraintTask_,this->tgtTorqueTask_,this->tgtForceTask_,this->normTask_};
    cnoid::VectorX result; // EEF+copOffset frame/origin
    if(prioritized_qp_base::solve(tasks,
                                   result,
                                   0 // debuglevel
                                   )){
      for(int i=0;i<supportEE.size();i++){
        int leg = supportEE[i];
        cnoid::Vector6 w = result.segment<6>(i*6); // EEF+copOffset frame/origin
        tgtEEWrench[leg].head<3>() += gaitParam.actEEPose[leg].linear() * w.head<3>();
        tgtEEWrench[leg].tail<3>() += gaitParam.actEEPose[leg].linear() * w.tail<3>();
        tgtEEWrench[leg].tail<3>() += (gaitParam.actEEPose[leg].linear() * gaitParam.copOffset[leg].value()).cross(gaitParam.actEEPose[leg].linear() * w.head<3>());
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
      this->aikComConstraint->weight() << 3.0, 3.0, 1.0; // 0.1, wn=1e-4だと、wnに負けて不正確になる
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
      std::cerr << "fail" << std::endl;
      // 重力補償だけする
      actRobotTqc->rootLink()->T() = gaitParam.actRobot->rootLink()->T();
      actRobotTqc->rootLink()->v() = cnoid::Vector3::Zero();
      actRobotTqc->rootLink()->w() = cnoid::Vector3::Zero();
      actRobotTqc->rootLink()->dv() = cnoid::Vector3::Zero();
      actRobotTqc->rootLink()->dw() = cnoid::Vector3::Zero();
      for(int i=0;i<actRobotTqc->numJoints();i++){
	actRobotTqc->joint(i)->q() = gaitParam.actRobot->joint(i)->q();
	actRobotTqc->joint(i)->dq() = 0.0;
	actRobotTqc->joint(i)->ddq() = 0.0;
      }
    }
    
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

