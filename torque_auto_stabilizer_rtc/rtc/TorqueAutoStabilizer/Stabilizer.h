#ifndef Stabilizer_H
#define Stabilizer_H

#include "GaitParam.h"
#include <prioritized_qp_osqp/prioritized_qp_osqp.h>
#include <cnoid/JointPath>

#include <aik_constraint/PositionConstraint.h>
#include <aik_constraint/COMConstraint.h>
#include <aik_constraint/JointAngleConstraint.h>
#include <aik_constraint/AngularMomentumConstraint.h>
#include <aik_constraint_joint_limit_table/JointLimitMinMaxTableConstraint.h>
#include <prioritized_acc_inverse_kinematics_solver/PrioritizedAccInverseKinematicsSolver.h>

class Stabilizer{
public:
  // Stabilizerでしか使わないパラメータ
  std::vector<cnoid::Vector6> ee_K; // 要素数EndEffectors. EndEffector frame. endEffector origin. 0以上
  std::vector<cnoid::Vector6> ee_D; // 要素数EndEffectors. EndEffector frame. endEffector origin. 0以上
  std::vector<cnoid::Vector6> ee_swing_K; // 要素数NUM_LEGS. EndEffector frame. endEffector origin. 0以上
  std::vector<cnoid::Vector6> ee_swing_D; // 要素数NUM_LEGS. EndEffector frame. endEffector origin. 0以上
  std::vector<cnoid::Vector6> ee_landing_K; // 要素数NUM_LEGS. EndEffector frame. endEffector origin. 0以上
  std::vector<cnoid::Vector6> ee_landing_D; // 要素数NUM_LEGS. EndEffector frame. endEffector origin. 0以上
  std::vector<cnoid::Vector6> ee_support_D; // 要素数NUM_LEGS. EndEffector frame. endEffector origin. 0以上
  cnoid::Vector3 root_K = cnoid::Vector3(200,200,200); // rootlink frame.  0以上
  cnoid::Vector3 root_D = cnoid::Vector3(50,50,50); // rootlink frame 00以上
  cnoid::VectorXd joint_K; // 0以上
  cnoid::VectorXd joint_D; // 0以上
  std::vector<cpp_filters::TwoPointInterpolator<double> > aikdqWeight; // 要素数と順序はrobot->numJoints()と同じ. 0より大きい. 各関節の速度に対するダンピング項の比. default 1. 動かしたくない関節は大きくする. 全く動かしたくないなら、controllable_jointsを使うこと. resolved acceleration control用

  std::vector<double> torque_limit; // u に入れる関節トルクリミット. 電流リミット*トルク定数*ギア比
  std::vector<double> torque_ratcheting_limit = {220, 450, 1000, 1000, 220, 220, // 右足
                                                 220, 450, 1000, 1000, 220, 220, // 左足
                                                 1000, 1000, 220, 220, 220, // 胴体
                                                 220, 220, 220, 220, 220, 220, 220, 220, // 右腕
                                                 220, 220, 220, 220, 220, 220, 220, 220, // 左腕
                                                 10, 10, 10, 10, 10, 10, // 右指
                                                 10, 10, 10, 10, 10, 10, // 左指
                                                 };

  void init(const GaitParam& gaitParam, cnoid::BodyPtr& actRobotTqc){
    for(int i=0;i<gaitParam.eeName.size();i++){
      ee_K.push_back((cnoid::Vector6() << 30, 30, 30, 20, 20, 20).finished());
      ee_D.push_back((cnoid::Vector6() << 10, 10, 10, 10, 10, 10).finished());
    }
    for(int i=0;i<NUM_LEGS;i++){
      ee_swing_K.push_back((cnoid::Vector6() << 200, 200, 200, 100, 100, 100).finished());
      ee_swing_D.push_back((cnoid::Vector6() << 30, 30, 30, 20, 20, 20).finished());
      ee_landing_K.push_back((cnoid::Vector6() << 200, 200, 20, 100, 100, 100).finished());
      ee_landing_D.push_back((cnoid::Vector6() << 30, 30, 5, 20, 20, 20).finished());
      ee_support_D.push_back((cnoid::Vector6() << 30, 30, 50, 20, 20, 20).finished());
    }

    aikdqWeight.resize(actRobotTqc->numJoints(), cpp_filters::TwoPointInterpolator<double>(1.0, 0.0, 0.0, cpp_filters::HOFFARBIB));
    aikEEPositionConstraint.clear();
    for(int i=0;i<gaitParam.eeName.size();i++) aikEEPositionConstraint.push_back(std::make_shared<aik_constraint::PositionConstraint>());
    aikRefJointAngleConstraint.clear();
    for(int i=0;i<actRobotTqc->numJoints();i++) aikRefJointAngleConstraint.push_back(std::make_shared<aik_constraint::JointAngleConstraint>());
    aikJointLimitConstraint.clear();
    for(int i=0;i<actRobotTqc->numJoints();i++) aikJointLimitConstraint.push_back(std::make_shared<aik_constraint_joint_limit_table::JointLimitMinMaxTableConstraint>());
    
    this->torque_limit.resize(gaitParam.actRobotTqc->numJoints());
    for (int i=0;i<gaitParam.actRobotTqc->numJoints();i++){
      double climit = gaitParam.actRobotTqc->joint(i)->info<double>("climit");
      if(climit >= 200) climit = 10; // 大きすぎるor設定されていないので適当
      double gearRatio = gaitParam.actRobotTqc->joint(i)->info<double>("gearRatio");
      double torqueConst = gaitParam.actRobotTqc->joint(i)->info<double>("torqueConst");
      cnoid::Matrix3 Inertia = gaitParam.actRobotTqc->joint(i)->I();
      cnoid::Vector3 axis = gaitParam.actRobotTqc->joint(i)->jointAxis();
      this->torque_limit[i] = std::max(climit * gearRatio * torqueConst, this->torque_ratcheting_limit[i]);
      //this->ddq_limit[i] = climit * gearRatio * torqueConst / (Inertia * axis).norm();
    }

    this->joint_K = cnoid::VectorXd::Zero(gaitParam.actRobotTqc->numJoints());
    this->joint_D = cnoid::VectorXd::Zero(gaitParam.actRobotTqc->numJoints());
    for (int i=0;i<gaitParam.actRobotTqc->numJoints();i++){
      this->joint_K[i] = 1;
      if((i==12) || (i==13) || (i==14)) this->joint_K[i] = 20; // 腰roll pitch yaw
    }
    for (int i=0;i<gaitParam.actRobotTqc->numJoints();i++){
      this->joint_D[i] = 1;
      if((i==12) || (i==13) || (i==14)) this->joint_D[i] = 10; // 腰roll pitch yaw
    }
  }
protected:
  // 計算高速化のためのキャッシュ. クリアしなくても別に副作用はない.
  // for calcWrench
  mutable std::shared_ptr<prioritized_qp_osqp::Task> eomTask_ = std::make_shared<prioritized_qp_osqp::Task>();
  mutable std::shared_ptr<prioritized_qp_osqp::Task> wrenchConstraintTask_ = std::make_shared<prioritized_qp_osqp::Task>();
  mutable std::shared_ptr<prioritized_qp_osqp::Task> torqueLimitTask_ = std::make_shared<prioritized_qp_osqp::Task>();
  mutable std::shared_ptr<prioritized_qp_osqp::Task> normTask_ = std::make_shared<prioritized_qp_osqp::Task>();
  // for calcTorque
  mutable std::vector<std::shared_ptr<aik_constraint::PositionConstraint> > aikEEPositionConstraint; // 要素数と順序はeeNameと同じ.
  mutable std::vector<std::shared_ptr<aik_constraint::JointAngleConstraint> > aikRefJointAngleConstraint; // 要素数と順序はrobot->numJoints()と同じ
  mutable std::shared_ptr<aik_constraint::PositionConstraint> aikRootPositionConstraint = std::make_shared<aik_constraint::PositionConstraint>();
  mutable std::shared_ptr<aik_constraint::COMConstraint> aikComConstraint = std::make_shared<aik_constraint::COMConstraint>();
  mutable std::shared_ptr<aik_constraint::AngularMomentumConstraint> aikAngularMomentumConstraint = std::make_shared<aik_constraint::AngularMomentumConstraint>();
  mutable std::vector<std::shared_ptr<aik_constraint_joint_limit_table::JointLimitMinMaxTableConstraint> > aikJointLimitConstraint;
  mutable std::vector<std::shared_ptr<prioritized_qp_base::Task> > aikTasks;

public:
  void initStabilizerOutput(const GaitParam& gaitParam,
                            cnoid::Vector3& o_stTargetZmp, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoPGainPercentage, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoDGainPercentage) const;

  bool calcResolvedAccelationControl(const GaitParam& gaitParam, double dt, bool useActState, cnoid::BodyPtr& actRobotTqc, 
				     cnoid::Vector3& o_stTargetZmp, std::vector<cnoid::Vector6>& o_stEETargetWrench,
				     std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoPGainPercentage, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoDGainPercentage) const;

protected:
  bool calcZMP(const GaitParam& gaitParam, double dt, bool useActState,
               cnoid::Vector3& o_tgtZmp/*generate座標系*/, cnoid::Vector3& o_tgtForce/*generate座標系*/, cnoid::Vector3& o_tgtCogAcc /*generate座標系*/) const;
  bool calcWrench(const GaitParam& gaitParam, cnoid::BodyPtr& actRobotTqc, 
                  std::vector<cnoid::Vector6>& o_tgtEEWrench /* 要素数EndEffector数. generate座標系. EndEffector origin*/) const;
  bool calcTorque(double dt, const GaitParam& gaitParam, bool useActState, cnoid::BodyPtr& actRobotTqc, const cnoid::Vector3& targetCogAcc, 
                  std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoPGainPercentage, std::vector<cpp_filters::TwoPointInterpolator<double> >& o_stServoDGainPercentage) const;

};

#endif
