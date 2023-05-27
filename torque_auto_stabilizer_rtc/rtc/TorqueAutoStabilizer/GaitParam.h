#ifndef GAITPARAM_H
#define GAITPARAM_H

#include <vector>
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "MathUtil.h"

enum leg_enum{RLEG=0, LLEG=1, NUM_LEGS=2};

namespace Eigen{
typedef Eigen::Matrix<double, 6, 1> Vector6d;
}

class GaitParam {
  // pinocchioではqは(base position, base quaternion, joints),vやaも(base pos, base rot, joints)
  // data portとのやり取り時及びcout時にtableをみて直す。
public:
    // constant parameter
  std::vector<std::string> eeName; // constant. 要素数2以上. 0番目がrleg, 1番目がllegという名前である必要がある
  std::vector<std::string> eeParentLink; // constant. 要素数と順序はeeNameと同じ. 
  std::vector<pinocchio::SE3> eeLocalT; // constant. 要素数と順序はeeNameと同じ. Parent Link Frame

  std::string imuParentLink; // constant
  pinocchio::SE3 imuLocalT; // constant Parent Link Frame

  std::vector<std::string> fsensorName; // constant. 要素数2以上. 0番目がrleg, 1番目がllegという名前である必要がある
  std::vector<std::string> fsensorParentLink; // constant. 要素数と順序はfsensorNameと同じ. 
  std::vector<pinocchio::SE3> fsensorLocalT; // constant. 要素数と順序はfsenorNameと同じ. Parent Link Frame

private:
  // parametor
  std::vector<bool> jointControllable; // 要素数はnq-7. 順序はurdf準拠．falseの場合、qやtauはrefの値をそのまま出力する(writeOutputPort時にref値で上書き). 指を位置制御にするため．
  // from data port
public:
  Eigen::VectorXd refRobotPos; // 要素数nq.
  Eigen::VectorXd actRobotPos; // 要素数nq.
  Eigen::VectorXd actRobotVel; // 要素数nv.
  std::vector<Eigen::Vector6d> actFSensorWrenchOrigin; // 要素数と順序はfsenorNameと同じ. fsensor frame.

  // refToGenFrameConverter
  pinocchio::Data refRobot;
  // actToGenFrameConverter
  pinocchio::Data actRobot;
  mathutil::FirstOrderLowPassFilter<Eigen::Vector3d> actCogVel = mathutil::FirstOrderLowPassFilter<Eigen::Vector3d>(3.5, Eigen::Vector3d::Zero());  // generate frame.  現在のCOM速度. cutoff=4.0Hzは今の歩行時間と比べて遅すぎる気もするが、実際のところ問題なさそう? もとは4Hzだったが、 静止時に衝撃が加わると上下方向に左右交互に振動することがあるので少し小さくする必要がある. 3Hzにすると、追従性が悪くなってギアが飛んだ
  std::vector<pinocchio::SE3> actEEPose; // 要素数と順序はeeNameと同じ.generate frame
  std::vector<Eigen::Vector6d> actFSensorWrench; // 要素数と順序はfsenorNameと同じ.generate frame. この値でインピーダンス制御を行うので、はじめからEE数まではEndEffector origin. それ以降はfsensorParentLink origin．ロボットが受けた力

  // LegCoordsGenerator
  std::vector<pinocchio::SE3> eeTargetPose; // 要素数と順序はeeNameと同じ.generate frame. abcで計算された目標位置姿勢

  // Stabillizer
  pinocchio::Data genRobotTqc;
public:
  void init(const pinocchio::Model& model){
    jointControllable.resize(model.nq-7, true);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    std::cerr << "model.njoints : " << model.njoints << std::endl;
    std::cerr << "model.nq : " << model.nq << std::endl;
    std::cerr << "model.nv : " << model.nv << std::endl;
    pinocchio::Data data(model);
    refRobotPos = Eigen::VectorXd::Zero(model.nq);
    actRobotPos  = Eigen::VectorXd::Zero(model.nq);
    actRobotVel  = Eigen::VectorXd::Zero(model.nv);
    refRobot = data;
    pinocchio::forwardKinematics(model,refRobot,q);
    actRobot = data;
    pinocchio::forwardKinematics(model,actRobot,q);
    genRobotTqc = data;
    pinocchio::forwardKinematics(model,genRobotTqc,q);
  }

  void push_backEE(const std::string& name_, const std::string& parentLink_, const pinocchio::SE3& localT_){
    eeName.push_back(name_);
    eeParentLink.push_back(parentLink_);
    eeLocalT.push_back(localT_);
    actEEPose.push_back(pinocchio::SE3());
    eeTargetPose.push_back(pinocchio::SE3());
  }

  void initImu(const std::string& parentLink_, const pinocchio::SE3& localT_){
    imuParentLink = parentLink_;
    imuLocalT = localT_;
  }

  void push_backFSensor(const std::string& name_, const std::string& parentLink_, const pinocchio::SE3& localT_){
    fsensorName.push_back(name_);
    fsensorParentLink.push_back(parentLink_);
    fsensorLocalT.push_back(localT_);
    actFSensorWrench.push_back(Eigen::Vector6d::Zero());
    actFSensorWrenchOrigin.push_back(Eigen::Vector6d::Zero());
  }
};
#endif
