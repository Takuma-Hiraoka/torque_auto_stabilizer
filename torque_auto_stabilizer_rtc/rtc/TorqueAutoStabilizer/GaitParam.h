#ifndef GAITPARAM_H
#define GAITPARAM_H

#include <vector>
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

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
  
private:
  // parametor
  std::vector<bool> jointControllable; // 要素数はnq-7. 順序はurdf準拠．falseの場合、qやtauはrefの値をそのまま出力する(writeOutputPort時にref値で上書き). 指を位置制御にするため．
  // from data port
public:
  Eigen::VectorXd refRobotPos; // 要素数nq.
  Eigen::VectorXd actRobotPos; // 要素数nq.
  Eigen::VectorXd actRobotVel; // 要素数nv.

  // refToGenFrameConverter
  pinocchio::Data refRobot;
  // actToGenFrameConverter
  pinocchio::Data actRobot;
  
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
  }

  void initImu(const std::string& parentLink_, const pinocchio::SE3& localT_){
    imuParentLink = parentLink_;
    imuLocalT = localT_;
  }
};
#endif
