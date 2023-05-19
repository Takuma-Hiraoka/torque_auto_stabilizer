#ifndef GAITPARAM_H
#define GAITPARAM_H

#include <vector>
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

class GaitParam {
  // pinocchioのdataにおいてnjointsは(floating_base, urdfのjoint)だが、qは(urdfのjoint)なので注意．
  // data portとのやり取り時及びcout時にtableをみて直す。
private:
  // parametor
  std::vector<bool> jointControllable; // 要素数と順序はnjointsと同じ. falseの場合、qやtauはrefの値をそのまま出力する(writeOutputPort時にref値で上書き). 指を位置制御にするため．
  // from data port
  pinocchio::Data refRobotRaw;
  pinocchio::Data actRobotRaw;

  // refToGenFrameConverter
  pinocchio::Data refRobot;
  // actToGenFrameConverter
  pinocchio::Data actRobot;
  
  // Stabillizer
  pinocchio::Data genRobotTqc;
public:
  void init(const pinocchio::Model& model){
    jointControllable.resize(model.njoints-1, true);
    std::cerr << "njoints : " << model.njoints;
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.njoints-1);
    pinocchio::Data data(model);
    refRobotRaw = data;
    pinocchio::forwardKinematics(model,refRobotRaw,q);
    actRobotRaw = data;
    pinocchio::forwardKinematics(model,actRobotRaw,q);
    refRobot = data;
    pinocchio::forwardKinematics(model,refRobot,q);
    actRobot = data;
    pinocchio::forwardKinematics(model,actRobot,q);
    genRobotTqc = data;
    pinocchio::forwardKinematics(model,genRobotTqc,q);
  }
};
#endif
