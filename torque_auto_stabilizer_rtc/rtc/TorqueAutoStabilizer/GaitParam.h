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

public:
  // parametor
  std::vector<mathutil::TwoPointInterpolator<Eigen::Vector3d> > copOffset = std::vector<mathutil::TwoPointInterpolator<Eigen::Vector3d> >{mathutil::TwoPointInterpolator<Eigen::Vector3d>(Eigen::Vector3d(0.0,0.02,0.0),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),mathutil::HOFFARBIB),mathutil::TwoPointInterpolator<Eigen::Vector3d>(Eigen::Vector3d(0.0,-0.02,0.0),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),mathutil::HOFFARBIB)}; // 要素数2. rleg: 0. lleg: 1. endeffector frame. 足裏COPの目標位置. 幾何的な位置はcopOffset.value()無しで考えるが、目標COPを考えるときはcopOffset.value()を考慮する. クロスできたりジャンプできたりする脚でないと左右方向(外側向き)の着地位置修正は難しいので、その方向に転びそうになることが極力ないように内側にcopをオフセットさせておくとよい.  単位[m]. 滑らかに変化する
  std::vector<mathutil::TwoPointInterpolator<Eigen::Vector3d> > defaultTranslatePos = std::vector<mathutil::TwoPointInterpolator<Eigen::Vector3d> >(2,mathutil::TwoPointInterpolator<Eigen::Vector3d>(Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),mathutil::HOFFARBIB)); // goPos, goVelocity, setFootSteps等するときの右脚と左脚の中心からの相対位置. また、reference frameとgenerate frameの対応付けに用いられる. (Z軸は鉛直). Z成分はあったほうが計算上扱いやすいからありにしているが、0でなければならない. RefToGenFrameConverter(handFixMode)が「左右」という概念を使うので、X成分も0でなければならない. 単位[m] 滑らかに変化する.
  std::vector<bool> jointControllable; // 要素数はnq-7. 順序はurdf準拠．falseの場合、qやtauはrefの値をそのまま出力する(writeOutputPort時にref値で上書き). 指を位置制御にするため．
  // from data port
public:
  Eigen::VectorXd refRobotPos; // 要素数nq.
  Eigen::VectorXd actRobotPos; // 要素数nq.
  Eigen::VectorXd actRobotVel; // 要素数nv.
  std::vector<Eigen::Vector6d> actFSensorWrenchOrigin; // 要素数と順序はfsenorNameと同じ. fsensor frame.
  std::vector<Eigen::Vector6d> refEEWrenchOrigin; // 要素数と順序はeeNameと同じ.FootOrigin frame. EndEffector origin. ロボットが受ける力
  std::vector<mathutil::TwoPointInterpolatorSE3> refEEPoseRaw; // 要素数と順序はeeNameと同じ. reference world frame. EEPoseはjoint angleなどと比べて遅い周期で届くことが多いので、interpolaterで補間する.

  // refToGenFrameConverter
  pinocchio::Data refRobot;
  std::vector<pinocchio::SE3> refEEPose; // 要素数と順序はeeNameと同じ.generate frame
  std::vector<Eigen::Vector6d> refEEWrench; // 要素数と順序はeeNameと同じ.generate frame. EndEffector origin. ロボットが受けたい力
  double refdz = 1.0; // generate frame. 支持脚からのCogの目標高さ. 0より大きい
  Eigen::Vector3d l = Eigen::Vector3d(0, 0, refdz); // generate frame. FootGuidedControlで外力を計算するときの、ZMP-重心の相対位置に対するオフセット. また、CMPの計算時にDCMに対するオフセット(CMP + l = DCM). 連続的に変化する.
  mathutil::TwoPointInterpolatorSE3 footMidCoords = mathutil::TwoPointInterpolatorSE3(pinocchio::SE3::Identity(),Eigen::Vector6d::Zero(),Eigen::Vector6d::Zero(),mathutil::HOFFARBIB); // generate frame. Z軸は鉛直. 支持脚の位置姿勢(Z軸は鉛直)にdefaultTranslatePosを適用したものの間をつなぐ. interpolatorによって連続的に変化する. reference frameとgenerate frameの対応付けに用いられる

  // actToGenFrameConverter
  pinocchio::Data actRobot;
  mathutil::FirstOrderLowPassFilter<Eigen::Vector3d> actCogVel = mathutil::FirstOrderLowPassFilter<Eigen::Vector3d>(3.5, Eigen::Vector3d::Zero());  // generate frame.  現在のCOM速度. cutoff=4.0Hzは今の歩行時間と比べて遅すぎる気もするが、実際のところ問題なさそう? もとは4Hzだったが、 静止時に衝撃が加わると上下方向に左右交互に振動することがあるので少し小さくする必要がある. 3Hzにすると、追従性が悪くなってギアが飛んだ
  std::vector<pinocchio::SE3> actEEPose; // 要素数と順序はeeNameと同じ.generate frame
  std::vector<Eigen::Vector6d> actFSensorWrench; // 要素数と順序はfsenorNameと同じ.generate frame. この値でインピーダンス制御を行うので、はじめからEE数まではEndEffector origin. それ以降はfsensorParentLink origin．ロボットが受けた力

    // FootStepGenerator
  class FootStepNodes {
  public:
    /*
      各足につきそれぞれ、remainTime 後に dstCoordsに動く.

      footstepNodesList[0]のisSupportPhaseは、変更されない
      footstepNodesList[0]のdstCoordsを変更する場合には、footstepNodesList[0]のremainTimeの小ささに応じて変更量を小さくする. remainTimeがほぼゼロなら変更量もほぼゼロ.
      footstepNodesList[0]は、!footstepNodesList[0].isSupportPhase && footstepNodesList[1].isSupportPhaseの足がある場合に、突然footstepNodesList[1]に遷移する場合がある(footStepGeneratorのearlyTouchDown)
      それ以外には、footstepNodesList[0]のremainTimeが突然0になることはない
      footstepNodesList[1]のisSupportPhaseは、footstepNodesListのサイズが1である場合を除いて変更されない.
      両足支持期の次のstepのisSupportPhaseを変えたり、後ろに新たにfootstepNodesを追加する場合、必ず両足支持期のremainTimeをそれなりに長い時間にする(片足に重心やrefZmpを移す補間の時間のため)

      右脚支持期の直前のnodeのendRefZmpStateはRLEGでなければならない
      右脚支持期のnodeのendRefZmpStateはRLEGでなければならない
      左脚支持期の直前のnodeのendRefZmpStateはLLEGでなければならない
      左脚支持期のnodeのendRefZmpStateはLLEGでなければならない
      両脚支持期の直前のnodeのendRefZmpStateはRLEG or LLEG or MIDDLEでなければならない.
      両脚支持期のnodeのendRefZmpStateはRLEG or LLEG or MIDDLEでなければならない.
      footstepNodesList[0]のendRefZmpStateは変更されない.
      footstepNodesList[0]のendRefZmpStateは、isStatic()である場合を除いて変更されない.
    */
    std::vector<pinocchio::SE3> dstCoords = std::vector<pinocchio::SE3>(NUM_LEGS); // 要素数2. rleg: 0. lleg: 1. generate frame.
    std::vector<bool> isSupportPhase = std::vector<bool>(NUM_LEGS, true); // 要素数2. rleg: 0. lleg: 1. footstepNodesListの末尾の要素が両方falseであることは無い
    double remainTime = 0.0;
    enum class refZmpState_enum{RLEG, LLEG, MIDDLE};
    refZmpState_enum endRefZmpState = refZmpState_enum::MIDDLE; // このnode終了時のrefZmpの位置.

    // 遊脚軌道用パラメータ
    std::vector<std::vector<double> > stepHeight = std::vector<std::vector<double> >(NUM_LEGS,std::vector<double>(2,0)); // 要素数2. rleg: 0. lleg: 1. swing期には、srcCoordsの高さ+[0]とdstCoordsの高さ+[1]の高い方に上げるような軌道を生成する
    std::vector<bool> stopCurrentPosition = std::vector<bool>(NUM_LEGS, false); // 現在の位置で強制的に止めるかどうか
    std::vector<double> goalOffset = std::vector<double>(NUM_LEGS,0.0); // [m]. 遊脚軌道生成時に、遅づきの場合、generate frameで鉛直方向に, 目標着地位置に対して加えるオフセットの最大値. 0以下.
    std::vector<double> touchVel = std::vector<double>(NUM_LEGS,0.3); // 0より大きい. 単位[m/s]. 足を下ろすときの速さ

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  std::vector<FootStepNodes> footStepNodesList = std::vector<FootStepNodes>(1); // 要素数1以上. 0番目が現在の状態. 末尾の要素以降は、末尾の状態がずっと続くとして扱われる.

  // LegCoordsGenerator
  std::vector<pinocchio::SE3> eeTargetPose; // 要素数と順序はeeNameと同じ.generate frame. astで計算された目標位置姿勢

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
    refEEPose.push_back(pinocchio::SE3());
    refEEWrench.push_back(Eigen::Vector6d::Zero());
    refEEWrenchOrigin.push_back(Eigen::Vector6d::Zero());
    refEEPoseRaw.push_back(mathutil::TwoPointInterpolatorSE3(pinocchio::SE3::Identity(), Eigen::Vector6d::Zero(),Eigen::Vector6d::Zero(), mathutil::HOFFARBIB));
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
