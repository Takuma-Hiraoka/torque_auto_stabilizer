#ifndef TorqueAutoStabilizer_H
#define TorqueAutoStabilizer_H

#include <mutex>
#include <vector>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>
#include <rtm/CorbaPort.h>
#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>

#include <torque_auto_stabilizer_msgs/idl/TorqueAutoStabilizer.hh>

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "TorqueAutoStabilizerService_impl.h"
#include "GaitParam.h"

class TorqueAutoStabilizer : public RTC::DataFlowComponentBase{
protected:

  RTC::TimedDoubleSeq m_qRef_;
  RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn_;
  RTC::TimedDoubleSeq m_tauRef_;
  RTC::InPort<RTC::TimedDoubleSeq> m_tauRefIn_;
  RTC::TimedPoint3D m_refBasePos_; // Reference World frame
  RTC::InPort<RTC::TimedPoint3D> m_refBasePosIn_;
  RTC::TimedOrientation3D m_refBaseRpy_; // Reference World frame
  RTC::InPort<RTC::TimedOrientation3D> m_refBaseRpyIn_;
  RTC::TimedDoubleSeq m_qAct_;
  RTC::InPort<RTC::TimedDoubleSeq> m_qActIn_;
  RTC::TimedDoubleSeq m_dqAct_;
  RTC::InPort<RTC::TimedDoubleSeq> m_dqActIn_;
  RTC::TimedDoubleSeq m_tauAct_;
  RTC::TimedOrientation3D m_actImu_; // Actual Imu World Frame. 
  RTC::InPort<RTC::TimedOrientation3D> m_actImuIn_;
  RTC::InPort<RTC::TimedDoubleSeq> m_tauActIn_;
  RTC::TimedDoubleSeq m_q_;
  RTC::OutPort<RTC::TimedDoubleSeq> m_qOut_;
  RTC::TimedDoubleSeq m_tau_;
  RTC::OutPort<RTC::TimedDoubleSeq> m_tauOut_;

  TorqueAutoStabilizerService_impl m_service0_;
  RTC::CorbaPort m_torqueAutoStabilizerServicePort_;

public:
  TorqueAutoStabilizer(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  bool startAutoStabilizer();
  bool stopAutoStabilizer();
  bool torqueAutoStabilizerParam(const double data);

private:
  pinocchio::Model model_;
  std::vector<int> joint_id_table_;
  GaitParam gaitParam_;
  
protected:
  double dt_;
  std::mutex mutex_;
  bool getProperty(const std::string& key, std::string& ret);
  bool readInPortData(const GaitParam& gaitParam, const pinocchio::Model& model, Eigen::VectorXd& refRobotPos, Eigen::VectorXd& actRobotPos, Eigen::VectorXd& actRobotVel);
  static bool execAutoStabilizer(GaitParam& gaitParam, double dt);
  bool writeOutPortData(const GaitParam & gaitParam);
  class ControlMode{
  public:
    /*
      MODE_IDLE -> startAutoStabilizer() -> MODE_SYNC_TO_AST -> MODE_AST -> stopAutoStabilizer() -> MODE_SYNC_TO_IDLE -> MODE_IDLE
      MODE_SYNC_TO*の時間はtransition_timeの時間をかけて遷移するが、少なくとも1周期はMODE_SYNC_TO*を経由する.
      stopAutoStabilizerはとりあえず考慮しない．
      MODE_SYNC_TO*では、基本的に次のMODEと同じ処理が行われるが、出力時に前回のMODEの出力から補間するような軌道に加工されることで出力の連続性を確保する
      補間している途中で別のmodeに切り替わることは無いので、そこは安心してプログラムを書いてよい(例外はonActivated). 同様に、remainTimeが突然減ったり増えたりすることもない
     */
    enum Mode_enum{ MODE_IDLE, MODE_SYNC_TO_AST, MODE_AST, MODE_SYNC_TO_IDLE};
    enum Transition_enum{ START_AST, STOP_AST};
    double ast_start_transition_time, ast_stop_transition_time;
  private:
    Mode_enum current, previous, next;
    double remain_time;
  public:
    ControlMode(){ reset(); ast_start_transition_time = 0.2; ast_stop_transition_time = 2.0;}
    void reset(){ current = previous = next = MODE_IDLE; remain_time = 0;}
    bool setNextTransition(const Transition_enum request){
      switch(request){
      case START_AST:
        if(current == MODE_IDLE){ next = MODE_SYNC_TO_AST; return true; }else{ return false; }
      case STOP_AST:
        if(current == MODE_AST){ next = MODE_SYNC_TO_IDLE; return true; }else{ return false; }
      default:
        return false;
      }
    }
    void update(double dt){
      if(current != next) {
        previous = current; current = next;
        switch(current){
        case MODE_SYNC_TO_AST:
          remain_time = ast_start_transition_time; break;
        case MODE_SYNC_TO_IDLE:
          remain_time = ast_stop_transition_time; break;
        default:
          break;
        }
      }else{
        previous = current;
        remain_time -= dt;
        if(remain_time <= 0.0){
          remain_time = 0.0;
          switch(current){
          case MODE_SYNC_TO_AST:
            current = next = MODE_AST; break;
          case MODE_SYNC_TO_IDLE:
            current = next = MODE_IDLE; break;
          default:
            break;
          }
        }
      }
    }
    double remainTime() const{ return remain_time;}
    Mode_enum now() const{ return current; }
    Mode_enum pre() const{ return previous; }
    bool isASTRunning() const{ return (current==MODE_SYNC_TO_AST) || (current==MODE_AST);}
    bool isSyncToAST() const{ return current==MODE_SYNC_TO_AST;}
    bool isSyncToASTInit() const{ return (current != previous) && isSyncToAST();}
    bool isSyncToIdle() const{ return current==MODE_SYNC_TO_IDLE;}
    bool isSyncToIdleInit() const{ return (current != previous) && isSyncToIdle();}
  };
  ControlMode mode_;
};

extern "C"
{
  void TorqueAutoStabilizerInit(RTC::Manager* manager);
}

#endif // TorqueAutoStabilizer_H
