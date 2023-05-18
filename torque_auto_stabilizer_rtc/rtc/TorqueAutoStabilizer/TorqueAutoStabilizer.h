#ifndef TorqueAutoStabilizer_H
#define TorqueAutoStabilizer_H

#include <mutex>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>
#include <rtm/CorbaPort.h>
#include <rtm/idl/BasicDataType.hh>

#include <torque_auto_stabilizer_msgs/idl/TorqueAutoStabilizer.hh>

#include "TorqueAutoStabilizerService_impl.h"

class TorqueAutoStabilizer : public RTC::DataFlowComponentBase{
protected:

  RTC::TimedDoubleSeq m_qRef_;
  RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn_;
  RTC::TimedDoubleSeq m_tauRef_;
  RTC::InPort<RTC::TimedDoubleSeq> m_tauRefIn_;
  RTC::TimedDoubleSeq m_qAct_;
  RTC::InPort<RTC::TimedDoubleSeq> m_qActIn_;
  RTC::TimedDoubleSeq m_dqAct_;
  RTC::InPort<RTC::TimedDoubleSeq> m_dqActIn_;
  RTC::TimedDoubleSeq m_tauAct_;
  RTC::InPort<RTC::TimedDoubleSeq> m_tauActIn_;
  RTC::TimedDoubleSeq m_q_;
  RTC::OutPort<RTC::TimedDoubleSeq> m_qOut_;
  RTC::TimedDoubleSeq m_tau_;
  RTC::OutPort<RTC::TimedDoubleSeq> m_tauOut_;

  TorqueAutoStabilizerService_impl m_service0_;
  RTC::CorbaPort m_torqueAutoStabilizerServicePort_;

  std::mutex mutex_;
public:
  TorqueAutoStabilizer(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  bool torqueAutoStabilizerParam(const double data);

private:
};

extern "C"
{
  void TorqueAutoStabilizerInit(RTC::Manager* manager);
}

#endif // TorqueAutoStabilizer_H
