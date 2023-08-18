// -*-C++-*-
#ifndef TorqueAutoStabilizerSERVICESVC_IMPL_H
#define TorqueAutoStabilizerSERVICESVC_IMPL_H

#include "torque_auto_stabilizer_rtc/idl/TorqueAutoStabilizerService.hh"

class TorqueAutoStabilizer;

class TorqueAutoStabilizerService_impl
  : public virtual POA_OpenHRP::TorqueAutoStabilizerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  TorqueAutoStabilizerService_impl();// 実装は.cppファイルの方に書かないと、registerProvider時にSegmentation Faultになる
  ~TorqueAutoStabilizerService_impl();
  CORBA::Boolean goPos( CORBA::Double x,  CORBA::Double y,  CORBA::Double th);
  CORBA::Boolean goVelocity( CORBA::Double vx,  CORBA::Double vy,  CORBA::Double vth);
  CORBA::Boolean goStop();
  CORBA::Boolean jumpTo( CORBA::Double x,  CORBA::Double y,  CORBA::Double z,  CORBA::Double ts,  CORBA::Double tf);
  CORBA::Boolean setFootSteps(const OpenHRP::TorqueAutoStabilizerService::FootstepSequence& fs);
  CORBA::Boolean setFootStepsWithParam(const OpenHRP::TorqueAutoStabilizerService::FootstepSequence& fs, const OpenHRP::TorqueAutoStabilizerService::StepParamSequence& spss);
  void waitFootSteps();
  CORBA::Boolean startAutoBalancer();
  CORBA::Boolean stopAutoBalancer();
  CORBA::Boolean startStabilizer(void);
  CORBA::Boolean stopStabilizer(void);
  CORBA::Boolean setTorqueAutoStabilizerParam(const OpenHRP::TorqueAutoStabilizerService::TorqueAutoStabilizerParam& i_param);
  CORBA::Boolean getTorqueAutoStabilizerParam(OpenHRP::TorqueAutoStabilizerService::TorqueAutoStabilizerParam_out i_param);
  CORBA::Boolean getFootStepState(OpenHRP::TorqueAutoStabilizerService::FootStepState_out i_param);
  CORBA::Boolean releaseEmergencyStop();
  CORBA::Boolean startImpedanceController(const char *i_name_);
  CORBA::Boolean stopImpedanceController(const char *i_name_);
  CORBA::Boolean startWholeBodyMasterSlave();
  CORBA::Boolean stopWholeBodyMasterSlave();
  //
  //
  void setComp(TorqueAutoStabilizer *i_comp);
private:
  TorqueAutoStabilizer *comp_;
};

#endif
