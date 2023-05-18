#ifndef TorqueAutoStabilizerSERVICESVC_IMPL_H
#define TorqueAutoStabilizerSERVICESVC_IMPL_H

#include "torque_auto_stabilizer_rtc/idl/TorqueAutoStabilizerService.hh"

class TorqueAutoStabilizer;

class TorqueAutoStabilizerService_impl
  : public virtual POA_torque_auto_stabilizer_rtc::TorqueAutoStabilizerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  TorqueAutoStabilizerService_impl();
  ~TorqueAutoStabilizerService_impl();

  CORBA::Boolean torqueAutoStabilizerParam(const CORBA::Double data);
  void setComp(TorqueAutoStabilizer *i_comp);
private:
  TorqueAutoStabilizer *comp_;
};

#endif
