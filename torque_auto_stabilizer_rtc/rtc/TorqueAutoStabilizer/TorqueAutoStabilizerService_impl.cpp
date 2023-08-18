#include "TorqueAutoStabilizerService_impl.h"
#include "TorqueAutoStabilizer.h"

TorqueAutoStabilizerService_impl::TorqueAutoStabilizerService_impl()
{
}

TorqueAutoStabilizerService_impl::~TorqueAutoStabilizerService_impl()
{
}

CORBA::Boolean TorqueAutoStabilizerService_impl::goPos( CORBA::Double x,  CORBA::Double y,  CORBA::Double th)
{
  return this->comp_->goPos(x, y, th);
};

CORBA::Boolean TorqueAutoStabilizerService_impl::goVelocity( CORBA::Double vx,  CORBA::Double vy,  CORBA::Double vth)
{
  return this->comp_->goVelocity(vx, vy, vth);
};

CORBA::Boolean TorqueAutoStabilizerService_impl::goStop()
{
  return this->comp_->goStop();
};

CORBA::Boolean TorqueAutoStabilizerService_impl::jumpTo( CORBA::Double x,  CORBA::Double y,  CORBA::Double z,  CORBA::Double ts,  CORBA::Double tf)
{
  return this->comp_->jumpTo(x, y, z, ts, tf);
};

CORBA::Boolean TorqueAutoStabilizerService_impl::setFootSteps(const OpenHRP::TorqueAutoStabilizerService::FootstepSequence& fs)
{
  return this->comp_->setFootSteps(fs);
}

CORBA::Boolean TorqueAutoStabilizerService_impl::setFootStepsWithParam(const OpenHRP::TorqueAutoStabilizerService::FootstepSequence& fs, const OpenHRP::TorqueAutoStabilizerService::StepParamSequence& spss)
{
  return this->comp_->setFootStepsWithParam(fs, spss);
}

void TorqueAutoStabilizerService_impl::waitFootSteps()
{
  return this->comp_->waitFootSteps();
};

CORBA::Boolean TorqueAutoStabilizerService_impl::startAutoBalancer()
{
  return this->comp_->startAutoBalancer();
};

CORBA::Boolean TorqueAutoStabilizerService_impl::stopAutoBalancer()
{
  return this->comp_->stopAutoBalancer();
};

CORBA::Boolean TorqueAutoStabilizerService_impl::startStabilizer(void)
{
  return this->comp_->startStabilizer();
}

CORBA::Boolean TorqueAutoStabilizerService_impl::stopStabilizer(void)
{
  return this->comp_->stopStabilizer();
}

CORBA::Boolean TorqueAutoStabilizerService_impl::setTorqueAutoStabilizerParam(const OpenHRP::TorqueAutoStabilizerService::TorqueAutoStabilizerParam& i_param)
{
  return this->comp_->setTorqueAutoStabilizerParam(i_param);
};

CORBA::Boolean TorqueAutoStabilizerService_impl::getTorqueAutoStabilizerParam(OpenHRP::TorqueAutoStabilizerService::TorqueAutoStabilizerParam_out i_param)
{
  i_param = new OpenHRP::TorqueAutoStabilizerService::TorqueAutoStabilizerParam();
  return this->comp_->getTorqueAutoStabilizerParam(*i_param);
};

CORBA::Boolean TorqueAutoStabilizerService_impl::getFootStepState(OpenHRP::TorqueAutoStabilizerService::FootStepState_out i_param)
{
  i_param = new OpenHRP::TorqueAutoStabilizerService::FootStepState();
  return this->comp_->getFootStepState(*i_param);
};

CORBA::Boolean TorqueAutoStabilizerService_impl::releaseEmergencyStop()
{
    return this->comp_->releaseEmergencyStop();
};

CORBA::Boolean TorqueAutoStabilizerService_impl::startImpedanceController(const char *i_name_)
{
  return this->comp_->startImpedanceController(i_name_);
};

CORBA::Boolean TorqueAutoStabilizerService_impl::stopImpedanceController(const char *i_name_)
{
  return this->comp_->stopImpedanceController(i_name_);
};

CORBA::Boolean TorqueAutoStabilizerService_impl::startWholeBodyMasterSlave(void)
{
  return this->comp_->startWholeBodyMasterSlave();
}

CORBA::Boolean TorqueAutoStabilizerService_impl::stopWholeBodyMasterSlave(void)
{
  return this->comp_->stopWholeBodyMasterSlave();
}

void TorqueAutoStabilizerService_impl::setComp(TorqueAutoStabilizer *i_comp)
{
  this->comp_ = i_comp;
}
