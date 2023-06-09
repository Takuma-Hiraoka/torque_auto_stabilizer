#include "TorqueAutoStabilizerService_impl.h"
#include "TorqueAutoStabilizer.h"

TorqueAutoStabilizerService_impl::TorqueAutoStabilizerService_impl()
{
}

TorqueAutoStabilizerService_impl::~TorqueAutoStabilizerService_impl()
{
}

void TorqueAutoStabilizerService_impl::setComp(TorqueAutoStabilizer *i_comp)
{
  comp_ = i_comp;
}

CORBA::Boolean TorqueAutoStabilizerService_impl::torqueAutoStabilizerParam(const CORBA::Double data)
{
  return comp_->torqueAutoStabilizerParam(data);
};

CORBA::Boolean TorqueAutoStabilizerService_impl::startAutoStabilizer()
{
  return this->comp_->startAutoStabilizer();
};

CORBA::Boolean TorqueAutoStabilizerService_impl::stopAutoStabilizer()
{
  return this->comp_->stopAutoStabilizer();
};
