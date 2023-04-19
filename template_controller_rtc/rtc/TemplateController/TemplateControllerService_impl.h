#ifndef TemplateControllerSERVICESVC_IMPL_H
#define TemplateControllerSERVICESVC_IMPL_H

#include "template_controller_rtc/idl/TemplateControllerService.hh"

class TemplateController;

class TemplateControllerService_impl
  : public virtual POA_template_controller_rtc::TemplateControllerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  TemplateControllerService_impl();
  ~TemplateControllerService_impl();

  CORBA::Boolean templateParam(const CORBA::Double data);
  void setComp(TemplateController *i_comp);
private:
  TemplateController *comp_;
};

#endif
