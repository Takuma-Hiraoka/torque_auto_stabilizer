#include "TemplateControllerService_impl.h"
#include "TemplateController.h"

TemplateControllerService_impl::TemplateControllerService_impl()
{
}

TemplateControllerService_impl::~TemplateControllerService_impl()
{
}

void TemplateControllerService_impl::setComp(TemplateController *i_comp)
{
  comp_ = i_comp;
}

CORBA::Boolean TemplateControllerService_impl::templateParam(const CORBA::Double data)
{
  return comp_->templateParam(data);
};
