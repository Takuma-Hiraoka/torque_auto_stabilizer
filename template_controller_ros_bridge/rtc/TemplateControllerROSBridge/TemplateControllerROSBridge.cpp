#include "TemplateControllerROSBridge.h"

TemplateControllerROSBridge::TemplateControllerROSBridge(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager)
{
}

RTC::ReturnCode_t TemplateControllerROSBridge::onInitialize(){
  ros::NodeHandle pnh("~");
  
  return RTC::RTC_OK;
}

RTC::ReturnCode_t TemplateControllerROSBridge::onExecute(RTC::UniqueId ec_id){
  ros::spinOnce();

  return RTC::RTC_OK;
}


static const char* TemplateControllerROSBridge_spec[] = {
  "implementation_id", "TemplateControllerROSBridge",
  "type_name",         "TemplateControllerROSBridge",
  "description",       "TemplateControllerROSBridge component",
  "version",           "0.0",
  "vendor",            "Takuma-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};

extern "C"{
    void TemplateControllerROSBridgeInit(RTC::Manager* manager) {
        RTC::Properties profile(TemplateControllerROSBridge_spec);
        manager->registerFactory(profile, RTC::Create<TemplateControllerROSBridge>, RTC::Delete<TemplateControllerROSBridge>);
    }
};
