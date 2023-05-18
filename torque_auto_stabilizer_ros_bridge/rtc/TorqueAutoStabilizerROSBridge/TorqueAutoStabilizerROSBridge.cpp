#include "TorqueAutoStabilizerROSBridge.h"

TorqueAutoStabilizerROSBridge::TorqueAutoStabilizerROSBridge(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager)
{
}

RTC::ReturnCode_t TorqueAutoStabilizerROSBridge::onInitialize(){
  ros::NodeHandle pnh("~");
  
  return RTC::RTC_OK;
}

RTC::ReturnCode_t TorqueAutoStabilizerROSBridge::onExecute(RTC::UniqueId ec_id){
  ros::spinOnce();

  return RTC::RTC_OK;
}


static const char* TorqueAutoStabilizerROSBridge_spec[] = {
  "implementation_id", "TorqueAutoStabilizerROSBridge",
  "type_name",         "TorqueAutoStabilizerROSBridge",
  "description",       "TorqueAutoStabilizerROSBridge component",
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
    void TorqueAutoStabilizerROSBridgeInit(RTC::Manager* manager) {
        RTC::Properties profile(TorqueAutoStabilizerROSBridge_spec);
        manager->registerFactory(profile, RTC::Create<TorqueAutoStabilizerROSBridge>, RTC::Delete<TorqueAutoStabilizerROSBridge>);
    }
};
