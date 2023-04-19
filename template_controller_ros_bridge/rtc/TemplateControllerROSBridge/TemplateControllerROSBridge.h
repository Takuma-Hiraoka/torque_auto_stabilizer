#ifndef TemplateControllerROSBridge_H
#define TemplateControllerROSBridge_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>

#include <template_controller_msgs/idl/TemplateController.hh>

#include <ros/ros.h>

class TemplateControllerROSBridge : public RTC::DataFlowComponentBase{
protected:
  ros::NodeHandle nh;

public:
  TemplateControllerROSBridge(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
};

extern "C"
{
  void TemplateControllerROSBridgeInit(RTC::Manager* manager);
};

#endif // TemplateControllerROSBridge_H
