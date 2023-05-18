#ifndef TorqueAutoStabilizerROSBridge_H
#define TorqueAutoStabilizerROSBridge_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>

#include <torque_auto_stabilizer_msgs/idl/TorqueAutoStabilizer.hh>

#include <ros/ros.h>

class TorqueAutoStabilizerROSBridge : public RTC::DataFlowComponentBase{
protected:
  ros::NodeHandle nh;

public:
  TorqueAutoStabilizerROSBridge(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
};

extern "C"
{
  void TorqueAutoStabilizerROSBridgeInit(RTC::Manager* manager);
};

#endif // TorqueAutoStabilizerROSBridge_H
