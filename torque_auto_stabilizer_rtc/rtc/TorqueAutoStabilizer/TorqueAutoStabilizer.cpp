#include "TorqueAutoStabilizer.h"

TorqueAutoStabilizer::TorqueAutoStabilizer(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_qRefIn_("qRefIn", m_qRef_),
  m_tauRefIn_("tauRefIn", m_tauRef_),
  m_qActIn_("qActIn", m_qAct_),
  m_dqActIn_("dqActIn", m_dqAct_),
  m_tauActIn_("tauActIn", m_tauAct_),
  m_qOut_("q", m_q_),
  m_tauOut_("tauOut", m_tau_),
  m_torqueAutoStabilizerServicePort_("TorqueAutoStabilizerService")
{
  this->m_service0_.setComp(this);
}

RTC::ReturnCode_t TorqueAutoStabilizer::onInitialize(){
  addInPort("qRefIn", this->m_qRefIn_);
  addInPort("tauRefIn", this->m_tauRefIn_);
  addInPort("qActIn", this->m_qActIn_);
  addInPort("dqActIn", this->m_dqActIn_);
  addInPort("tauActIn", this->m_tauActIn_);
  addOutPort("q", this->m_qOut_);
  addOutPort("tauOut", this->m_tauOut_);
  this->m_torqueAutoStabilizerServicePort_.registerProvider("service0", "TorqueAutoStabilizerService", this->m_service0_);
  addPort(this->m_torqueAutoStabilizerServicePort_);

  return RTC::RTC_OK;
}

RTC::ReturnCode_t TorqueAutoStabilizer::onExecute(RTC::UniqueId ec_id){
  std::cerr << "TorqueAutoStabilizer rtc onExecute" << std::endl;

  if (this->m_qRefIn_.isNew()){
    this->m_qRefIn_.read();
  }

  if (this->m_tauRefIn_.isNew()){
    this->m_tauRefIn_.read();
  }

  if (this->m_qActIn_.isNew()){
    this->m_qActIn_.read();
  }

  if (this->m_dqActIn_.isNew()){
    this->m_dqActIn_.read();
  }

  if (this->m_tauActIn_.isNew()){
    this->m_tauActIn_.read();
  }

  {
    m_q_.tm = m_qRef_.tm;
    m_q_.data.length(m_qRef_.data.length());
    for (int i = 0 ; i < m_q_.data.length(); i++){
      m_q_.data[i] = m_qRef_.data[i];
    }
    this->m_qOut_.write();
  }

  {
    m_tau_.tm = m_qRef_.tm;
    m_tau_.data.length(m_tauRef_.data.length());
    for (int i = 0 ; i < m_q_.data.length(); i++){
      m_tau_.data[i] = m_tauRef_.data[i];
    }
    this->m_tauOut_.write();
  }

  return RTC::RTC_OK;
}

bool TorqueAutoStabilizer::torqueAutoStabilizerParam(const double data){
}

static const char* TorqueAutoStabilizer_spec[] = {
  "implementation_id", "TorqueAutoStabilizer",
  "type_name",         "TorqueAutoStabilizer",
  "description",       "TorqueAutoStabilizer component",
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
  void TorqueAutoStabilizerInit(RTC::Manager* manager) {
    RTC::Properties profile(TorqueAutoStabilizer_spec);
    manager->registerFactory(profile, RTC::Create<TorqueAutoStabilizer>, RTC::Delete<TorqueAutoStabilizer>);
  }
};
