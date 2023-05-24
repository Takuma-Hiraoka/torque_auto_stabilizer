#include "TorqueAutoStabilizer.h"
#include "pinocchio/parsers/urdf.hpp"
#include <fstream>
#include <sstream>

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

  {
    // load dt
    std::string buf; this->getProperty("dt", buf);
    this->dt_ = std::stod(buf);
    if(this->dt_ <= 0.0){
      this->getProperty("exec_cxt.periodic.rate", buf);
      double rate = std::stod(buf);
      if(rate > 0.0){
        this->dt_ = 1.0/rate;
      }else{
        std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "dt is invalid" << "\x1b[39m" << std::endl;
        return RTC::RTC_ERROR;
      }
    }
  }

  {
    // load robot model
    std::string fileName;
    this->getProperty("urdf_model", fileName);
    if (fileName.find("file://") == 0) fileName.erase(0, strlen("file://"));
    pinocchio::urdf::buildModel(fileName,pinocchio::JointModelFreeFlyer(),this->model_,true);
    std::cerr << "model name: " << this->model_.name << std::endl;
    this->gaitParam_.init(this->model_);
  }

  {
    // load joint id table
    std::string fileName;
    this->getProperty("joint_id_table", fileName);
    if (fileName.find("file://") == 0) fileName.erase(0, strlen("file://"));
    std::fstream id_table;
    id_table.open(fileName);
    this->joint_id_table_.resize(this->model_.nv -6);

    if (id_table.is_open()) {
      double tmp;
      int i = 0;
      for (; i < this->model_.nv -6; i++) { // jointの数だけ

      retry:
        {
          std::string str;
          if (std::getline(id_table, str)) {
            if (str.empty())   goto retry;
            if (str[0] == '#') goto retry;

            std::istringstream sstrm(str);
            sstrm >> tmp; // RobotHardware joint id. iと同じ
            if(sstrm.eof()) goto next;
            sstrm >> tmp;
            joint_id_table_[i] = tmp;
          } else {
            i--;
            break;
          }
        }

      next:
        std::cerr << "RobotHardware joint: " << i << " pinocchio joint_id: " << joint_id_table_[i] << std::endl;
      }
      id_table.close();
    }
  }

  return RTC::RTC_OK;
}

bool TorqueAutoStabilizer::readInPortData(Eigen::VectorXd& refRobotData, Eigen::VectorXd& actRobotData){
  if (this->m_qRefIn_.isNew()){
    this->m_qRefIn_.read();
    for(int i=0;i<this->m_qRef_.data.length();i++){
      refRobotData[7+this->joint_id_table_[i]] = m_qRef_.data[i];
    }
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
}

bool TorqueAutoStabilizer::writeOutPortData(const GaitParam & gaitParam){

  {
    m_q_.tm = m_qRef_.tm;
    m_q_.data.length(m_qRef_.data.length());
    for (int i = 0 ; i < m_q_.data.length(); i++){
      m_q_.data[i] = gaitParam.refRobotData[7+this->joint_id_table_[i]];
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

}

RTC::ReturnCode_t TorqueAutoStabilizer::onExecute(RTC::UniqueId ec_id){
  this->readInPortData(this->gaitParam_.refRobotData,this->gaitParam_.actRobotData);
  this->writeOutPortData(this->gaitParam_);
  return RTC::RTC_OK;
}

RTC::ReturnCode_t TorqueAutoStabilizer::onActivated(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);
  std::cerr << "[" << m_profile.instance_name << "] "<< "onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t TorqueAutoStabilizer::onDeactivated(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);
  std::cerr << "[" << m_profile.instance_name << "] "<< "onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

bool TorqueAutoStabilizer::torqueAutoStabilizerParam(const double data){
}

bool TorqueAutoStabilizer::getProperty(const std::string& key, std::string& ret) {
  if (this->getProperties().hasKey(key.c_str())) {
    ret = std::string(this->getProperties()[key.c_str()]);
  } else if (this->m_pManager->getConfig().hasKey(key.c_str())) { // 引数 -o で与えたプロパティを捕捉
    ret = std::string(this->m_pManager->getConfig()[key.c_str()]);
  } else {
    return false;
  }
  std::cerr << "[" << this->m_profile.instance_name << "] " << key << ": " << ret <<std::endl;
  return true;
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
