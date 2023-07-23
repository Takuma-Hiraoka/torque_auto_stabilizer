#include "TorqueAutoStabilizer.h"
#include <cnoid/BodyLoader>
#include <cnoid/ForceSensor>
#include <cnoid/RateGyroSensor>
#include <cnoid/EigenUtil>
#include "MathUtil.h"
#include <fstream>
#include <sstream>

TorqueAutoStabilizer::TorqueAutoStabilizer(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_qRefIn_("qRefIn", m_qRef_),
  m_tauRefIn_("tauRefIn", m_tauRef_),
  m_refBasePosIn_("refBasePosIn", m_refBasePos_),
  m_refBaseRpyIn_("refBaseRpyIn", m_refBaseRpy_),
  m_qActIn_("qActIn", m_qAct_),
  m_dqActIn_("dqActIn", m_dqAct_),
  m_actImuIn_("actImuIn", m_actImu_),
  m_tauActIn_("tauActIn", m_tauAct_),
  m_qOut_("q", m_q_),
  m_tauOut_("tauOut", m_tau_),
  m_genZmpOut_("genZmpOut", m_genZmp_),
  m_actCogOut_("actCogOut", m_actCog_),
  m_actDcmOut_("actDcmOut", m_actDcm_),
  m_dstLandingPosOut_("dstLandingPosOut", m_dstLandingPos_),
  m_remainTimeOut_("remainTimeOut", m_remainTime_),
  m_genCoordsOut_("genCoordsOut", m_genCoords_),

  m_torqueAutoStabilizerServicePort_("TorqueAutoStabilizerService")
{
  this->m_service0_.setComp(this);
}

RTC::ReturnCode_t TorqueAutoStabilizer::onInitialize(){
  addInPort("qRefIn", this->m_qRefIn_);
  addInPort("tauRefIn", this->m_tauRefIn_);
  addInPort("refBasePosIn", this->m_refBasePosIn_);
  addInPort("refBaseRpyIn", this->m_refBaseRpyIn_);
  addInPort("qActIn", this->m_qActIn_);
  addInPort("dqActIn", this->m_dqActIn_);
  addInPort("actImuIn", this->m_actImuIn_);
  addInPort("tauActIn", this->m_tauActIn_);
  addOutPort("q", this->m_qOut_);
  addOutPort("tauOut", this->m_tauOut_);
  addOutPort("genZmpOut", this->m_genZmpOut_);
  addOutPort("actCogOut", this->m_actCogOut_);
  addOutPort("actDcmOut", this->m_actDcmOut_);
  addOutPort("dstLandingPosOut", this->m_dstLandingPosOut_);
  addOutPort("remainTimeOut", this->m_remainTimeOut_);
  addOutPort("genCoordsOut", this->m_genCoordsOut_);

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
    cnoid::BodyLoader bodyLoader;
    std::string fileName; this->getProperty("model", fileName);
    if (fileName.find("file://") == 0) fileName.erase(0, strlen("file://"));
    cnoid::BodyPtr robot = bodyLoader.load(fileName);
    if(!robot){
      std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
      return RTC::RTC_ERROR;
    }
    this->gaitParam_.init(robot);
  }

  {
    // load end_effector
    std::string endEffectors; this->getProperty("end_effectors", endEffectors);
    std::stringstream ss_endEffectors(endEffectors);
    std::string buf;
    while(std::getline(ss_endEffectors, buf, ',')){
      std::string name;
      std::string parentLink;
      Eigen::Vector3d localp;
      Eigen::Vector3d localaxis;
      double localangle;

      //   name, parentLink, (not used), x, y, z, theta, ax, ay, az
      name = buf;
      if(!std::getline(ss_endEffectors, buf, ',')) break; parentLink = buf;
      if(!std::getline(ss_endEffectors, buf, ',')) break; // not used
      if(!std::getline(ss_endEffectors, buf, ',')) break; localp[0] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localp[1] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localp[2] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localaxis[0] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localaxis[1] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localaxis[2] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localangle = std::stod(buf);

      // check validity
      name.erase(std::remove(name.begin(), name.end(), ' '), name.end()); // remove whitespace
      parentLink.erase(std::remove(parentLink.begin(), parentLink.end(), ' '), parentLink.end()); // remove whitespace
      if(!this->gaitParam_.refRobotRaw->link(parentLink)){
        std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << " link [" << parentLink << "]" << " is not found for " << name << "\x1b[39m" << std::endl;
        return RTC::RTC_ERROR;
      }
      cnoid::Matrix3 localR;
      if(localaxis.norm() == 0) localR = cnoid::Matrix3::Identity();
      else localR = Eigen::AngleAxisd(localangle, localaxis.normalized()).toRotationMatrix();
      cnoid::Position localT;
      localT.translation() = localp;
      localT.linear() = localR;

      this->gaitParam_.push_backEE(name, parentLink, localT);
    }
    // 0番目が右脚. 1番目が左脚. という仮定がある.
  }

  {
    // add more ports (EndEffectorやForceSensorの情報を使って)

    // 各EndEffectorにつき、ref<name>WrenchInというInPortをつくる
    this->m_refEEWrenchIn_.resize(this->gaitParam_.eeName.size());
    this->m_refEEWrench_.resize(this->gaitParam_.eeName.size());
    for(int i=0;i<this->gaitParam_.eeName.size();i++){
      std::string name = "ref"+this->gaitParam_.eeName[i]+"WrenchIn";
      this->m_refEEWrenchIn_[i] = std::make_unique<RTC::InPort<RTC::TimedDoubleSeq> >(name.c_str(), this->m_refEEWrench_[i]);
      this->addInPort(name.c_str(), *(this->m_refEEWrenchIn_[i]));
    }

    // 各ForceSensorにつき、act<name>InというInportをつくる
    cnoid::DeviceList<cnoid::ForceSensor> forceSensors(this->gaitParam_.actRobotRaw->devices());
    this->m_actWrenchIn_.resize(forceSensors.size());
    this->m_actWrench_.resize(forceSensors.size());
    for(int i=0;i<forceSensors.size();i++){
      std::string name = "act"+forceSensors[i]->name()+"In";
      this->m_actWrenchIn_[i] = std::make_unique<RTC::InPort<RTC::TimedDoubleSeq> >(name.c_str(), this->m_actWrench_[i]);
      this->addInPort(name.c_str(), *(this->m_actWrenchIn_[i]));
    }

    // 各EndEffectorにつき、act<name>PoseOutというOutPortをつくる
    this->m_actEEPoseOut_.resize(this->gaitParam_.eeName.size());
    this->m_actEEPose_.resize(this->gaitParam_.eeName.size());
    for(int i=0;i<this->gaitParam_.eeName.size();i++){
      std::string name = "act"+this->gaitParam_.eeName[i]+"PoseOut";
      this->m_actEEPoseOut_[i] = std::make_unique<RTC::OutPort<RTC::TimedPose3D> >(name.c_str(), this->m_actEEPose_[i]);
      this->addOutPort(name.c_str(), *(this->m_actEEPoseOut_[i]));
    }

    // 各EndEffectorにつき、act<name>WrenchOutというOutPortをつくる
    this->m_actEEWrenchOut_.resize(this->gaitParam_.eeName.size());
    this->m_actEEWrench_.resize(this->gaitParam_.eeName.size());
    for(int i=0;i<this->gaitParam_.eeName.size();i++){
      std::string name = "act"+this->gaitParam_.eeName[i]+"WrenchOut";
      this->m_actEEWrenchOut_[i] = std::make_unique<RTC::OutPort<RTC::TimedDoubleSeq> >(name.c_str(), this->m_actEEWrench_[i]);
      this->addOutPort(name.c_str(), *(this->m_actEEWrenchOut_[i]));
    }

  }

  {
    // init ActToGenFrameConverter
    this->actToGenFrameConverter_.eeForceSensor.resize(this->gaitParam_.eeName.size());
    cnoid::DeviceList<cnoid::ForceSensor> forceSensors(this->gaitParam_.refRobotRaw->devices());
    for(int i=0;i<this->gaitParam_.eeName.size();i++){
      // 各EndEffectorsから親リンク側に遡っていき、最初に見つかったForceSensorをEndEffectorに対応付ける. 以後、ForceSensorの値を座標変換したものがEndEffectorが受けている力とみなされる. 見つからなければ受けている力は常に0とみなされる
      std::string forceSensor = "";
      cnoid::LinkPtr link = this->gaitParam_.refRobotRaw->link(this->gaitParam_.eeParentLink[i]);
      bool found = false;
      while (link != nullptr && found == false) {
        for (size_t j = 0; j < forceSensors.size(); j++) {
          if(forceSensors[j]->link() == link) {
            forceSensor = forceSensors[j]->name();
            found = true;
            break;
          }
        }
      }
      this->actToGenFrameConverter_.eeForceSensor[i] = forceSensor;
    }
  }


  return RTC::RTC_OK;
}

bool TorqueAutoStabilizer::readInPortData(const GaitParam& gaitParam, cnoid::BodyPtr refRobotRaw, cnoid::BodyPtr actRobotRaw){
  if (this->m_qRefIn_.isNew()){
    this->m_qRefIn_.read();
    for(int i=0;i<this->m_qRef_.data.length();i++){
      refRobotRaw->joint(i)->q() = m_qRef_.data[i];
    }
  }

  if (this->m_tauRefIn_.isNew()){
    this->m_tauRefIn_.read();
    for(int i=0;i<this->m_qRef_.data.length();i++){
      refRobotRaw->joint(i)->u() = m_tauRef_.data[i];
    }
  }

  if(this->m_refBasePosIn_.isNew()){
    this->m_refBasePosIn_.read();
    refRobotRaw->rootLink()->p()[0] = this->m_refBasePos_.data.x;
    refRobotRaw->rootLink()->p()[1] = this->m_refBasePos_.data.y;
    refRobotRaw->rootLink()->p()[2] = this->m_refBasePos_.data.z;
  }
  if(this->m_refBaseRpyIn_.isNew()){
    this->m_refBaseRpyIn_.read();
    refRobotRaw->rootLink()->R() = mathutil::rotFromRpy(this->m_refBaseRpy_.data.r, this->m_refBaseRpy_.data.p, this->m_refBaseRpy_.data.y);
  }

  refRobotRaw->calcForwardKinematics();
  refRobotRaw->calcCenterOfMass();

  for(int i=0;i<this->m_refEEWrenchIn_.size();i++){
    if(this->m_refEEWrenchIn_[i]->isNew()){
      this->m_refEEWrenchIn_[i]->read();
    }
  }

  if (this->m_qActIn_.isNew()){
    this->m_qActIn_.read();
    for(int i=0;i<this->m_qAct_.data.length();i++){
      actRobotRaw->joint(i)->q() = m_qAct_.data[i];
    }
  }

  if (this->m_dqActIn_.isNew()){
    this->m_dqActIn_.read();
    for(int i=0;i<this->m_dqAct_.data.length();i++){
      actRobotRaw->joint(i)->dq() = m_dqAct_.data[i];
    }
  }

  if(this->m_actImuIn_.isNew()){
    this->m_actImuIn_.read();
    actRobotRaw->calcForwardKinematics();
    cnoid::RateGyroSensorPtr imu = actRobotRaw->findDevice<cnoid::RateGyroSensor>("gyrometer");
    cnoid::Matrix3 imuR = imu->link()->R() * imu->R_local();
    cnoid::Matrix3 actR = cnoid::rotFromRpy(this->m_actImu_.data.r, this->m_actImu_.data.p, this->m_actImu_.data.y);
    actRobotRaw->rootLink()->R() = Eigen::Matrix3d(Eigen::AngleAxisd(actR) * Eigen::AngleAxisd(imuR.transpose() * actRobotRaw->rootLink()->R())); // 単純に3x3行列の空間でRを積算していると、だんだん数値誤差によって回転行列でなくなってしまう恐れがあるので念の為
  }

  actRobotRaw->calcForwardKinematics();
  actRobotRaw->calcCenterOfMass();

  if (this->m_tauActIn_.isNew()){
    this->m_tauActIn_.read();
    for(int i=0;i<this->m_qRef_.data.length();i++){
      actRobotRaw->joint(i)->u() = m_tauAct_.data[i];
    }
  }

  cnoid::DeviceList<cnoid::ForceSensor> forceSensors(actRobotRaw->devices());
  for(int i=0;i<this->m_actWrenchIn_.size();i++){
    if(this->m_actWrenchIn_[i]->isNew()){
      this->m_actWrenchIn_[i]->read();
      if(this->m_actWrench_[i].data.length() == 6){
        for(int j=0;j<6;j++){
          forceSensors[i]->F()[j] = this->m_actWrench_[i].data[j];
        }
      }
    }
  }
}

bool TorqueAutoStabilizer::execAutoStabilizer(const TorqueAutoStabilizer::ControlMode& mode, GaitParam& gaitParam, double dt, const ActToGenFrameConverter& actToGenFrameConverter, const RefToGenFrameConverter& refToGenFrameConverter, const FootStepGenerator& footStepGenerator, const LegCoordsGenerator& legCoordsGenerator) {
  // FootOrigin座標系を用いてactRobotRawをgenerate frameに投影しactRobotとする
  if(mode.isSyncToASTInit()){
    refToGenFrameConverter.initFootCoords(gaitParam, gaitParam.footMidCoords, gaitParam.refRobot);
    footStepGenerator.initFootStepNodesList(gaitParam,
                                            gaitParam.footStepNodesList, gaitParam.srcCoords, gaitParam.dstCoordsOrg, gaitParam.remainTimeOrg, gaitParam.swingState, gaitParam.elapsedTime, gaitParam.prevSupportPhase);
    legCoordsGenerator.initLegCoords(gaitParam,
				     gaitParam.refZmpTraj, gaitParam.genCoords);
  }

  // FootOrigin座標系を用いてactRobotRawをgenerate frameに投影しactRobotとする
  actToGenFrameConverter.convertFrame(gaitParam, dt,
                                      gaitParam.actRobot, gaitParam.actEEPose, gaitParam.actEEWrench, gaitParam.actCogVel, gaitParam.actRootVel, gaitParam.actCog, gaitParam.prevOriginLeg);

  // FootOrigin座標系を用いてrefRobotPosをgenerate frameに投影しrefRobotとする
  refToGenFrameConverter.convertFrame(gaitParam, dt, gaitParam.refRobot, gaitParam.refEEPose, gaitParam.refEEWrench, gaitParam.refdz, gaitParam.footMidCoords);

  // FootStepNodesListをdtすすめる
  footStepGenerator.procFootStepNodesList(gaitParam, dt, true,
                                          gaitParam.footStepNodesList, gaitParam.srcCoords, gaitParam.dstCoordsOrg, gaitParam.remainTimeOrg, gaitParam.swingState, gaitParam.elapsedTime, gaitParam.prevSupportPhase, gaitParam.relLandingHeight);

  // 次のFootStepの追加、修正を行う
  footStepGenerator.calcFootSteps(gaitParam, dt, true,
                                  gaitParam.debugData, //for log
                                  gaitParam.footStepNodesList);

  // 目標とすべき足先位置を求める
  //  legCoordsGenerator.calcLegCoords(gaitParam, model, dt,
  //                                  gaitParam.refZmpTraj, gaitParam.genCoords, gaitParam.swingState);

  return true;
}

bool TorqueAutoStabilizer::writeOutPortData(const GaitParam & gaitParam){

  {
    m_q_.tm = m_qRef_.tm;
    m_q_.data.length(m_qRef_.data.length());
    for (int i = 0 ; i < m_q_.data.length(); i++){
      m_q_.data[i] = gaitParam.refRobotRaw->joint(i)->q();
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

  if(this->mode_.isASTRunning()){
    for(int i=0;i<gaitParam.eeName.size();i++){
      this->m_actEEPose_[i].tm = this->m_qRef_.tm;
      this->m_actEEPose_[i].data.position.x = gaitParam.actEEPose[i].translation()[0];
      this->m_actEEPose_[i].data.position.y = gaitParam.actEEPose[i].translation()[1];
      this->m_actEEPose_[i].data.position.z = gaitParam.actEEPose[i].translation()[2];
      Eigen::Vector3d rpy = mathutil::rpyFromRot(gaitParam.actEEPose[i].linear());
      this->m_actEEPose_[i].data.orientation.r = rpy[0];
      this->m_actEEPose_[i].data.orientation.p = rpy[1];
      this->m_actEEPose_[i].data.orientation.y = rpy[2];
      this->m_actEEPoseOut_[i]->write();
    }

    for(int i=0;i<gaitParam.actEEWrench.size();i++){
      this->m_actEEWrench_[i].tm = this->m_qRef_.tm;
      this->m_actEEWrench_[i].data.length(6);
      for(int j=0;j<6;j++) this->m_actEEWrench_[i].data[j] = gaitParam.actEEWrench[i][j];
      this->m_actEEWrenchOut_[i]->write();
    }

    this->m_genZmp_.tm = this->m_qRef_.tm;
    this->m_genZmp_.data.x = gaitParam.refZmpTraj[0].getStart()[0];
    this->m_genZmp_.data.y = gaitParam.refZmpTraj[0].getStart()[1];
    this->m_genZmp_.data.z = gaitParam.refZmpTraj[0].getStart()[2];
    this->m_genZmpOut_.write();
    this->m_actCog_.tm = this->m_qRef_.tm;
    this->m_actCog_.data.x = gaitParam.actCog[0];
    this->m_actCog_.data.y = gaitParam.actCog[1];
    this->m_actCog_.data.z = gaitParam.actCog[2];
    this->m_actCogOut_.write();
    Eigen::Vector3d actDcm = gaitParam.actCog + gaitParam.actCogVel.value() / gaitParam.omega;
    this->m_actDcm_.tm = this->m_qRef_.tm;
    this->m_actDcm_.data.x = actDcm[0];
    this->m_actDcm_.data.y = actDcm[1];
    this->m_actDcm_.data.z = actDcm[2];
    this->m_actDcmOut_.write();
    this->m_dstLandingPos_.tm = this->m_qRef_.tm;
    this->m_dstLandingPos_.data.length(6);
    this->m_dstLandingPos_.data[0] = gaitParam.footStepNodesList[0].dstCoords[RLEG].translation()[0];
    this->m_dstLandingPos_.data[1] = gaitParam.footStepNodesList[0].dstCoords[RLEG].translation()[1];
    this->m_dstLandingPos_.data[2] = gaitParam.footStepNodesList[0].dstCoords[RLEG].translation()[2];
    this->m_dstLandingPos_.data[3] = gaitParam.footStepNodesList[0].dstCoords[LLEG].translation()[0];
    this->m_dstLandingPos_.data[4] = gaitParam.footStepNodesList[0].dstCoords[LLEG].translation()[1];
    this->m_dstLandingPos_.data[5] = gaitParam.footStepNodesList[0].dstCoords[LLEG].translation()[2];
    this->m_dstLandingPosOut_.write();
    this->m_remainTime_.tm = this->m_qRef_.tm;
    this->m_remainTime_.data.length(1);
    this->m_remainTime_.data[0] = gaitParam.footStepNodesList[0].remainTime;
    this->m_remainTimeOut_.write();
    this->m_genCoords_.tm = this->m_qRef_.tm;
    this->m_genCoords_.data.length(12);
    for (int i=0; i<3; i++) {
      this->m_genCoords_.data[0+i] = gaitParam.genCoords[RLEG].value().translation()[i];
      this->m_genCoords_.data[3+i] = gaitParam.genCoords[LLEG].value().translation()[i];
      this->m_genCoords_.data[6+i] = gaitParam.genCoords[RLEG].getGoal().translation()[i];
      this->m_genCoords_.data[9+i] = gaitParam.genCoords[LLEG].getGoal().translation()[i];
    }
    this->m_genCoordsOut_.write();
  }

}

RTC::ReturnCode_t TorqueAutoStabilizer::onExecute(RTC::UniqueId ec_id){
  this->readInPortData(this->gaitParam_, this->gaitParam_.refRobotRaw, this->gaitParam_.actRobotRaw);

  this->mode_.update(this->dt_);

  if(this->mode_.isASTRunning()) {
    if(this->mode_.isSyncToASTInit()){ // startAutoBalancer直後の初回. 内部パラメータのリセット
      this->actToGenFrameConverter_.reset();
      this->refToGenFrameConverter_.reset();
      this->footStepGenerator_.reset();
    }
    TorqueAutoStabilizer::execAutoStabilizer(this->mode_, this->gaitParam_, this->dt_, this->actToGenFrameConverter_, this->refToGenFrameConverter_, this->footStepGenerator_, this->legCoordsGenerator_);
  }

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
bool TorqueAutoStabilizer::startAutoStabilizer(){
  if(this->mode_.setNextTransition(ControlMode::START_AST)){
    std::cerr << "[" << m_profile.instance_name << "] start auto stabilizer mode" << std::endl;
    while (this->mode_.now() != ControlMode::MODE_AST) usleep(1000);
    usleep(1000);
    return true;
  }else{
    std::cerr << "[" << this->m_profile.instance_name << "] auto stabilizer is already started" << std::endl;
    return false;
  }
}
bool TorqueAutoStabilizer::stopAutoStabilizer(){
  if(this->mode_.setNextTransition(ControlMode::STOP_AST)){
    std::cerr << "[" << m_profile.instance_name << "] stop auto stabilizer mode" << std::endl;
    while (this->mode_.now() != ControlMode::MODE_IDLE) usleep(1000);
    usleep(1000);
    return true;
  }else{
    std::cerr << "[" << this->m_profile.instance_name << "] auto stabilizer is already stopped or stabilizer is running" << std::endl;
    return false;
  }
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
