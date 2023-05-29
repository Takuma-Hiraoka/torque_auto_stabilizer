#include "FootStepGenerator.h"

bool FootStepGenerator::initFootStepNodesList(const GaitParam& gaitParam, const pinocchio::Model& model,
                                              std::vector<GaitParam::FootStepNodes>& o_footStepNodesList, std::vector<pinocchio::SE3>& o_srcCoords, std::vector<pinocchio::SE3>& o_dstCoordsOrg, double& o_remainTimeOrg, std::vector<GaitParam::SwingState_enum>& o_swingState, double& o_elapsedTime, std::vector<bool>& o_prevSupportPhase) const{
  // footStepNodesListを初期化する
  std::vector<GaitParam::FootStepNodes> footStepNodesList(1);
  pinocchio::SE3 rlegCoords = gaitParam.refRobot.oMi[model.getJointId(gaitParam.eeParentLink[RLEG])]*gaitParam.eeLocalT[RLEG];
  pinocchio::SE3 llegCoords = gaitParam.refRobot.oMi[model.getJointId(gaitParam.eeParentLink[LLEG])]*gaitParam.eeLocalT[LLEG];
  footStepNodesList[0].dstCoords = {rlegCoords, llegCoords};
  footStepNodesList[0].isSupportPhase = {(gaitParam.isManualControlMode[RLEG].getGoal() == 0.0), (gaitParam.isManualControlMode[LLEG].getGoal() == 0.0)};
  footStepNodesList[0].remainTime = 0.0;
  if(footStepNodesList[0].isSupportPhase[RLEG] && !footStepNodesList[0].isSupportPhase[LLEG]) footStepNodesList[0].endRefZmpState = GaitParam::FootStepNodes::refZmpState_enum::RLEG;
  else if(!footStepNodesList[0].isSupportPhase[RLEG] && footStepNodesList[0].isSupportPhase[LLEG]) footStepNodesList[0].endRefZmpState = GaitParam::FootStepNodes::refZmpState_enum::LLEG;
  else footStepNodesList[0].endRefZmpState = GaitParam::FootStepNodes::refZmpState_enum::MIDDLE;
  std::vector<pinocchio::SE3> srcCoords = footStepNodesList[0].dstCoords;
  std::vector<pinocchio::SE3> dstCoordsOrg = footStepNodesList[0].dstCoords;
  double remainTimeOrg = footStepNodesList[0].remainTime;
  std::vector<GaitParam::SwingState_enum> swingState(NUM_LEGS);
  for(int i=0;i<NUM_LEGS;i++) swingState[i] = GaitParam::LIFT_PHASE;
  std::vector<bool> prevSupportPhase(NUM_LEGS);
  for(int i=0;i<NUM_LEGS;i++) prevSupportPhase[i] = footStepNodesList[0].isSupportPhase[i];
  double elapsedTime = 0.0;

  o_prevSupportPhase = prevSupportPhase;
  o_footStepNodesList = footStepNodesList;
  o_srcCoords = srcCoords;
  o_dstCoordsOrg = dstCoordsOrg;
  o_remainTimeOrg = remainTimeOrg;
  o_swingState = swingState;
  o_elapsedTime = elapsedTime;

  return true;
}

bool FootStepGenerator::setFootSteps(const GaitParam& gaitParam, const std::vector<StepNode>& footsteps,
                                     std::vector<GaitParam::FootStepNodes>& o_footStepNodesList) const{
  if(!gaitParam.isStatic()){ // 静止中でないと無効
    o_footStepNodesList = gaitParam.footStepNodesList;
    return false;
  }

  if(footsteps.size() <= 1) { // 何もしない
    o_footStepNodesList = gaitParam.footStepNodesList;
    return true;
  }

  if(footsteps[0].l_r == footsteps[1].l_r){ // 基準が無い. 無効
    o_footStepNodesList = gaitParam.footStepNodesList;
    return false;
  }

  if((!gaitParam.footStepNodesList[0].isSupportPhase[RLEG] && footsteps[1].l_r == LLEG) ||
     (!gaitParam.footStepNodesList[0].isSupportPhase[LLEG] && footsteps[1].l_r == RLEG)){ // 空中の足を支持脚にしようとしている. 無効
    o_footStepNodesList = gaitParam.footStepNodesList;
    return false;
  }

  for(int i=1;i<footsteps.size();i++){
    if((footsteps[i-1].l_r == RLEG && footsteps[i-1].swingEnd == true && footsteps[i].l_r == LLEG) ||
       (footsteps[i-1].l_r == LLEG && footsteps[i-1].swingEnd == true && footsteps[i].l_r == RLEG)){ // 空中の足を支持脚にしようとしている. 無効
      o_footStepNodesList = gaitParam.footStepNodesList;
      return false;
    }
  }

  std::vector<GaitParam::FootStepNodes> footStepNodesList;
  footStepNodesList.push_back(gaitParam.footStepNodesList[0]);

  if(footStepNodesList.back().isSupportPhase[RLEG] && footStepNodesList.back().isSupportPhase[LLEG]){ // 両足支持期を延長
    // 現在の時刻から突然refZmpTrajが変化すると、大きなZMP入力変化が必要になる. いまの位置でrefZmpTrajをthis->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio)の間とめて、次にthis->defaultStepTime * this->defaultDoubleSupportRatioの間で次の支持脚側に動かす
    footStepNodesList.back().remainTime = this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio);
    footStepNodesList.push_back(calcDefaultDoubleSupportStep(footStepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio, GaitParam::FootStepNodes::refZmpState_enum::MIDDLE));
  }

  // footstepsの0番目の要素は、実際には歩かず、基準座標としてのみ使われる.
  // footstepsの反対側の足が最後についた位置姿勢のZ軸を鉛直に直した座標系からみたfootstepsのi番目の要素の位置姿勢に、
  // footStepNodesList.back().dstCoordsのZ軸を鉛直に直した座標系から見た次の一歩の着地位置がなるように、座標変換する.
  std::vector<pinocchio::SE3> legPoseInFootSteps(2); // footStepsの足が最後についた位置姿勢. footSteps frame
  for(int i=0;i<2;i++) legPoseInFootSteps[footsteps[i].l_r] = mathutil::orientCoordToAxis(footsteps[i].coords, Eigen::Vector3d::UnitZ());
  for(int i=1;i<footsteps.size();i++){
    if(footStepNodesList.back().endRefZmpState != GaitParam::FootStepNodes::refZmpState_enum::RLEG && footsteps[i].l_r == LLEG){ // 両足支持期
      footStepNodesList.back().endRefZmpState = GaitParam::FootStepNodes::refZmpState_enum::RLEG;
    }else if(footStepNodesList.back().endRefZmpState != GaitParam::FootStepNodes::refZmpState_enum::LLEG && footsteps[i].l_r == RLEG){ // 両足支持期
      footStepNodesList.back().endRefZmpState = GaitParam::FootStepNodes::refZmpState_enum::LLEG;
    }

    GaitParam::FootStepNodes fs;
    int swingLeg = footsteps[i].l_r;
    int supportLeg = swingLeg == RLEG ? LLEG: RLEG;
    fs.dstCoords[supportLeg] = footStepNodesList.back().dstCoords[supportLeg];
    {
      pinocchio::SE3 transform = legPoseInFootSteps[supportLeg].inverse() * footsteps[i].coords; // supportLeg相対(Z軸は鉛直)での次のswingLegの位置
      // 着地位置をoverwritableStrideLimitationでリミット. swingEndのときでもリミットしたほうが良い
      Eigen::Vector3d localZ = transform.rotation() * Eigen::Vector3d::Zero();
      double theta = mathutil::rpyFromRot(mathutil::orientCoordToAxis(transform.rotation(), Eigen::Vector3d::Zero()))[2];
      theta = mathutil::clamp(theta, this->overwritableStrideLimitationMinTheta[swingLeg], this->overwritableStrideLimitationMaxTheta[swingLeg]);
      transform.rotation() = mathutil::orientCoordToAxis(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix(), localZ);
      std::vector<Eigen::Vector3d> strideLimitationHull = this->calcRealStrideLimitationHull(swingLeg, theta, gaitParam.legHull, gaitParam.defaultTranslatePos, this->overwritableStrideLimitationHull);
      transform.translation().head<2>() = mathutil::calcNearestPointOfHull(transform.translation(), strideLimitationHull).head<2>();
      fs.dstCoords[swingLeg] = mathutil::orientCoordToAxis(footStepNodesList.back().dstCoords[supportLeg], Eigen::Vector3d::UnitZ()) * transform;
    }
    legPoseInFootSteps[swingLeg] = mathutil::orientCoordToAxis(footsteps[i].coords, Eigen::Vector3d::UnitZ());
    fs.isSupportPhase[supportLeg] = true;
    fs.isSupportPhase[swingLeg] = false;
    fs.remainTime = footsteps[i].stepTime * (1 - this->defaultDoubleSupportRatio);
    fs.endRefZmpState = supportLeg == RLEG ? GaitParam::FootStepNodes::refZmpState_enum::RLEG : GaitParam::FootStepNodes::refZmpState_enum::LLEG;
    double stepHeight = std::max(0.0, footsteps[i].stepHeight);
    double beforeHeight = footStepNodesList.back().isSupportPhase[swingLeg] ? stepHeight : 0.0;
    double afterHeight = footsteps[i].swingEnd ? 0.0 : stepHeight;
    fs.stepHeight[swingLeg] = {beforeHeight,afterHeight};
    fs.touchVel[swingLeg] = footsteps[i].swingEnd ? 0.0 : this->touchVel;
    fs.goalOffset[swingLeg] = 0.0;
    footStepNodesList.push_back(fs);

    if(!footsteps[i].swingEnd) footStepNodesList.push_back(calcDefaultDoubleSupportStep(footStepNodesList.back(), footsteps[i].stepTime * this->defaultDoubleSupportRatio, swingLeg == RLEG ? GaitParam::FootStepNodes::refZmpState_enum::RLEG : GaitParam::FootStepNodes::refZmpState_enum::LLEG));
  }

  if(footStepNodesList.back().isSupportPhase[RLEG] && footStepNodesList.back().isSupportPhase[LLEG]){
    // refZmpを両足の中心へ戻す
    footStepNodesList.push_back(calcDefaultDoubleSupportStep(footStepNodesList.back(), this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio), footStepNodesList.back().endRefZmpState));
    footStepNodesList.push_back(calcDefaultDoubleSupportStep(footStepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio, GaitParam::FootStepNodes::refZmpState_enum::MIDDLE));
    footStepNodesList.push_back(calcDefaultDoubleSupportStep(footStepNodesList.back(), this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio), GaitParam::FootStepNodes::refZmpState_enum::MIDDLE)); // 末尾の両足支持期を延長. これがないと重心が目標位置に収束する前に返ってしまい, emergencyStepが無限に誘発する footGudedBalanceTimeを0.4程度に小さくすると収束が速くなるのでこの処理が不要になるのだが、今度はZ方向に振動しやすい
  }

  o_footStepNodesList = footStepNodesList;
  return true;
}

bool FootStepGenerator::goPos(const GaitParam& gaitParam, double x/*m*/, double y/*m*/, double th/*deg*/,
                              std::vector<GaitParam::FootStepNodes>& o_footStepNodesList) const{
  if(!gaitParam.isStatic()){ // 静止中でないと無効
    o_footStepNodesList = gaitParam.footStepNodesList;
    return false;
  }

  std::vector<GaitParam::FootStepNodes> footStepNodesList;
  footStepNodesList.push_back(gaitParam.footStepNodesList[0]);

  pinocchio::SE3 currentPose;
  {
    pinocchio::SE3 rleg = mathutil::orientCoordToAxis(footStepNodesList.back().dstCoords[RLEG], Eigen::Vector3d::UnitZ());
    rleg.translation() -= rleg.rotation() * gaitParam.defaultTranslatePos[RLEG].value();
    pinocchio::SE3 lleg = mathutil::orientCoordToAxis(footStepNodesList.back().dstCoords[LLEG], Eigen::Vector3d::UnitZ());
    lleg.translation() -= lleg.rotation() * gaitParam.defaultTranslatePos[LLEG].value();
    currentPose = mathutil::calcMidCoords(std::vector<pinocchio::SE3>{rleg, lleg}, std::vector<double>{footStepNodesList.back().isSupportPhase[RLEG] ? 1.0 : 0.0, footStepNodesList.back().isSupportPhase[LLEG] ? 1.0 : 0.0});
  }
  pinocchio::SE3 trans;
  trans.translation() = Eigen::Vector3d(x, y, 0.0);
  trans.rotation() = Eigen::AngleAxisd(th * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const pinocchio::SE3 goalPose = currentPose * trans; // generate frame. Z軸は鉛直

  int steps = 0;
  while(steps < 100){
    Eigen::Vector3d diff;
    pinocchio::SE3 trans = currentPose.inverse() * goalPose; // currentPose frame
    diff.head<2>() = trans.translation().head<2>(); //currentPose frame. [m]
    diff[2] = mathutil::rpyFromRot(trans.rotation())[2]; // currentPose frame. [rad]
    if(steps >= 1 && // 最低1歩は歩く. (そうしないと、両脚がdefaultTranslatePosとは違う開き方をしているときにgoPos(0,0,0)すると、defaultTranslatePosに戻れない)
       (diff.head<2>().norm() < 1e-3 * 0.1) && (std::abs(diff[2]) < 0.5 * M_PI / 180.0)) break;
    this->calcDefaultNextStep(footStepNodesList, gaitParam, diff);
    steps++;
    {
      pinocchio::SE3 rleg = mathutil::orientCoordToAxis(footStepNodesList.back().dstCoords[RLEG], Eigen::Vector3d::UnitZ());
      rleg.translation() -= rleg.rotation() * gaitParam.defaultTranslatePos[RLEG].value();
      pinocchio::SE3 lleg = mathutil::orientCoordToAxis(footStepNodesList.back().dstCoords[LLEG], Eigen::Vector3d::UnitZ());
      lleg.translation() -= lleg.rotation() * gaitParam.defaultTranslatePos[LLEG].value();
      currentPose = mathutil::calcMidCoords(std::vector<pinocchio::SE3>{rleg, lleg}, std::vector<double>{footStepNodesList[footStepNodesList.size()-2].isSupportPhase[LLEG] ? 1.0 : 0.0, footStepNodesList[footStepNodesList.size()-2].isSupportPhase[RLEG] ? 1.0 : 0.0}); // 前回swingした足の位置を見る
    }
  }

  if(steps >= 100) { // goalに到達しない
    o_footStepNodesList = gaitParam.footStepNodesList;
    return false;
  }

  // 両脚が横に並ぶ位置に1歩歩く.
  for(int i=0;i<1;i++){
    this->calcDefaultNextStep(footStepNodesList, gaitParam);
  }

  // refZmpを両足の中心へ戻す
  footStepNodesList.push_back(calcDefaultDoubleSupportStep(footStepNodesList.back(), this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio), footStepNodesList.back().endRefZmpState));
  footStepNodesList.push_back(calcDefaultDoubleSupportStep(footStepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio, GaitParam::FootStepNodes::refZmpState_enum::MIDDLE));
  footStepNodesList.push_back(calcDefaultDoubleSupportStep(footStepNodesList.back(), this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio), GaitParam::FootStepNodes::refZmpState_enum::MIDDLE)); // 末尾の両足支持期を延長. これがないと重心が目標位置に収束する前に返ってしまい, emergencyStepが無限に誘発する footGudedBalanceTimeを0.4程度に小さくすると収束が速くなるのでこの処理が不要になるのだが、今度はZ方向に振動しやすい

  o_footStepNodesList = footStepNodesList;
  return true;
}


bool FootStepGenerator::goStop(const GaitParam& gaitParam,
            std::vector<GaitParam::FootStepNodes>& o_footStepNodesList) const {
  if(gaitParam.isStatic()){
    o_footStepNodesList = gaitParam.footStepNodesList;
    return true;
  }

  std::vector<GaitParam::FootStepNodes> footStepNodesList = gaitParam.footStepNodesList;

  // footStepNodesList[0]と[1]は変えない. footStepNodesList[1]以降で次に両足支持期になるときを探し、それ以降のstepを上書きする. 片足支持期の状態が末尾の要素になると、片足立ちで止まるという状態を意味することに注意
  for(int i=1;i<footStepNodesList.size();i++){
    if(footStepNodesList[i].isSupportPhase[RLEG] && footStepNodesList[i].isSupportPhase[LLEG]){
      footStepNodesList.resize(i+1);
      break;
    }
  }

  // 両脚が横に並ぶ位置に1歩歩く.
  for(int i=0;i<1;i++){
    this->calcDefaultNextStep(footStepNodesList, gaitParam);
  }

  // refZmpを両足の中心へ戻す
  footStepNodesList.push_back(calcDefaultDoubleSupportStep(footStepNodesList.back(), this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio), footStepNodesList.back().endRefZmpState));
  footStepNodesList.push_back(calcDefaultDoubleSupportStep(footStepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio, GaitParam::FootStepNodes::refZmpState_enum::MIDDLE));
  footStepNodesList.push_back(calcDefaultDoubleSupportStep(footStepNodesList.back(), this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio), GaitParam::FootStepNodes::refZmpState_enum::MIDDLE)); // 末尾の両足支持期を延長. これがないと重心が目標位置に収束する前に返ってしまい, emergencyStepが無限に誘発する footGudedBalanceTimeを0.4程度に小さくすると収束が速くなるのでこの処理が不要になるのだが、今度はZ方向に振動しやすい

  o_footStepNodesList = footStepNodesList;
  return true;

}

// FootStepNodesListをdtすすめる
bool FootStepGenerator::procFootStepNodesList(const GaitParam& gaitParam, const double& dt,
                                              std::vector<GaitParam::FootStepNodes>& o_footStepNodesList, std::vector<pinocchio::SE3>& o_srcCoords, std::vector<pinocchio::SE3>& o_dstCoordsOrg, double& o_remainTimeOrg, std::vector<GaitParam::SwingState_enum>& o_swingState, double& o_elapsedTime, std::vector<bool>& o_prevSupportPhase, double& relLandingHeight) const{
  std::vector<GaitParam::FootStepNodes> footStepNodesList = gaitParam.footStepNodesList;
  std::vector<bool> prevSupportPhase = gaitParam.prevSupportPhase;
  double elapsedTime = gaitParam.elapsedTime;
  std::vector<pinocchio::SE3> srcCoords = gaitParam.srcCoords;
  std::vector<pinocchio::SE3> dstCoordsOrg = gaitParam.dstCoordsOrg;
  double remainTimeOrg = gaitParam.remainTimeOrg;
  std::vector<GaitParam::SwingState_enum> swingState = gaitParam.swingState;

  // 早づきしたらremainTimeにかかわらずすぐに次のnodeへ移る(remainTimeをdtにする). この機能が無いと少しでもロボットが傾いて早づきするとジャンプするような挙動になる.
  this->checkEarlyTouchDown(footStepNodesList, gaitParam, dt);

  // footStepNodesListを進める
  footStepNodesList[0].remainTime = std::max(0.0, footStepNodesList[0].remainTime - dt);
  elapsedTime += dt;
  for(int i=0;i<NUM_LEGS;i++) prevSupportPhase[i] = footStepNodesList[0].isSupportPhase[i];

  if(footStepNodesList[0].remainTime <= 0.0 && footStepNodesList.size() > 1){ // 次のfootStepNodesListのindexに移る.
    if(this->isModifyFootSteps && this->isStableGoStopMode){
      // footStepNodesList[0]で着地位置修正を行っていたら、footStepNodesListがemergencyStepNumのサイズになるまで歩くnodeが末尾に入る.
      this->checkStableGoStop(footStepNodesList, gaitParam);
    }

    // footStepNodesList[1]へ進む
    this->goNextFootStepNodesList(gaitParam, dt,
                                  footStepNodesList, srcCoords, dstCoordsOrg, remainTimeOrg, swingState, elapsedTime, relLandingHeight);
  }

  o_footStepNodesList = footStepNodesList;
  o_prevSupportPhase = prevSupportPhase;
  o_elapsedTime = elapsedTime;
  o_srcCoords = srcCoords;
  o_dstCoordsOrg = dstCoordsOrg;
  o_remainTimeOrg = remainTimeOrg;
  o_swingState = swingState;

  return true;
}

bool FootStepGenerator::calcFootSteps(const GaitParam& gaitParam, const double& dt,
                                      GaitParam::DebugData& debugData, //for Log
                                      std::vector<GaitParam::FootStepNodes>& o_footStepNodesList) const{
  std::vector<GaitParam::FootStepNodes> footStepNodesList = gaitParam.footStepNodesList;

  // goVelocityModeなら、進行方向に向けてfootStepNodesList[2] ~ footStepNodesList[goVelocityStepNum]の要素を機械的に計算してどんどん末尾appendしていく. cmdVelに応じてきまる
  if(this->isGoVelocityMode){
    // footStepNodesList[0]と[1]は変えない. footStepNodesList[1]以降で次に両足支持期になるときを探し、それ以降のstepを上書きする. 片足支持期の状態が末尾の要素になると、片足立ちで止まるという状態を意味することに注意
    for(int i=1;i<footStepNodesList.size();i++){
      if(footStepNodesList[i].isSupportPhase[RLEG] && footStepNodesList[i].isSupportPhase[LLEG]){
        footStepNodesList.resize(i+1);
        break;
      }
    }
    while(footStepNodesList.size() <= this->goVelocityStepNum){
      this->calcDefaultNextStep(footStepNodesList, gaitParam, gaitParam.cmdVel * this->defaultStepTime);
    }
  }

  if(this->isModifyFootSteps && this->isEmergencyStepMode){
    this->checkEmergencyStep(footStepNodesList, gaitParam);
  }

  if(this->isModifyFootSteps){
    this->modifyFootSteps(footStepNodesList, debugData, gaitParam);
  }

  o_footStepNodesList = footStepNodesList;

  return true;
}

bool FootStepGenerator::goNextFootStepNodesList(const GaitParam& gaitParam, double dt,
                                                std::vector<GaitParam::FootStepNodes>& footStepNodesList, std::vector<pinocchio::SE3>& srcCoords, std::vector<pinocchio::SE3>& dstCoordsOrg, double& remainTimeOrg, std::vector<GaitParam::SwingState_enum>& swingState, double& elapsedTime, double& relLandingHeight) const{
  // 今のgenCoordsとdstCoordsが異なるなら、将来のstepの位置姿勢をそれに合わせてずらす.
  // early touch downまたはlate touch downが発生していると、今のgenCoordsとdstCoordsが異なる.
  //   今のgenCoordsが正しい地形を表していることが期待されるので、将来のstepの位置姿勢をgenCoordsにあわせてずらしたくなる
  //   ところが実際には、genCoordsは位置制御指令関節角度から計算されるものなので、実際の地形とは誤差がある. (特に高さ方向に1cmくらい、実際よりも高いと誤認識する)
  //   そのため、両足支持期のときに、平らな地面を歩いているのに、後から地面についた足の位置がもとから地面についていた足の位置よりも高く逆運動学が解かれて、その結果身体が後から地面についた足の方に傾く. すると、その傾きによる重心位置の誤差を補正するために、特に左右方向に目標ZMPが8cmくらい大きくふれてしまう. これは許容できないので、基本的にはずらしてはいけない.
  // early touch downまたはlate touch downが発生していると、今のgenCoordsとdstCoordsが異なるが、無視して次のnodeの間にそのnodeのもとのdstCoordsにそのまま補間してゆっくり遷移することにする.
  // しかし、あまりにもずれが大きい場合は、遷移速度が無視できないほど大きくなるので、将来のstepの位置姿勢をgenCoordsにあわせてずらした方がよい
  // 触覚を精度良く利用したければ、「指令関節角度」という概念を使うのをやめて、actual angleに基づく手法に変えるべきかと
  // 基本的に、環境の地形は、触覚からは取得せず、視覚から取得する方針. 転倒等でロボットの身体が大きく傾いてearly touch down, late touch downした場合にのみ、やむを得ず触覚に基づいてずらす、という扱い.
  // 以上は関節角度制御時代の話、関節トルク制御でgenCoordsではなくactEEPoseを使う。

  //   footStepNodesList[1:]で一回でも遊脚になった以降は、位置とYawをずらす. footStepNodesList[1:]で最初に遊脚になる脚があるならその反対の脚の偏差にあわせてずらす. そうでないなら両脚の中心(+-defaultTranslatePos)の偏差にあわせてずらす
  //   footStepNodesList[1]が支持脚なら、遊脚になるまで、位置姿勢をその足の今の偏差にあわせてずらす.
  for(int i=1;i<footStepNodesList.size();i++){
    if(footStepNodesList[i].isSupportPhase[RLEG] && !footStepNodesList[i].isSupportPhase[LLEG]){ // LLEGが次に最初に遊脚になる
      pinocchio::SE3 origin = mathutil::orientCoordToAxis(footStepNodesList[i-1].dstCoords[RLEG], Eigen::Vector3d::UnitZ()); // generate frame. 一つ前のRLEGの位置を基準にずらす
      pinocchio::SE3 offsetedPos = mathutil::orientCoordToAxis(gaitParam.actEEPose[RLEG] * footStepNodesList[0].dstCoords[RLEG].inverse() * footStepNodesList[i-1].dstCoords[RLEG], Eigen::Vector3d::UnitZ()); // generate frame. 一つ前のRLEGはこの位置にずれている
      pinocchio::SE3 transform = offsetedPos * origin.inverse(); // generate frame
      if(transform.translation().norm() < this->contactModificationThreshold) transform = pinocchio::SE3::Identity(); // ズレが大きい場合のみずらす
      this->transformFutureSteps(footStepNodesList, i, transform); // footStepNodesList[1:]で一回でも遊脚になった以降の脚の、位置とYawをずらす
      break;
    }else if(footStepNodesList[i].isSupportPhase[LLEG] && !footStepNodesList[i].isSupportPhase[RLEG]){ // RLEGが次に最初に遊脚になる
      pinocchio::SE3 origin = mathutil::orientCoordToAxis(footStepNodesList[i-1].dstCoords[LLEG], Eigen::Vector3d::UnitZ()); // generate frame. 一つ前のLLEGの位置を基準にずらす
      pinocchio::SE3 offsetedPos = mathutil::orientCoordToAxis(gaitParam.actEEPose[LLEG] * footStepNodesList[0].dstCoords[LLEG].inverse() * footStepNodesList[i-1].dstCoords[LLEG], Eigen::Vector3d::UnitZ()); // generate frame. 一つ前のLLEGはこの位置にずれている
      pinocchio::SE3 transform = offsetedPos * origin.inverse(); // generate frame
      if(transform.translation().norm() < this->contactModificationThreshold) transform = pinocchio::SE3::Identity(); // ズレが大きい場合のみずらす
      this->transformFutureSteps(footStepNodesList, i, transform); // footStepNodesList[1:]で一回でも遊脚になった以降の脚の、位置とYawをずらす
      break;
    }else if(!footStepNodesList[i].isSupportPhase[LLEG] && !footStepNodesList[i].isSupportPhase[RLEG]){ // RLEGとLLEG同時に次に最初に遊脚になる
      pinocchio::SE3 rleg = footStepNodesList[i-1].dstCoords[RLEG]; // generate frame
      rleg.translation() -= rleg.rotation() * gaitParam.defaultTranslatePos[RLEG].value();
      pinocchio::SE3 lleg = footStepNodesList[i-1].dstCoords[LLEG];
      lleg.translation() -= lleg.rotation() * gaitParam.defaultTranslatePos[LLEG].value();
      pinocchio::SE3 origin = mathutil::orientCoordToAxis(mathutil::calcMidCoords({rleg, lleg}, {1.0, 1.0}), Eigen::Vector3d::UnitZ()); // 一つ前の両脚の中心の位置を基準にずらす
      pinocchio::SE3 offseted_rleg = gaitParam.actEEPose[RLEG] * footStepNodesList[0].dstCoords[RLEG].inverse() * footStepNodesList[i-1].dstCoords[RLEG]; // generate frame
      offseted_rleg.translation() -= rleg.rotation() * gaitParam.defaultTranslatePos[RLEG].value();
      pinocchio::SE3 offseted_lleg = gaitParam.actEEPose[LLEG] * footStepNodesList[0].dstCoords[LLEG].inverse() * footStepNodesList[i-1].dstCoords[LLEG];
      offseted_lleg.translation() -= lleg.rotation() * gaitParam.defaultTranslatePos[LLEG].value();
      pinocchio::SE3 offsetedPos = mathutil::orientCoordToAxis(mathutil::calcMidCoords({offseted_rleg, offseted_lleg}, {1.0, 1.0}), Eigen::Vector3d::UnitZ());
      pinocchio::SE3 transform = offsetedPos * origin.inverse(); // generate frame
      if(transform.translation().norm() < this->contactModificationThreshold) transform = pinocchio::SE3::Identity(); // ズレが大きい場合のみずらす
      this->transformFutureSteps(footStepNodesList, i, transform); // footStepNodesList[1:]で一回でも遊脚になった以降の脚の、位置とYawをずらす
      break;
    }
  }
  for(int i=0;i<NUM_LEGS;i++){
    if(footStepNodesList[1].isSupportPhase[i]){
      pinocchio::SE3 transform = gaitParam.actEEPose[i] * footStepNodesList[0].dstCoords[i].inverse(); // generate frame
      if(transform.translation().norm() < this->contactModificationThreshold) transform = pinocchio::SE3::Identity(); // ズレが大きい場合のみずらす
      this->transformCurrentSupportSteps(i, footStepNodesList, 1, transform); // 遊脚になるまで、位置姿勢をその足の今の偏差にあわせてずらす.
    }
  }

  // footStepNodesListをpop front
  footStepNodesList.erase(footStepNodesList.begin()); // vectorではなくlistにするべきかもしれないが、要素数がそんなに大きくないのでよしとする
  for(int i=0;i<NUM_LEGS;i++) {
    srcCoords[i] = gaitParam.actEEPose[i];
    dstCoordsOrg[i] = footStepNodesList[0].dstCoords[i];
    swingState[i] = GaitParam::LIFT_PHASE;
  }
  remainTimeOrg = footStepNodesList[0].remainTime;
  elapsedTime = 0.0;
  relLandingHeight = -1e15;

  return true;
}

void FootStepGenerator::transformFutureSteps(std::vector<GaitParam::FootStepNodes>& footStepNodesList, int index, const pinocchio::SE3& transform/*generate frame*/) const{
  for(int l=0;l<NUM_LEGS;l++){
    bool swinged = false;
    for(int i=index;i<footStepNodesList.size();i++){
      if(!footStepNodesList[i].isSupportPhase[l]) swinged = true;
      if(swinged) footStepNodesList[i].dstCoords[l] = transform * footStepNodesList[i].dstCoords[l];
    }
  }
}

void FootStepGenerator::transformFutureSteps(std::vector<GaitParam::FootStepNodes>& footStepNodesList, int index, const Eigen::Vector3d& transform/*generate frame*/) const{
  for(int l=0;l<NUM_LEGS;l++){
    bool swinged = false;
    for(int i=index;i<footStepNodesList.size();i++){
      if(!footStepNodesList[i].isSupportPhase[l]) swinged = true;
      if(swinged) footStepNodesList[i].dstCoords[l].translation() += transform;
    }
  }
}

// indexのsupportLegが次にswingするまでの間の位置を、generate frameでtransformだけ動かす
void FootStepGenerator::transformCurrentSupportSteps(int leg, std::vector<GaitParam::FootStepNodes>& footStepNodesList, int index, const pinocchio::SE3& transform/*generate frame*/) const{
  assert(0<=leg && leg < NUM_LEGS);
  for(int i=index;i<footStepNodesList.size();i++){
    if(!footStepNodesList[i].isSupportPhase[leg]) return;
    footStepNodesList[i].dstCoords[leg] = transform * footStepNodesList[i].dstCoords[leg];
  }
}

void FootStepGenerator::calcDefaultNextStep(std::vector<GaitParam::FootStepNodes>& footStepNodesList, const GaitParam& gaitParam, const Eigen::Vector3d& offset /*leg frame*/, bool stableStart) const{
  if(footStepNodesList.back().isSupportPhase[RLEG] && footStepNodesList.back().isSupportPhase[LLEG]){
    if(footStepNodesList.back().endRefZmpState == GaitParam::FootStepNodes::refZmpState_enum::MIDDLE){
      // offsetを、両足の中間からの距離と解釈する(これ以外のケースでは支持脚からの距離と解釈する)
      pinocchio::SE3 rleg = footStepNodesList[0].dstCoords[RLEG];
      rleg.translation() -= rleg.rotation() * gaitParam.defaultTranslatePos[RLEG].value();
      pinocchio::SE3 lleg = footStepNodesList[0].dstCoords[LLEG];
      lleg.translation() -= lleg.rotation() * gaitParam.defaultTranslatePos[LLEG].value();
      pinocchio::SE3 midCoords = mathutil::calcMidCoords(std::vector<pinocchio::SE3>{rleg, lleg}, std::vector<double>{1.0, 1.0});
      rleg = mathutil::orientCoordToAxis(rleg, Eigen::Vector3d::UnitZ());
      lleg = mathutil::orientCoordToAxis(lleg, Eigen::Vector3d::UnitZ());
      midCoords = mathutil::orientCoordToAxis(midCoords, Eigen::Vector3d::UnitZ());
      // どっちをswingしてもいいので、進行方向に近いLegをswingする
      Eigen::Vector2d rlegTolleg = (gaitParam.defaultTranslatePos[LLEG].value() - gaitParam.defaultTranslatePos[RLEG].value()).head<2>(); // leg frame
      if(rlegTolleg.dot(offset.head<2>()) > 0) { // LLEGをswingする
        if(stableStart && footStepNodesList.size() == 1) { // 現在の時刻から突然refZmpTrajが変化すると、大きなZMP入力変化が必要になる. いまの位置でrefZmpTrajをthis->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio)の間とめる
          footStepNodesList.push_back(calcDefaultDoubleSupportStep(footStepNodesList.back(), this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio), GaitParam::FootStepNodes::refZmpState_enum::MIDDLE));
        }
        // this->defaultStepTime * this->defaultDoubleSupportRatioの間で次の支持脚側に動かす
        if(footStepNodesList.size() <= 2) { // 末尾に両足支持期を追加
          footStepNodesList.push_back(calcDefaultDoubleSupportStep(footStepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio, GaitParam::FootStepNodes::refZmpState_enum::RLEG));
        }else{ // 末尾の両足支持期を直接編集
          footStepNodesList.back().remainTime = this->defaultStepTime * this->defaultDoubleSupportRatio; // 両足支持期を延長 or 短縮
          footStepNodesList.back().endRefZmpState = GaitParam::FootStepNodes::refZmpState_enum::RLEG;
        }
        pinocchio::SE3 rlegTomidCoords = rleg.inverse() * midCoords;
        Eigen::Vector3d offset_rlegLocal;
        offset_rlegLocal.head<2>() = rlegTomidCoords.translation().head<2>() + (rlegTomidCoords.rotation() * Eigen::Vector3d(offset[0],offset[1],0.0)).head<2>();
        offset_rlegLocal[2] = mathutil::rpyFromRot(rlegTomidCoords.rotation())[2] + offset[2];
        footStepNodesList.push_back(calcDefaultSwingStep(LLEG, footStepNodesList.back(), gaitParam, offset_rlegLocal)); // LLEGをswingする
        footStepNodesList.push_back(calcDefaultDoubleSupportStep(footStepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio, GaitParam::FootStepNodes::refZmpState_enum::LLEG));
      }else{ // RLEGをswingする
        if(stableStart && footStepNodesList.size() == 1) { // 現在の時刻から突然refZmpTrajが変化すると、大きなZMP入力変化が必要になる. いまの位置でrefZmpTrajをthis->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio)の間とめる
          footStepNodesList.push_back(calcDefaultDoubleSupportStep(footStepNodesList.back(), this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio), GaitParam::FootStepNodes::refZmpState_enum::MIDDLE));
        }
        // this->defaultStepTime * this->defaultDoubleSupportRatioの間で次の支持脚側に動かす
        if(footStepNodesList.size() <= 2) { // 末尾に両足支持期を追加
          footStepNodesList.push_back(calcDefaultDoubleSupportStep(footStepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio, GaitParam::FootStepNodes::refZmpState_enum::LLEG));
        }else{ // 末尾の両足支持期を直接編集
          footStepNodesList.back().remainTime = this->defaultStepTime * this->defaultDoubleSupportRatio; // 両足支持期を延長 or 短縮
          footStepNodesList.back().endRefZmpState = GaitParam::FootStepNodes::refZmpState_enum::LLEG;
        }
        pinocchio::SE3 llegTomidCoords = lleg.inverse() * midCoords;
        Eigen::Vector3d offset_llegLocal;
        offset_llegLocal.head<2>() = llegTomidCoords.translation().head<2>() + (llegTomidCoords.rotation() * Eigen::Vector3d(offset[0],offset[1],0.0)).head<2>();
        offset_llegLocal[2] = mathutil::rpyFromRot(llegTomidCoords.rotation())[2] + offset[2];
        footStepNodesList.push_back(calcDefaultSwingStep(RLEG, footStepNodesList.back(), gaitParam, offset_llegLocal)); // RLEGをswingする
        footStepNodesList.push_back(calcDefaultDoubleSupportStep(footStepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio, GaitParam::FootStepNodes::refZmpState_enum::RLEG));
      }
    }else if(footStepNodesList.back().endRefZmpState == GaitParam::FootStepNodes::refZmpState_enum::LLEG){ // 前回LLEGをswingした
        footStepNodesList.push_back(calcDefaultSwingStep(RLEG, footStepNodesList.back(), gaitParam, offset)); // RLEGをswingする
        footStepNodesList.push_back(calcDefaultDoubleSupportStep(footStepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio, GaitParam::FootStepNodes::refZmpState_enum::RLEG));
    }else{ // 前回RLEGをswingした
        footStepNodesList.push_back(calcDefaultSwingStep(LLEG, footStepNodesList.back(), gaitParam, offset)); // LLEGをswingする
        footStepNodesList.push_back(calcDefaultDoubleSupportStep(footStepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio, GaitParam::FootStepNodes::refZmpState_enum::LLEG));
    }
  }else if(footStepNodesList.back().isSupportPhase[RLEG] && !footStepNodesList.back().isSupportPhase[LLEG]){ // LLEGが浮いている
    footStepNodesList.push_back(calcDefaultSwingStep(LLEG, footStepNodesList.back(), gaitParam, offset, true)); // LLEGをswingする. startWithSingleSupport
    footStepNodesList.push_back(calcDefaultDoubleSupportStep(footStepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio, GaitParam::FootStepNodes::refZmpState_enum::LLEG));
  }else if(!footStepNodesList.back().isSupportPhase[RLEG] && footStepNodesList.back().isSupportPhase[LLEG]){ // RLEGが浮いている
    footStepNodesList.push_back(calcDefaultSwingStep(RLEG, footStepNodesList.back(), gaitParam, offset, true)); // RLEGをswingする. startWithSingleSupport
    footStepNodesList.push_back(calcDefaultDoubleSupportStep(footStepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio, GaitParam::FootStepNodes::refZmpState_enum::RLEG));
  }// footStepNodesListの末尾の要素が両方falseであることは無い
}

GaitParam::FootStepNodes FootStepGenerator::calcDefaultSwingStep(const int& swingLeg, const GaitParam::FootStepNodes& footStepNodes, const GaitParam& gaitParam, const Eigen::Vector3d& offset, bool startWithSingleSupport) const{
  GaitParam::FootStepNodes fs;
  int supportLeg = (swingLeg == RLEG) ? LLEG : RLEG;

  pinocchio::SE3 transform = pinocchio::SE3::Identity(); // supportLeg相対(Z軸は鉛直)での次のswingLegの位置
  double theta = mathutil::clamp(offset[2],this->defaultStrideLimitationMinTheta[swingLeg],this->defaultStrideLimitationMaxTheta[swingLeg]);
  transform.rotation() = Eigen::Matrix3d(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
  transform.translation() = - gaitParam.defaultTranslatePos[supportLeg].value() + Eigen::Vector3d(offset[0], offset[1], 0.0) + transform.rotation() * gaitParam.defaultTranslatePos[swingLeg].value();
  std::vector<Eigen::Vector3d> strideLimitationHull = this->calcRealStrideLimitationHull(swingLeg, theta, gaitParam.legHull, gaitParam.defaultTranslatePos, this->defaultStrideLimitationHull);
  transform.translation() = mathutil::calcNearestPointOfHull(transform.translation(), strideLimitationHull);

  fs.dstCoords[supportLeg] = footStepNodes.dstCoords[supportLeg];
  pinocchio::SE3 prevOrigin = mathutil::orientCoordToAxis(footStepNodes.dstCoords[supportLeg], Eigen::Vector3d::UnitZ());
  fs.dstCoords[swingLeg] = prevOrigin * transform;
  fs.isSupportPhase[supportLeg] = true;
  fs.isSupportPhase[swingLeg] = false;
  fs.remainTime = this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio);
  fs.endRefZmpState = (supportLeg == RLEG) ? GaitParam::FootStepNodes::refZmpState_enum::RLEG : GaitParam::FootStepNodes::refZmpState_enum::LLEG;
  if(!startWithSingleSupport) fs.stepHeight[swingLeg] = {this->defaultStepHeight,this->defaultStepHeight};
  else fs.stepHeight[swingLeg] = {0.0,this->defaultStepHeight};
  fs.touchVel[swingLeg] = this->touchVel;
  fs.goalOffset[swingLeg] = 0.0;
  return fs;
}

GaitParam::FootStepNodes FootStepGenerator::calcDefaultDoubleSupportStep(const GaitParam::FootStepNodes& footStepNodes, double doubleSupportTime, GaitParam::FootStepNodes::refZmpState_enum endRefZmpState) const{
  GaitParam::FootStepNodes fs;
  for(int i=0;i<NUM_LEGS;i++){
    fs.dstCoords[i] = footStepNodes.dstCoords[i];
    fs.isSupportPhase[i] = true;
    fs.stepHeight[i] = {0.0,0.0};
  }
  fs.remainTime = doubleSupportTime;
  fs.endRefZmpState = endRefZmpState;
  return fs;
}

void FootStepGenerator::modifyFootSteps(std::vector<GaitParam::FootStepNodes>& footStepNodesList, // input & output
                                        GaitParam::DebugData& debugData, //for Log
                                        const GaitParam& gaitParam) const{
  // 現在片足支持期で、次が両足支持期であるときのみ、行う
  if(!(footStepNodesList.size() > 1 &&
       (footStepNodesList[1].isSupportPhase[RLEG] && footStepNodesList[1].isSupportPhase[LLEG]) &&
       ((footStepNodesList[0].isSupportPhase[RLEG] && !footStepNodesList[0].isSupportPhase[LLEG]) || (!footStepNodesList[0].isSupportPhase[RLEG] && footStepNodesList[0].isSupportPhase[LLEG]))))
     return;

  // CapturePointが遊脚の着地位置に行かない場合には行わない
  if((!footStepNodesList[0].isSupportPhase[RLEG] && footStepNodesList[1].endRefZmpState != GaitParam::FootStepNodes::refZmpState_enum::RLEG) ||
     (!footStepNodesList[0].isSupportPhase[LLEG] && footStepNodesList[1].endRefZmpState != GaitParam::FootStepNodes::refZmpState_enum::LLEG))
    return;


  // 残り時間がoverwritableRemainTimeを下回っている場合、着地位置時間修正を行わない.
  if(footStepNodesList[0].remainTime < this->overwritableRemainTime) return;

  // one step capturabilityに基づき、footStepNodesList[0]のremainTimeとdstCoordsを修正する.
  int swingLeg = footStepNodesList[0].isSupportPhase[RLEG] ? LLEG : RLEG;
  int supportLeg = (swingLeg == RLEG) ? LLEG : RLEG;
  pinocchio::SE3 swingPose = gaitParam.actEEPose[swingLeg];
  pinocchio::SE3 supportPose = gaitParam.actEEPose[supportLeg];
  pinocchio::SE3 supportPoseHorizontal = mathutil::orientCoordToAxis(supportPose, Eigen::Vector3d::UnitZ());

  // stopCurrentPositionが発動しているなら、着地位置時間修正を行っても無駄なので行わない
  if(footStepNodesList[0].stopCurrentPosition[swingLeg]) return;

  // dx = w ( x - z - l)
  Eigen::Vector3d actDCM = gaitParam.actCog + gaitParam.actCogVel.value() / gaitParam.omega;

  /*
    capturable: ある時刻t(overwritableMinTime<=t<=overwritableMaxTime)が存在し、時刻tに着地すれば転倒しないような着地位置.
    reachable: ある時刻t(overwritableMinTime<=t<=overwritableMaxTime)が存在し、今の脚の位置からの距離が時刻tに着地することができる範囲である着地位置.
    strideLimitation: overwritableStrideLimitationHullの範囲内の着地位置(自己干渉・IKの考慮が含まれる).
    steppable: 着地可能な地形であるような着地位置

    * capturableとreachableの積集合を考えるときは、各時刻のtごとで考える

    優先度(小さいほうが高い)
    1. strideLimitation: 絶対満たす
    1. reachable: 絶対満たす
    2. steppable: 達成不可の場合や、着地可能領域が与えられていない場合は、考慮しない
    3. capturable: 達成不可の場合は、時間が速い方優先(次の一歩に期待). 複数ある場合は可能な限り近い位置. (角運動量 TODO)
    4. もとの着地位置(dstCoordsOrg): 達成不可の場合は、各hullの中の最も近い位置をそれぞれ求めて、着地位置修正前の進行方向(遊脚のsrcCoordsからの方向)に最も進むもの優先 (支持脚からの方向にすると、横歩き時に後ろ足の方向が逆になってしまう)
    5. もとの着地時刻(remainTimeOrg): 達成不可の場合は、可能な限り近い時刻
   */

  std::vector<std::pair<std::vector<Eigen::Vector3d>, double> > candidates; // first: generate frame. 着地領域(convex Hull). second: 着地時刻. サイズが0になることはない

  // 1. strideLimitation と reachable
  {
    std::vector<double> samplingTimes;
    samplingTimes.push_back(footStepNodesList[0].remainTime);
    double minTime = std::max(this->overwritableMinTime, this->overwritableMinStepTime - gaitParam.elapsedTime); // 次indexまでの残り時間がthis->overwritableMinTimeを下回るようには着地時間修正を行わない. 現index開始時からの経過時間がthis->overwritableStepMinTimeを下回るようには着地時間修正を行わない.
    minTime = std::min(minTime, footStepNodesList[0].remainTime); // もともと下回っている場合には、その値を下回るようには着地時刻修正を行わない.
    double maxTime = std::max(this->overwritableMaxStepTime - gaitParam.elapsedTime, minTime); // 現index開始時からの経過時間がthis->overwritableStepMaxTimeを上回るようには着地時間修正を行わない.
    maxTime = std::max(maxTime, footStepNodesList[0].remainTime); // もともと上回っている場合には、その値を上回るようには着地時刻修正を行わない.
    int sample = std::max(10, int((maxTime - minTime) / 0.1));
    for(int i=0;i<=sample;i++) {
      double t = minTime + (maxTime - minTime) / sample * i;
      if(t != footStepNodesList[0].remainTime) samplingTimes.push_back(t);
    }

    std::vector<Eigen::Vector3d> strideLimitationHull; // generate frame. overwritableStrideLimitationHullの範囲内の着地位置(自己干渉・IKの考慮が含まれる). Z成分には0を入れる
    {
      double theta = mathutil::rpyFromRot(mathutil::orientCoordToAxis(supportPoseHorizontal.rotation().transpose() * footStepNodesList[0].dstCoords[swingLeg].rotation(), Eigen::Vector3d::Zero()))[2]; // supportLeg(水平)相対のdstCoordsのyaw
      std::vector<Eigen::Vector3d> strideLimitationHullSupportLegLocal = this->calcRealStrideLimitationHull(swingLeg, theta, gaitParam.legHull, gaitParam.defaultTranslatePos, this->overwritableStrideLimitationHull); // support leg frame(水平)
      strideLimitationHull.reserve(strideLimitationHullSupportLegLocal.size());
      for(int i=0;i<strideLimitationHullSupportLegLocal.size();i++){
        Eigen::Vector3d p = (supportPoseHorizontal * pinocchio::SE3(Eigen::Matrix3d::Identity(), strideLimitationHullSupportLegLocal[i])).translation();
        strideLimitationHull.emplace_back(p[0],p[1],0.0);
      }
    }

    for(int i=0;i<samplingTimes.size();i++){
      double t = samplingTimes[i];
      std::vector<Eigen::Vector3d> reachableHull; // generate frame. 今の脚の位置からの距離が時刻tに着地することができる範囲. Z成分には0を入れる
      int segment = 8;
      for(int j=0; j < segment; j++){
        reachableHull.emplace_back(swingPose.translation()[0] + this->overwritableMaxSwingVelocity * t * std::cos(2 * M_PI / segment * j),
                                   swingPose.translation()[1] + this->overwritableMaxSwingVelocity * t * std::sin(2 * M_PI / segment * j),
                                   0.0);
      }
      std::vector<Eigen::Vector3d> hull = mathutil::calcIntersectConvexHull(reachableHull, strideLimitationHull);
      if(hull.size() > 0) candidates.emplace_back(hull, t);
    }

    if(candidates.size() == 0) candidates.emplace_back(std::vector<Eigen::Vector3d>{footStepNodesList[0].dstCoords[swingLeg].translation()}, footStepNodesList[0].remainTime); // まず起こらないと思うが念の為

    debugData.strideLimitationHull = strideLimitationHull; // for debeg
  }
  // std::cerr << "strideLimitation と reachable" << std::endl;
  // std::cerr << candidates << std::endl;

  // 2. steppable: 達成不可の場合や、着地可能領域が与えられていない場合は、考慮しない
  // TODO. 高低差と時間の関係
  {
    std::vector<std::pair<std::vector<Eigen::Vector3d>, double> > steppableHulls; // generate frame. first: visionに基づく着地可能領域. Z成分には0を入れる.  second: 高さ
    for(int i=0;i<gaitParam.steppableRegion.size();i++){
      // 空のHullは除外
      // overwritableMaxLandingHeightとoverwritableMinLandingHeightを満たさないregionは除外.
      if(gaitParam.steppableRegion[i].size() > 0 &&
         (gaitParam.steppableHeight[i] - supportPose.translation()[2] <= this->overwritableMaxLandingHeight) &&
         (gaitParam.steppableHeight[i] - supportPose.translation()[2] >= this->overwritableMinLandingHeight)){
        std::vector<Eigen::Vector3d> steppableHull;
        steppableHull.reserve(gaitParam.steppableRegion[i].size());
        for(int j=0;j<gaitParam.steppableRegion[i].size();j++) steppableHull.emplace_back(gaitParam.steppableRegion[i][j][0],gaitParam.steppableRegion[i][j][1],0.0);
        steppableHulls.emplace_back(steppableHull, gaitParam.steppableHeight[i]);
      }
    }

    std::vector<std::pair<std::vector<Eigen::Vector3d>, double> > nextCandidates;
    for(int i=0;i<candidates.size();i++){
      for(int j=0;j<steppableHulls.size();j++){
        // overwritableMaxGroundZVelocityを満たさないregionは除外
        if(std::abs(steppableHulls[j].second - gaitParam.actEEPose[swingLeg].translation()[2]) > 0.2 && candidates[i].second < 0.8) continue; // TODO
        if(std::abs(steppableHulls[j].second - gaitParam.srcCoords[swingLeg].translation()[2]) > (gaitParam.elapsedTime + candidates[i].second) * this->overwritableMaxSrcGroundZVelocity) continue;
        std::vector<Eigen::Vector3d> hull = mathutil::calcIntersectConvexHull(candidates[i].first, steppableHulls[j].first);
        if(hull.size() > 0) nextCandidates.emplace_back(hull, candidates[i].second);
      }
    }
    if(nextCandidates.size() > 0) candidates = nextCandidates;
    // nextCandidates.size() == 0の場合は、達成不可の場合や、着地可能領域が与えられていない場合なので、candidateの絞り込みを行わない
  }
  // std::cerr << "steppable" << std::endl;
  // std::cerr << candidates << std::endl;

  // 3. capturable: 達成不可の場合は、時間が速い方優先(次の一歩に期待). 複数ある場合は可能な限り近い位置. (角運動量 TODO)
  // 次の両足支持期終了時に入るケースでもOKにしたい
  if (this->safeLegHull[supportLeg].size() == 4) { // for log
    for(int i=0;i<this->safeLegHull[supportLeg].size();i++){
      Eigen::Vector3d zmp = (supportPose * pinocchio::SE3(Eigen::Matrix3d::Identity(), this->safeLegHull[supportLeg][i])).translation();// generate frame
      debugData.cpViewerLog[i*2+0] = zmp[0];
      debugData.cpViewerLog[i*2+1] = zmp[1];
    }
  }
  {
    std::vector<std::vector<Eigen::Vector3d> > capturableHulls; // 要素数と順番はcandidatesに対応
    for(int i=0;i<candidates.size();i++){
      std::vector<Eigen::Vector3d> capturableVetices; // generate frame. 時刻tに着地すれば転倒しないような着地位置. Z成分には0を入れる
      for(double t = candidates[i].second; t <= candidates[i].second + footStepNodesList[1].remainTime; t += footStepNodesList[1].remainTime){ // 接地する瞬間と、次の両足支持期の終了時. 片方だけだと特に横歩きのときに厳しすぎる. refZmpTrajを考えると本当は次の両足支持期の終了時のみを使うのが望ましい. しかし、位置制御成分が大きいロボットだと、力分配しているつもりがなくても接地している足から力を受けるので、接地する瞬間も含めてしまってそんなに問題はない?
        for(int j=0;j<this->safeLegHull[supportLeg].size();j++){
          Eigen::Vector3d zmp = (supportPose * pinocchio::SE3(Eigen::Matrix3d::Identity(), this->safeLegHull[supportLeg][j])).translation();// generate frame
          Eigen::Vector3d endDCM = (actDCM - zmp - gaitParam.l) * std::exp(gaitParam.omega * t) + zmp + gaitParam.l; // generate frame. 着地時のDCM
          // for(int k=0;k<this->safeLegHull[swingLeg].size();k++){
          //   Eigen::Vector3d p = endDCM - gaitParam.l - footStepNodesList[0].dstCoords[swingLeg].rotation() * this->safeLegHull[swingLeg][k]; // こっちのほうが厳密であり、着地位置時刻修正を最小限にできるが、ロバストさに欠ける
          //   capturableVetices.emplace_back(p[0], p[1], 0.0);
          // }
          Eigen::Vector3d p = endDCM - gaitParam.l - footStepNodesList[0].dstCoords[swingLeg].rotation() * gaitParam.copOffset[swingLeg].value();
          capturableVetices.emplace_back(p[0], p[1], 0.0);
        }
      }
      capturableHulls.push_back(mathutil::calcConvexHull(capturableVetices)); // generate frame. 時刻tに着地すれば転倒しないような着地位置. Z成分には0を入れる
    }

    std::vector<std::pair<std::vector<Eigen::Vector3d>, double> > nextCandidates;
    for(int i=0;i<candidates.size();i++){
      std::vector<Eigen::Vector3d> hull = mathutil::calcIntersectConvexHull(candidates[i].first, capturableHulls[i]);
      if(hull.size() > 0) nextCandidates.emplace_back(hull, candidates[i].second);
    }
    if(nextCandidates.size() > 0) candidates = nextCandidates;
    else{
      // 達成不可の場合は、時間が速い方優先(次の一歩に期待). 複数ある場合は可能な限り近い位置.
      //   どうせこの一歩ではバランスがとれないので、位置よりも速く次の一歩に移ることを優先したほうが良い
      double minTime = std::numeric_limits<double>::max();
      double minDistance = std::numeric_limits<double>::max();
      Eigen::Vector3d minp;
      for(int i=0;i<candidates.size();i++){
        if(candidates[i].second <= minTime){
          std::vector<Eigen::Vector3d> p, q;
          double distance = mathutil::calcNearestPointOfTwoHull(candidates[i].first, capturableHulls[i], p, q); // candidates[i].first, capturableHulls[i]は重なっていない・接していない
          if(candidates[i].second < minTime ||
             (candidates[i].second == minTime && distance < minDistance)){
            minTime = candidates[i].second;
            minDistance = distance;
            nextCandidates.clear();
            nextCandidates.emplace_back(p,minTime); // pは、最近傍が点の場合はその点が入っていて、最近傍が線分の場合はその線分の両端点が入っている
          }else if(candidates[i].second == minTime && distance == minDistance){
            nextCandidates.emplace_back(p,minTime);
          }
        }
      }
      candidates = nextCandidates;
    }

    debugData.capturableHulls = capturableHulls; // for debug
  }

  // std::cerr << "capturable" << std::endl;
  // std::cerr << candidates << std::endl;

  // 4. もとの着地位置: 達成不可の場合は、もとの着地位置よりも着地位置修正前の進行方向(遊脚のsrcCoordsからの方向)に進むなかで、もとの着地位置に最も近いもの. それがなければもとの着地位置の最も近いもの. (支持脚からの方向にすると、横歩き時に後ろ足の方向が逆になってしまう)
  debugData.cpViewerLog[8] = gaitParam.dstCoordsOrg[swingLeg].translation()[0]; //for log もとの目標着地位置
  debugData.cpViewerLog[9] = gaitParam.dstCoordsOrg[swingLeg].translation()[1]; //for log もとの目標着地位置
  {
    std::vector<std::pair<std::vector<Eigen::Vector3d>, double> > nextCandidates;
    for(int i=0;i<candidates.size();i++){
      if(mathutil::isInsideHull(gaitParam.dstCoordsOrg[swingLeg].translation(), candidates[i].first)){
        nextCandidates.emplace_back(std::vector<Eigen::Vector3d>{gaitParam.dstCoordsOrg[swingLeg].translation()},candidates[i].second);
      }
    }
    if(nextCandidates.size() > 0) candidates = nextCandidates;
    else{ // 達成不可の場合
      Eigen::Vector3d dir = gaitParam.dstCoordsOrg[swingLeg].translation() - gaitParam.srcCoords[swingLeg].translation(); dir[2] = 0.0;
      if(dir.norm() > 1e-2){ //着地位置修正前の進行方向(遊脚のsrcCoordsからの方向)に進むなかで、もとの着地位置に最も近いもの
        dir = dir.normalized();
        std::vector<Eigen::Vector3d> motionDirHull; // generate frame. 進行方向全体を覆うHull
        {
          pinocchio::SE3 p = pinocchio::SE3::Identity();
          p.translation() = gaitParam.dstCoordsOrg[swingLeg].translation() - dir * 1e-3/*数値誤差に対応するための微小なマージン*/; p.translation()[2] = 0.0;
          p.rotation() = (Eigen::Matrix3d() << dir[0], -dir[1], 0.0,
                                            dir[1], dir[0] , 0.0,
                                            0.0,    0.0,     1.0).finished();
          motionDirHull.push_back((p * pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, -1e10, 0.0))).translation());
          motionDirHull.push_back((p * pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1e10, -1e10, 0.0))).translation());
          motionDirHull.push_back((p * pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1e10, 1e10, 0.0))).translation());
          motionDirHull.push_back((p * pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 1e10, 0.0))).translation());
        }
        double minDistance = std::numeric_limits<double>::max();
        for(int i=0;i<candidates.size();i++){
          std::vector<Eigen::Vector3d> hull = mathutil::calcIntersectConvexHull(candidates[i].first, motionDirHull);
          if(hull.size() == 0) continue;
          Eigen::Vector3d p = mathutil::calcNearestPointOfHull(gaitParam.dstCoordsOrg[swingLeg].translation(), hull);
          double distance = (p - gaitParam.dstCoordsOrg[swingLeg].translation()).norm();
          if(distance < minDistance){
            minDistance = distance;
            nextCandidates.clear();
            nextCandidates.emplace_back(std::vector<Eigen::Vector3d>{p},candidates[i].second);
          }else if (distance == minDistance){
            nextCandidates.emplace_back(std::vector<Eigen::Vector3d>{p}, candidates[i].second);
          }
        }
      }
      if(nextCandidates.size() > 0) candidates = nextCandidates;
      else{ // もとの着地位置に最も近いもの.
        double minDistance = std::numeric_limits<double>::max();
        for(int i=0;i<candidates.size();i++){
          Eigen::Vector3d p = mathutil::calcNearestPointOfHull(gaitParam.dstCoordsOrg[swingLeg].translation(), candidates[i].first);
          double distance = (p - gaitParam.dstCoordsOrg[swingLeg].translation()).norm();
          if(distance < minDistance){
            minDistance = distance;
            nextCandidates.clear();
            nextCandidates.emplace_back(std::vector<Eigen::Vector3d>{p}, candidates[i].second);
          }else if (distance == minDistance){
            nextCandidates.emplace_back(std::vector<Eigen::Vector3d>{p}, candidates[i].second);
          }
        }
        candidates = nextCandidates;
      }
    }
  }

  // std::cerr << "pos" << std::endl;
  // std::cerr << candidates << std::endl;

  // 5. もとの着地時刻(remainTimeOrg): 達成不可の場合は、可能な限り近い時刻
  {
    std::vector<std::pair<std::vector<Eigen::Vector3d>, double> > nextCandidates;
    double targetRemainTime = gaitParam.remainTimeOrg - gaitParam.elapsedTime; // 負になっているかもしれない.
    double minDiffTime = std::numeric_limits<double>::max();
    for(int i=0;i<candidates.size();i++){
      double diffTime = std::abs(candidates[i].second - targetRemainTime);
      if(diffTime < minDiffTime){
        minDiffTime = diffTime;
        nextCandidates.clear();
        nextCandidates.push_back(candidates[i]);
      }else if(diffTime == minDiffTime){
        minDiffTime = diffTime;
        nextCandidates.push_back(candidates[i]);
      }
    }
    candidates = nextCandidates;
  }

  // std::cerr << "time" << std::endl;
  // std::cerr << candidates << std::endl;

  // 修正を適用
  Eigen::Vector3d nextDstCoordsPos = candidates[0].first[0]; // generate frame
  // Z高さはrelLandingHeightから受け取った値を用いる. relLandingHeightが届いていなければ、修正しない.
  if (gaitParam.swingState[swingLeg] != GaitParam::DOWN_PHASE && gaitParam.relLandingHeight > -1e+10) {
    nextDstCoordsPos[2] = std::min(supportPose.translation()[2] + this->overwritableMaxLandingHeight, std::max(supportPose.translation()[2] + this->overwritableMinLandingHeight, gaitParam.relLandingHeight));   //landingHeightから受け取った値を用いて着地高さを変更
  }else{
    nextDstCoordsPos[2] = footStepNodesList[0].dstCoords[swingLeg].translation()[2];
  }
  Eigen::Vector3d displacement = nextDstCoordsPos - footStepNodesList[0].dstCoords[swingLeg].translation(); // generate frame
  this->transformFutureSteps(footStepNodesList, 0, displacement);
  footStepNodesList[0].remainTime = candidates[0].second;

  //landingHeightから受け取った値を用いて着地姿勢を変更
  if (gaitParam.swingState[swingLeg] != GaitParam::DOWN_PHASE && gaitParam.relLandingHeight > -1e+10) {
    footStepNodesList[0].dstCoords[swingLeg].rotation() = mathutil::orientCoordToAxis(gaitParam.dstCoordsOrg[swingLeg].rotation(), gaitParam.relLandingNormal); // dstCoordsを毎周期orientCoordToAxisすると計算誤差でだんだん変にずれてくるので、dstCoordsOrgを使う. この処理以外ではdstCoordsの傾きは変更されないという仮定がある.
  }
}

// 早づきしたらremainTimeが0になるまで今の位置で止める.
//   - この機能が無いと少しでもロボットが傾いて早づきするとジャンプするような挙動になる.
//   - remainTimeが0になるまで待たないと、本来の着地位置に着地したときに、DCMが次の着地位置まで移動しきっていなくて、かつ、支持脚状態が急激に変化するので、ZMP入力が急激に変化してしまう.
//   - ただし、この方法だと、早づきするときは転びそうになっているときなのですぐに次の一歩を踏み出さないと転んでしまうので、待つぶん弱い
// remainTimeが0になっても地面についていなかったら、remainTimeを少しずつ延長し着地位置を下方に少しずつオフセットさせる
//   - remainTimeが0のときには本来の着地位置に行くようにしないと、着地タイミングがrefZmpよりも常に早すぎ・遅すぎになるので良くない
//   - ただし、この方法だと、遅づきしたときに着地時刻が遅くなるのでDCMが移動しずぎてしまっているので、転びやすい.
void FootStepGenerator::checkEarlyTouchDown(std::vector<GaitParam::FootStepNodes>& footStepNodesList, const GaitParam& gaitParam, double dt) const{
  for(int i=0;i<NUM_LEGS;i++){
    actLegWrenchFilter[i].passFilter(gaitParam.actFSensorWrench[i], dt);
  }

  if(footStepNodesList.size() > 1){
    for(int leg = 0; leg<NUM_LEGS; leg++){
      if(!footStepNodesList[0].isSupportPhase[leg] && footStepNodesList[1].isSupportPhase[leg] && //現在swing期で次support期
         gaitParam.swingState[leg] == GaitParam::DOWN_PHASE && // DOWN_PHASE
         footStepNodesList[0].stopCurrentPosition[leg] == false){ // まだ地面についていない

        if(actLegWrenchFilter[leg].value()[2] > this->contactDetectionThreshold /*generate frame. ロボットが受ける力*/) {// 力センサの値が閾値以上
          footStepNodesList[0].stopCurrentPosition[leg] = true;
        }else if(footStepNodesList[0].remainTime <= dt && // remainTimeが0になる
                 footStepNodesList[0].touchVel[leg] > 0.0 && // touchVelが0ならいつまでもつかないのでgoaloffsetを適用しない
                 footStepNodesList[0].goalOffset[leg] > this->goalOffset){ // まだgoalOffsetまで下ろしていない
          footStepNodesList[0].remainTime += dt;
          footStepNodesList[0].goalOffset[leg] = std::max(this->goalOffset, footStepNodesList[0].goalOffset[leg] - footStepNodesList[0].touchVel[leg] * dt);
        }
      }
    }
  }
}

// emergengy step.
void FootStepGenerator::checkEmergencyStep(std::vector<GaitParam::FootStepNodes>& footStepNodesList, const GaitParam& gaitParam) const{
  // 現在静止状態で、CapturePointがsafeLegHullの外にあるなら、footStepNodesListがemergencyStepNumのサイズになるまで歩くnodeが末尾に入る.
  if(!(footStepNodesList.size() == 1 && footStepNodesList[0].remainTime == 0)) return; // static 状態でないなら何もしない

  std::vector<Eigen::Vector3d> supportVertices; // generate frame
  for(int i=0;i<NUM_LEGS;i++){
    if(footStepNodesList[0].isSupportPhase[i]){
      for(int j=0;j<this->safeLegHull[i].size();j++){
        supportVertices.push_back((gaitParam.actEEPose[i] * pinocchio::SE3(Eigen::Matrix3d::Identity(), this->safeLegHull[i][j])).translation()); // generate frame
      }
    }
  }
  std::vector<Eigen::Vector3d> supportHull = mathutil::calcConvexHull(supportVertices); // generate frame. Z成分はてきとう

  // dx = w ( x - z - l)
  Eigen::Vector3d actDCM = gaitParam.actCog + gaitParam.actCogVel.value() / gaitParam.omega; // generate frame
  Eigen::Vector3d actCMP = actDCM - gaitParam.l; // generate frame

  if(!mathutil::isInsideHull(actCMP, supportHull) || // supportHullに入っていない
     (actCMP.head<2>() - gaitParam.refZmpTraj[0].getStart().head<2>()).norm() > this->emergencyStepCpCheckMargin // 目標重心位置からの距離が閾値以上 (supportHullによるチェックだけだと両足支持の場合の左右方向の踏み出しがなかなか起こらず、左右方向の踏み出しは得意ではないため、踏み出しが起きたときには時既に遅しで転倒してしまう)
     ){
    Eigen::Vector3d dir = gaitParam.footMidCoords.value().rotation().transpose() * (actCMP - gaitParam.footMidCoords.value().translation()); // footmidcoords frame
    dir[2] = 0.0;
    if(dir.norm() > 0.0) dir = dir.normalized();
    while(footStepNodesList.size() <= this->emergencyStepNum){
      this->calcDefaultNextStep(footStepNodesList, gaitParam, 1e-6 * dir, false); // 現在両足で支持している場合、dirの方向の足を最初にswingする. 急ぐのでstableStart=false
    }
    // refZmpを両足の中心へ戻す
    footStepNodesList.push_back(calcDefaultDoubleSupportStep(footStepNodesList.back(), this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio), footStepNodesList.back().endRefZmpState));
    footStepNodesList.push_back(calcDefaultDoubleSupportStep(footStepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio, GaitParam::FootStepNodes::refZmpState_enum::MIDDLE));
    footStepNodesList.push_back(calcDefaultDoubleSupportStep(footStepNodesList.back(), this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio), GaitParam::FootStepNodes::refZmpState_enum::MIDDLE)); // 末尾の両足支持期を延長. これがないと重心が目標位置に収束する前に返ってしまい, emergencyStepが無限に誘発する footGudedBalanceTimeを0.4程度に小さくすると収束が速くなるのでこの処理が不要になるのだが、今度はZ方向に振動しやすい
  }
}

// Stable Go Stop
void FootStepGenerator::checkStableGoStop(std::vector<GaitParam::FootStepNodes>& footStepNodesList, const GaitParam& gaitParam) const{
  // 着地位置修正を行ったなら、footStepNodesListがemergencyStepNumのサイズになるまで歩くnodeが末尾に入る.
  for(int i=0;i<NUM_LEGS; i++){
    if((footStepNodesList[0].dstCoords[i].translation().head<2>() - gaitParam.dstCoordsOrg[i].translation().head<2>()).norm() > 0.01){ // 1cm以上着地位置修正を行ったなら
      while(footStepNodesList.size() >= 2 &&
            (footStepNodesList[footStepNodesList.size()-2].isSupportPhase[RLEG] && footStepNodesList[footStepNodesList.size()-2].isSupportPhase[LLEG]) &&
            (footStepNodesList[footStepNodesList.size()-1].isSupportPhase[RLEG] && footStepNodesList[footStepNodesList.size()-1].isSupportPhase[LLEG])){
        footStepNodesList.pop_back(); // 末尾に両足支持期が連続している場合、最初のものを除いて削除
      }
      while(footStepNodesList.size() < this->emergencyStepNum){
        // 歩を追加
        this->calcDefaultNextStep(footStepNodesList, gaitParam);
      }
      // refZmpを両足の中心へ戻す
      footStepNodesList.push_back(calcDefaultDoubleSupportStep(footStepNodesList.back(), this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio), footStepNodesList.back().endRefZmpState));
      footStepNodesList.push_back(calcDefaultDoubleSupportStep(footStepNodesList.back(), this->defaultStepTime * this->defaultDoubleSupportRatio, GaitParam::FootStepNodes::refZmpState_enum::MIDDLE));
      footStepNodesList.push_back(calcDefaultDoubleSupportStep(footStepNodesList.back(), this->defaultStepTime * (1.0 - this->defaultDoubleSupportRatio), GaitParam::FootStepNodes::refZmpState_enum::MIDDLE)); // 末尾の両足支持期を延長. これがないと重心が目標位置に収束する前に返ってしまい, emergencyStepが無限に誘発する footGudedBalanceTimeを0.4程度に小さくすると収束が速くなるのでこの処理が不要になるのだが、今度はZ方向に振動しやすい

      break;
    }
  }
}

std::vector<Eigen::Vector3d> FootStepGenerator::calcRealStrideLimitationHull(const int& swingLeg, const double& theta, const std::vector<std::vector<Eigen::Vector3d> >& legHull, const std::vector<mathutil::TwoPointInterpolator<Eigen::Vector3d> >& defaultTranslatePos, const std::vector<std::vector<Eigen::Vector3d> >& strideLimitationHull) const{
  std::vector<Eigen::Vector3d> realStrideLimitationHull = strideLimitationHull[swingLeg]; // 支持脚(水平)座標系. strideLimitationHullの要素数は1以上あることが保証されているという仮定

  int supportLeg = swingLeg == RLEG ? LLEG : RLEG;
  Eigen::Matrix3d swingR = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  {
    // legCollisionを考慮
    Eigen::Vector3d supportLegToSwingLeg = defaultTranslatePos[swingLeg].value() - defaultTranslatePos[supportLeg].value(); // 支持脚(水平)座標系.
    supportLegToSwingLeg[2] = 0.0;
    if(supportLegToSwingLeg.norm() > 0.0){
      Eigen::Vector3d supportLegToSwingLegDir = supportLegToSwingLeg.normalized();
      std::vector<Eigen::Vector3d> tmp;
      const std::vector<Eigen::Vector3d>& supportLegHull = legHull[supportLeg];
      double supportLegHullSize = mathutil::findExtreams(supportLegHull, supportLegToSwingLegDir, tmp);
      std::vector<Eigen::Vector3d> swingLegHull;
      for(int i=0;i<legHull[swingLeg].size();i++) swingLegHull.push_back(swingR * legHull[swingLeg][i]);
      double swingLegHullSize = mathutil::findExtreams(swingLegHull, - supportLegToSwingLegDir, tmp);
      double minDist = supportLegHullSize + this->legCollisionMargin + swingLegHullSize;
      std::vector<Eigen::Vector3d> minDistHull; // supportLegToSwingLeg方向にminDistを満たすHull
      {
        pinocchio::SE3 p = pinocchio::SE3::Identity();
        p.translation() = minDist * supportLegToSwingLegDir;
        p.rotation() = (Eigen::Matrix3d() << supportLegToSwingLegDir[0], -supportLegToSwingLegDir[1], 0.0,
                                          supportLegToSwingLegDir[1], supportLegToSwingLegDir[0] , 0.0,
                                          0.0,                        0.0,                         1.0).finished();
        minDistHull.push_back((p * pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, -1e10, 0.0))).translation());
        minDistHull.push_back((p * pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1e10, -1e10, 0.0))).translation());
        minDistHull.push_back((p * pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1e10, 1e10, 0.0))).translation());
        minDistHull.push_back((p * pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 1e10, 0.0))).translation());
      }

      std::vector<Eigen::Vector3d> nextRealStrideLimitationHull = mathutil::calcIntersectConvexHull(realStrideLimitationHull, minDistHull);
      if(nextRealStrideLimitationHull.size() > 0) realStrideLimitationHull = nextRealStrideLimitationHull;
    }
  }

  {
    // swingLegから見てもsupportLegから見てもStrideLimitationHullの中にあることを確認 (swing中の干渉を防ぐ)
    const std::vector<Eigen::Vector3d>& strideLimitationHullFromSupport = realStrideLimitationHull;
    std::vector<Eigen::Vector3d> strideLimitationHullFromSwing(realStrideLimitationHull.size());
    for(int i=0;i<realStrideLimitationHull.size();i++){
      strideLimitationHullFromSwing[i] = swingR * realStrideLimitationHull[i];
    }
    std::vector<Eigen::Vector3d> nextRealStrideLimitationHull = mathutil::calcIntersectConvexHull(strideLimitationHullFromSupport, strideLimitationHullFromSwing);
    if(nextRealStrideLimitationHull.size() > 0) realStrideLimitationHull = nextRealStrideLimitationHull;
  }

  return realStrideLimitationHull;
}
