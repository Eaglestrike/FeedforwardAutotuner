#include "Drive/AutoPath.h"

#include "iostream"

#include "Drive/DriveConstants.h"

#include <frc/smartdashboard/SmartDashboard.h>

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399
#endif

/**
 * Constructor
*/
AutoPath::AutoPath() : 
  m_curAng{0}, m_multiplier{0}, m_curAngVel{0}, m_prevTime{0}, m_prevTimeOdom{0},
  m_calcTrans{100}, m_calcAng{100}, m_curState{NOT_EXECUTING},
  m_kPPos{0}, m_kIPos{0}, m_kDPos{0}, m_kPAng{0}, m_kIAng{0}, m_kDAng{0}
{
  using namespace AutoConstants;
  SetPosPID(TRANS_KP, TRANS_KI, TRANS_KD);
  SetAngPID(ANG_KP, ANG_KI, ANG_KD);
}

/**
 * Adds a singular pose
 * 
 * @param pose Pose to add, an AutoPath::SwervePose object (use initializer list)
 * 
 * @note Robot will not take shortest path between angles, to make it take the shortest path,
 * add or subtract M_PI to angle
 * @note Units are m, rad, and s
*/
void AutoPath::AddPose(AutoPaths::SwervePose pose) {
  Pose2 poseTrans = {pose.time, {pose.x, pose.y}, {pose.vx, pose.vy}};
  Pose1 poseAng = {pose.time, {pose.ang}, {pose.angVel}};

  m_calcTrans.insertOrReplace(poseTrans);
  m_calcAng.insertOrReplace(poseAng);
}

/**
 * Adds a list of poses
 * 
 * @param poses Poses to add, a vector of AutoPath::SwervePose objects (use initializer list)
 * 
 * @note Robot will not take shortest path between angles, to make it take the shortest path,
 * add or subtract M_PI to angle
 * @note Units are m, rad, and s
*/
void AutoPath::AddPoses(std::vector<AutoPaths::SwervePose> poses) {
  for (auto pose : poses) {
    AddPose(pose);
  }
}

/**
 * Clears auto paths
*/
void AutoPath::ResetPath() {
  m_calcTrans = Hermite2{100};
  m_calcAng = Hermite1{100};
}

/**
 * Resets angle multiplier
*/
void AutoPath::ResetMultiplier() {
  if (Utils::NearZero(m_curAng, M_PI / 2)) {
    m_multiplier = 0;
  } else if (m_curAng < 0) {
    m_multiplier = 1;
  } else {
    m_multiplier = 0;
  }
}

/**
 * Sets position correction PID
 * 
 * @param kP p
 * @param kI i
 * @param kD d
*/
void AutoPath::SetPosPID(double kP, double kI, double kD) {
  m_kPPos = kP;
  m_kIPos = kI;
  m_kDPos = kD;
}

/**
 * Sets angular position correction PID
 * 
 * @param kP p
 * @param kI i
 * @param kD d
*/
void AutoPath::SetAngPID(double kP, double kI, double kD) {
  m_kPAng = kP;
  m_kIAng = kI;
  m_kDAng = kD;
}

/**
 * Stops auto path
*/
void AutoPath::Stop() {
  m_curState = NOT_EXECUTING;
}

/**
 * Starts auto path
*/
void AutoPath::StartMove() {
  m_expectFinish = m_calcTrans.getHighestTime();
  if(m_calcTrans.getHighestTime() == 0){
    Stop();
    return;
  }
  m_curState = EXECUTING_PATH;
  m_startTime = Utils::GetCurTimeS();
}

/**
 * Updates odometry
 * 
 * @note Make sure this is called every cycle
 * 
 * @param pos Current position
 * @param ang Current angle
*/
void AutoPath::UpdateOdom(vec::Vector2D curPos, double curAng, vec::Vector2D curWheelVel) {
  m_curPos = curPos;
  m_curWheelVel = curWheelVel;

  // update multiplier of angle
  double curTimeS = Utils::GetCurTimeS();
  double curAngSpeed = (curAng - m_curAng) / (curTimeS - m_prevTimeOdom);
  if (curAngSpeed < -AutoConstants::UNREASONABLE_ANG_SPEED && curAng < 0 && m_curAng > 0 && Utils::NearZero(curAng + M_PI, 0.2)) {
    // looping around counterclockwise
    m_multiplier++;
  } else if (curAngSpeed > AutoConstants::UNREASONABLE_ANG_SPEED && curAng > 0 && m_curAng < 0 && Utils::NearZero(curAng - M_PI, 0.2)) {
    // looping around clockwise
    m_multiplier--;
  }

  // frc::SmartDashboard::PutNumber("multiplier", m_multiplier);
  // frc::SmartDashboard::PutNumber("cur ang speed", curAngSpeed);

  m_prevTimeOdom = curTimeS;

  m_curAng = curAng;
}

/**
 * Periodic function
*/
void AutoPath::Periodic() {
  double curTime = Utils::GetCurTimeS();
  double deltaT = curTime - m_prevTime;

  switch (m_curState) {
    case NOT_EXECUTING:
      m_curVel = {0, 0};
      m_curAngVel = 0;
      break;
    case EXECUTING_PATH:
    {
      double relTime = curTime - m_startTime;
      double highestTime = m_calcTrans.getHighestTime();
      double lowestTime = m_calcTrans.getLowestTime();
      double getTime = std::clamp(relTime, lowestTime, highestTime);

      vec::Vector2D curExpectedPos = m_calcTrans.getPos(getTime);
      vec::Vector2D curExpectedVel = m_calcTrans.getVel(getTime);

      vec::Vector2D correctionVel = GetPIDTrans(deltaT, curExpectedPos, curExpectedVel);
      vec::Vector2D totalVel = curExpectedVel + correctionVel;

      double curExpectedAng = m_calcAng.getPos(getTime)[0];
      double curExpectedAngVel = m_calcAng.getVel(getTime)[0];

      double correctionAngVel = GetPIDAng(deltaT, curExpectedAng); 
      double totalAngVel = curExpectedAngVel + correctionAngVel;

      m_curVel = totalVel;
      m_curAngVel = totalAngVel;

      if (AtTarget()) {
        m_curState = AT_TARGET;
      }
      break;
    }
    case AT_TARGET:
    {
      m_curVel = {0, 0};
      m_curAngVel = 0;

      if (!AtTarget()) {
        m_curState = NOT_EXECUTING;
      }

      break;
    }
  }

  m_prevTime = curTime;
}

/**
 * Returns whether robot  is at the target
 * 
 * Assumes autoconstant error tolerance
 * 
 * @returns Wheter robot is where it is supposed to be 
*/
bool AutoPath::AtTarget() const {
  using namespace AutoConstants;
  return AtTransTarget(TRANS_POS_ERR_TOLERANCE, TRANS_VEL_ERR_TOLERANCE)
    && AtRotTarget(ANG_POS_ERR_TOLERANCE, ANG_VEL_ERR_TOLERANCE);
}

/**
 * Returns whether robot is at the target translationally
 * 
 * @param posErrTol position error tolerance
 * @param velErrTol velocity error tolerance
 * 
 * @returns Whether robot is where it is supposed to be 
*/
bool AutoPath::AtTransTarget(double posErrTol, double velErrTol) const {
  vec::Vector2D targetPos = m_calcTrans.getPos(m_calcTrans.getHighestTime());

  return Utils::NearZero(targetPos - m_curPos, posErrTol) && Utils::NearZero(m_curVel, velErrTol);
}

/**
 * Returns whether robot is at the target rotationally
 * 
 * @param posErrTol position error tolerance
 * @param velErrTol velocity error tolerance
 * 
 * @returns Whether robot is where it is supposed to be 
*/
bool AutoPath::AtRotTarget(double posErrTol, double velErrTol) const {
  double targetAng = m_calcAng.getPos(m_calcAng.getHighestTime())[0];
  return Utils::NearZero(targetAng - GetMultipliedAng(), posErrTol) && Utils::NearZero(m_curAngVel, velErrTol);
}

/**
 * Gets angle with multiplier (angle represented as something greater than 180 or smaller than -180)
 * 
 * @returns Multiplied angle
*/
double AutoPath::GetMultipliedAng() const {
  return m_curAng + m_multiplier * M_PI * 2;
}

/**
 * Calculates position PID for trnaslational motion
 * 
 * @param deltaT time difference
 * @param curExpectedPos current expected translational motion
 * 
 * @returns Velocity from PID
*/
vec::Vector2D AutoPath::GetPIDTrans(double deltaT, vec::Vector2D curExpectedPos, vec::Vector2D curExpectedVel) {
  vec::Vector2D err = curExpectedPos - m_curPos;

  // frc::SmartDashboard::PutNumber("Error x", err.x());
  // frc::SmartDashboard::PutNumber("Error y", err.y());

  vec::Vector2D deltaErr =  curExpectedVel - m_curWheelVel;
  m_totalPosErr += err * deltaT;
  vec::Vector2D res = err * m_kPPos + m_totalPosErr * m_kIPos + deltaErr * m_kDPos;

  // frc::SmartDashboard::PutNumber("Delta Error x", deltaErr.x());
  // frc::SmartDashboard::PutNumber("Delta Error y", deltaErr.y());

  m_prevPosErr = err;

  return res;
}

/**
 * Calculates position PID for trnaslational motion
 * 
 * @param deltaT time difference
 * @param curExpectedPos current expected translational motion
 * 
 * @returns Velocity from PID
*/
double AutoPath::GetPIDAng(double deltaT, double curExpectedAng) {
  double curAngAbs = m_curAng + m_multiplier * M_PI * 2;
  double err = curExpectedAng - curAngAbs;

  // frc::SmartDashboard::PutNumber("ang expect", curExpectedAng);
  // frc::SmartDashboard::PutNumber("ang now", curAngAbs);
  // frc::SmartDashboard::PutNumber("ang err", err);
  // frc::SmartDashboard::PutNumber("ang abs", curAngAbs);

  double deltaErr = (err - m_prevAngErr) / deltaT;
  m_totalAngErr += err * deltaT;
  double res = err * m_kPAng + m_totalAngErr * m_kIPos + deltaErr * m_kDPos;

  m_prevAngErr = err;

  return res;
}

/**
 * Gets current translational velocity
 * 
 * @returns Translational velocity
*/
vec::Vector2D AutoPath::GetVel() const {
  return m_curVel;
}

/**
 * Gets current rotational velocity
 * 
 * @returns Rotational velocity
*/
double AutoPath::GetAngVel() const {
  // TODO add angle calculations
  return m_curAngVel;
}

/**
 * Gets current execute state
 * 
 * @returns Current execute state
*/
AutoPath::ExecuteState AutoPath::GetExecuteState() const {
  return m_curState;
}

/**
 * Gets greatest time of setpoints
 * 
 * @returns greatest time
*/
double AutoPath::GreatestTime() const {
  return m_calcTrans.getHighestTime();
}

/**
 * Gets multiplier
 * 
 * @returns Multiplier
*/
double AutoPath::GetMultiplier() const {
  return m_multiplier;
}