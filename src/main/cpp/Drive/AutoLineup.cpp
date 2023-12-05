#include "Drive/AutoLineup.h"

#include <cmath>
#include <frc/smartdashboard/SmartDashboard.h>

#ifndef M_PI
#define M_PI 3.1415926535897932384626433
#endif

#include "Drive/DriveConstants.h"
#include "Util/Utils.h"

/**
 * Constructor
 * 
 * @param odometry Pointer to odometry object
*/
AutoLineup::AutoLineup()
  : m_curAng{0}, m_prevTime{0}, m_curExpectedAng{0}, m_targetAng{0}, m_curAngVel{0}, m_posTimes{0, 0, 0, 0},
  m_angTimes{0, 0, 0, 0}, m_angVecDir{0}, m_ffPos{0, 0}, m_ffAng{0, 0}, m_posState{NOT_EXECUTING},
  m_prevPos{0}, m_prevAng{0}, m_prevPosErr{0}, m_prevAngErr{0}, m_totalPosErr{0}, m_totalAngErr{0},
  /*m_prevAngVelErr{0}, m_totalAngVelErr{0},*/ m_kPPos{0}, m_kIPos{0}, m_kDPos{0}, m_kPAng{0}, m_kIAng{0}, m_kDAng{0}
{
  SetPosPID(LineupConstants::TRANS_KP, LineupConstants::TRANS_KI, LineupConstants::TRANS_KD);
  SetAngPID(LineupConstants::ANG_KP, LineupConstants::ANG_KI, LineupConstants::ANG_KD);
  SetPosFF({LineupConstants::TRANS_MAXSP, LineupConstants::TRANS_MAXACC});
  SetAngFF({LineupConstants::ANG_MAXSP, LineupConstants::ANG_MAXACC});
}


/**
 * Sets translational position target for robot
 * 
 * @param pos Absolute position, if rel is true, otherwise delta position
 * @param rel Whether pos parameter should be interpreted as absolute on field or relative to current pos
*/
void AutoLineup::SetPosTarget(vec::Vector2D pos, bool rel) {
  if (m_posState == EXECUTING_TARGET) {
    return;
  }

  if (rel) {
    m_targetPos = m_curPos + pos;
  } else {
    m_targetPos = pos;
  }
}

/**
 * Sets angular position target for robot
 * 
 * @param ang Absolute angle, if rel is true, otherwise delta position
 * @param rel Whether pos parameter should be interpreted as absolute on field or relative to current pos
 * 
 * @note ang parameter will be normalized to -180 to 180 if outside range
*/
void AutoLineup::SetAngTarget(double ang, bool rel) {
  if (m_posState == EXECUTING_TARGET) {
    return;
  }

  if (rel) {
    m_targetAng = Utils::NormalizeAng(m_curAng + ang);
  } else {
    m_targetAng = Utils::NormalizeAng(ang);
  }
}

/**
 * Sets both position and angular position target for robot
 * 
 * @param pos Absolute position, if rel is true, otherwise delta position
 * @param ang Absolute angle, if rel is true, otherwise delta position
 * @param rel Whether pos parameter should be interpreted as absolute on field or relative to current pos
 * 
 * @note ang parameter will be normalized to -180 to 180 if outside range
*/
void AutoLineup::SetTarget(vec::Vector2D pos, double ang, bool rel) {
  if (m_posState == EXECUTING_TARGET) {
    return;
  }

  SetPosTarget(pos, rel);
  SetAngTarget(ang, rel);
}

/**
 * Sets feed forward parameters for translational motion
 * 
 * Does not set if currently executing a path
 * 
 * @param ffPos Feed forward parameters
*/
void AutoLineup::SetPosFF(FFConfig ffPos) {
  if (m_posState != EXECUTING_TARGET) {
    m_ffPos.maxAccel = ffPos.maxAccel;
    m_ffPos.maxSpeed = ffPos.maxSpeed;
  }
}

/**
 * Sets feed forward parameters for angular motion
 * 
 * Does not set if currently executing a path
 * 
 * @param ffAng Feed fowrad parameters
*/
void AutoLineup::SetAngFF(FFConfig ffAng) {
  if (m_angState != EXECUTING_TARGET) {
    m_ffAng.maxAccel = ffAng.maxAccel;
    m_ffAng.maxSpeed = ffAng.maxSpeed;
  }
}

/**
 * Starts executing the move both in translational motion
*/
void AutoLineup::StartPosMove() {
  if (m_posState == EXECUTING_TARGET) {
    return;
  }

  vec::Vector2D posDiff = m_targetPos - m_curPos;

  CalcTimes(m_ffPos, magn(posDiff), m_posTimes);  

  if (Utils::NearZero(posDiff)) {
    m_posVecDir = {1, 0};
  } else {
    m_posVecDir = normalize(posDiff);
  }

  m_curExpectedPos = m_curPos;
  m_prevPosErr = {0, 0};
  m_totalPosErr = {0, 0};
  m_posState = EXECUTING_TARGET;
}

/**
 * Start execuitng move in angular motion
*/
void AutoLineup::StartAngMove() {
  if (m_angState == EXECUTING_TARGET) {
    return;
  }  
  // calculates angular distance
  double dist;
  if (std::abs(m_targetAng - m_curAng) < M_PI) {
    if (m_curAng < m_targetAng) {
      m_angVecDir = 1;
    } else {
      m_angVecDir = -1;
    }
    dist = std::abs(m_targetAng - m_curAng);
  } else {
    if (m_curAng < m_targetAng) {
      m_angVecDir = -1;
    } else {
      m_angVecDir = 1;
    }
    dist = 2 * M_PI - std::abs(m_targetAng - m_curAng);
  }

  CalcTimes(m_ffAng, dist, m_angTimes);
  
  m_curExpectedAng = m_curAng;
  m_prevAngErr = 0;
  m_totalAngErr = 0;
  m_angState = EXECUTING_TARGET;
}

/**
 * Starts executing the move in one of translational/rotational
 * 
 * @param config The ff config parameters
 * @param dist The distance to move
 * @param times The tiemes parameters
*/
void AutoLineup::CalcTimes(FFConfig &config, double dist, Times &times) {
  double curT = Utils::GetCurTimeS();

  if (Utils::NearZero(config.maxAccel) || Utils::NearZero(config.maxSpeed)) {
    return;
  }

  // time to accelerate/decelerate
  double increaseT = config.maxSpeed / config.maxAccel;
  // time to maintain max speed
  double maintainT = 0;
  if (increaseT * config.maxSpeed > dist) {
    // check math, may be wrong
    increaseT = std::sqrt(dist / config.maxAccel);
    maintainT = 0;
  } else {
    maintainT = (dist - increaseT * config.maxSpeed) / config.maxSpeed;
  }

  times.startT = curT;
  times.maxSpeedT = curT + increaseT;
  times.descentT = curT + increaseT + maintainT;
  times.endT = curT + increaseT * 2 + maintainT;

  // std::cout << "max speed" << config.maxSpeed << std::endl;
  // std::cout << "max ac" << config.maxSpeed << std::endl;
  // std::cout << "startT: " << times.startT << std::endl;
  // std::cout << "maxSpeedT: " << times.maxSpeedT << std::endl;
  // std::cout << "descentT: " << times.descentT << std::endl;
  // std::cout << "endT: " << times.endT << std::endl;
}

/**
 * Stops executing position command
*/
void AutoLineup::StopPos() {
  // std::cout << "stoppos" << std::endl;
  m_posState = NOT_EXECUTING;
}

/**
 * Stops executing angle command
*/
void AutoLineup::StopAng() {
  m_angState = NOT_EXECUTING;
}

/**
 * Gets current translational execute state
 * 
 * @returns Current execute state
*/
AutoLineup::ExecuteState AutoLineup::GetPosExecuteState() const {
  return m_posState;
}

/**
 * Get current ang execute state
 * 
 * @returns Current ang execute state
*/
AutoLineup::ExecuteState AutoLineup::GetAngExecuteState() const {
  return m_angState;
}

/**
 * Returns whether robot translational position is at the target
 * 
 * Assumes autoconstant error tolerance
 * 
 * @returns Wheter robot is where it is supposed to be positionally at the end of profile
*/
bool AutoLineup::AtPosTarget() const {
  return AtPosTarget(LineupConstants::TRANS_POS_ERR_TOLERANCE, LineupConstants::TRANS_VEL_ERR_TOLERANCE);
}

/**
 * Returns wheter robot translational position is at target
 * 
 * @param posErrTol position error tolerance
 * @param velErrTol velocity error tolerance
 * 
 * @returns Whether robot is where it is supposed to be positionally at the end of profile
*/
bool AutoLineup::AtPosTarget(double posErrTol, double velErrTol) const {
  return Utils::NearZero(m_targetPos - m_curPos, posErrTol) && Utils::NearZero(m_curVel, velErrTol);
}

/**
 * Returns whether robot angular position is at the target
 * 
 * Assumes autoconstant error tolerance
 * 
 * @returns Whether robot is where it is supposed to be angularly at the end of profile
*/
bool AutoLineup::AtAngTarget() const {
  return AtAngTarget(LineupConstants::ANG_POS_ERR_TOLERANCE, LineupConstants::ANG_VEL_ERR_TOLERANCE);
}

/**
 * Returns wheter robot angular position is at target
 * 
 * @param posErrTol position error tolerance
 * @param velErrTol velocity error tolerance
 * 
 * @returns Wheter robot is where it is supposed to be angularly at the end of profile
*/
bool AutoLineup::AtAngTarget(double posErrTol, double velErrTol) const {
  return Utils::NearZero(m_targetAng - m_curAng, posErrTol) && Utils::NearZero(m_curAngVel, velErrTol);
}

/**
 * Periodic function
*/
void AutoLineup::Periodic() { 
  double curTime = Utils::GetCurTimeS();
  double deltaT = curTime - m_prevTime;

  switch (m_posState) {
    case NOT_EXECUTING:
      m_curVel = {0, 0};
      break;
    case EXECUTING_TARGET:
    {
      double speed = GetSpeed(m_ffPos, m_posTimes);
      vec::Vector2D ffVel = m_posVecDir * speed;
      m_curExpectedPos += ffVel * deltaT;

      vec::Vector2D correctionVel = GetPIDTrans(deltaT, ffVel);
      // vec::Vector2D correctionVel = GetPIDTransVel(deltaT, ffVel);
      vec::Vector2D totalVel = ffVel + correctionVel;

      m_curVel = totalVel;

      if (AtPosTarget()) {
        m_posState = AT_TARGET;
      }

      break;
    }
    case AT_TARGET:
    {
      m_curVel = {0, 0};
      if (!AtPosTarget()) {
        m_posState = NOT_EXECUTING;
      }

      break;
    }
  }

  // frc::SmartDashboard::PutNumber("m state", m_angState);
  switch (m_angState) {
    case NOT_EXECUTING:
      m_curAngVel = 0;
      // m_dist = 0;
      break;
    case EXECUTING_TARGET:
    {
      double angSpeed = GetSpeed(m_ffAng, m_angTimes);
      double ffAngVel = m_angVecDir * angSpeed;

      m_curExpectedAng += ffAngVel * deltaT;
      m_curExpectedAng = Utils::NormalizeAng(m_curExpectedAng);

      double correctionVel = GetPIDAng(deltaT);
      double totalVel = ffAngVel + correctionVel;

      // frc::SmartDashboard::PutNumber("taarget ang", m_targetAng);
      // frc::SmartDashboard::PutNumber("cur speed", angSpeed);
      // frc::SmartDashboard::PutNumber("ang dir", m_angVecDir);
      // frc::SmartDashboard::PutNumber("cur dist", m_dist);
      // m_dist += angSpeed * 0.02;

      m_curAngVel = totalVel;

      if (AtAngTarget()) {
        m_angState = AT_TARGET;
      }
      break;
    }
    case AT_TARGET:
    {
      m_curAngVel = 0;
      if (!AtAngTarget()) {
        m_angState = NOT_EXECUTING;
      }

      break;
    }
  }

  m_prevTime = curTime;
}

/**
 * Gets current speed in one of rotational/translational motion
 * 
 * @param config One of rotational/translational ff paraameters
 * @param times The times parameters for rotational/translational
 * 
 * @returns Speed
*/
double AutoLineup::GetSpeed(FFConfig &config, Times &times) {
  double curT = Utils::GetCurTimeS();
  if (curT < times.startT) {
    // shouldn't be here
    return 0;
  }
  if (curT > times.endT) {
    return 0;
  }

  double speed;
  if (curT < times.maxSpeedT) {
    speed = config.maxAccel * (curT - times.startT);
  } else if (curT < times.descentT) {
    speed = config.maxSpeed;
  } else {
    speed = config.maxAccel * (times.endT - curT);
  }

  return speed;
}

/**
 * Gets current translational velocity
 * 
 * @returns Translational velocity
*/
vec::Vector2D AutoLineup::GetVel() const {
  return m_curVel;
}

/**
 * Gets current rotational velocity
 * 
 * @returns Rotational velocity
*/
double AutoLineup::GetAngVel() const {
  return m_curAngVel;
}

/**
 * Calculates position PID for trnaslational motion
 * 
 * @param deltaT time difference
 * 
 * @returns Velocity from PID
*/
vec::Vector2D AutoLineup::GetPIDTrans(double deltaT, vec::Vector2D curExpectedVel) {
  vec::Vector2D err = m_curExpectedPos - m_curPos;

  vec::Vector2D deltaErr = curExpectedVel - m_curVel;
  m_totalPosErr += err * deltaT;
  vec::Vector2D res = err * m_kPPos + m_totalPosErr * m_kIPos + deltaErr * m_kDPos;

  m_prevPosErr = err;

  return res;
}

/**
 * Calculates velocity PID for translational motion
 * 
 * This is currently unused
 * 
 * @param deltaT time difference
 * @param expectedVel Expected velocity at this time
 * 
 * @returns Velocity correction from PID
*/
vec::Vector2D AutoLineup::GetPIDTransVel(double deltaT, vec::Vector2D expectedVel) {
  vec::Vector2D curVel = (m_curPos - m_prevPos) / deltaT;

  vec::Vector2D err = expectedVel - curVel;

  vec::Vector2D deltaErr = (err - m_prevVelErr) / deltaT;
  m_totalVelErr += err * deltaT;
  vec::Vector2D res = err * m_kPPos + m_totalVelErr * m_kIPos + deltaErr * m_kDPos;

  m_prevPos = m_curPos;
  m_prevVelErr = err;

  return res;
}

/**
 * Calculates positional PID for angular motion
 * 
 * @param deltaT time diffrence
 * 
 * @returns Velcoity from PID
*/
double AutoLineup::GetPIDAng(double deltaT) {
  double err;
  double dir;
  if (std::abs(m_curExpectedAng - m_curAng) < M_PI) {
    if (m_curAng < m_curExpectedAng) {
      dir = 1;
    } else {
      dir = -1;
    }
    err = std::abs(m_curExpectedAng - m_curAng);
  } else {
    if (m_curAng < m_curExpectedAng) {
      dir = -1;
    } else {
      dir = 1;
    }
    err = 2 * M_PI - std::abs(m_curExpectedAng - m_curAng);
  }

  err = dir * err;

  double deltaErr = (err - m_prevAngErr) / deltaT;
  m_totalAngErr += err * deltaT;
  double res = err * m_kPAng + m_totalAngErr * m_kIAng + deltaErr * m_kDAng;

  m_prevAngErr = err;

  return res;
}

/**
 * Sets position correction PID
 * 
 * @param kP p
 * @param kI i
 * @param kD d
*/
void AutoLineup::SetPosPID(double kP, double kI, double kD) {
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
void AutoLineup::SetAngPID(double kP, double kI, double kD) {
  m_kPAng = kP;
  m_kIAng = kI;
  m_kDAng = kD;
}

/**
 * Updates odometry
 * 
 * @note Make sure this is called every cycle
 * 
 * @param pos Current position
 * @param ang Current angle
*/
void AutoLineup::UpdateOdom(vec::Vector2D pos, double ang, vec::Vector2D curWheelVel) {
  m_curPos = pos;
  m_curAng = ang;
  m_curWheelVel = curWheelVel;
}

/**
 * Gets target position
 * 
 * @returns target position
*/
vec::Vector2D AutoLineup::GetTargetPos() const {
  return m_targetPos;
}

/**
 * Gets target angle
 * 
 * @returns Target angle
*/
double AutoLineup::GetTargetAng() const {
  return m_targetAng;
}