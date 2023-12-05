#pragma once

#include <memory>

#include "Drive/Odometry.h"
#include "Drive/SwerveControl.h"
#include "Util/thirdparty/simplevectors.hpp"

namespace vec = svector;

/**
 * Class for autonomously going from one point to another using motion profiling
*/
class AutoLineup {
public:
  struct FFConfig {
    double maxSpeed;
    double maxAccel;
  };

  struct Times {
    double startT;
    double maxSpeedT;
    double descentT;
    double endT;
  };

  enum ExecuteState {
    NOT_EXECUTING,
    EXECUTING_TARGET,
    AT_TARGET
  };

  AutoLineup();

  void SetPosTarget(vec::Vector2D pos, bool rel);
  void SetAngTarget(double ang, bool rel);
  void SetTarget(vec::Vector2D pos, double ang, bool rel);
  void SetPosFF(FFConfig ffPos);
  void SetAngFF(FFConfig ffAng);
  void StartPosMove();
  void StartAngMove();
  void StopPos();
  void StopAng();
  void UpdateOdom(vec::Vector2D pos, double ang, vec::Vector2D curWheelVel);
  void Periodic();
  void SetPosPID(double kP, double kI, double kD);
  void SetAngPID(double kP, double kI, double kD);

  vec::Vector2D GetVel() const;
  double GetAngVel() const;
  vec::Vector2D GetTargetPos() const;
  double GetTargetAng() const;
  ExecuteState GetPosExecuteState() const;
  ExecuteState GetAngExecuteState() const;

private:
  vec::Vector2D m_curPos;
  vec::Vector2D m_curWheelVel;
  double m_curAng;

  double m_prevTime;

  vec::Vector2D m_curExpectedPos;
  double m_curExpectedAng;
  vec::Vector2D m_targetPos;
  double m_targetAng;

  vec::Vector2D m_curVel;
  double m_curAngVel;

  Times m_posTimes;
  Times m_angTimes;
  vec::Vector2D m_posVecDir;
  double m_angVecDir;
  FFConfig m_ffPos;
  FFConfig m_ffAng;

  ExecuteState m_posState;
  ExecuteState m_angState;

  vec::Vector2D m_prevPos;
  double m_prevAng;

  // for position PID
  vec::Vector2D m_prevPosErr;
  double m_prevAngErr;
  vec::Vector2D m_totalPosErr;
  double m_totalAngErr;

  // for velocity PID
  vec::Vector2D m_prevVelErr;
  // double m_prevAngVelErr;
  vec::Vector2D m_totalVelErr;
  // double m_totalAngVelErr;

  double m_kPPos;
  double m_kIPos;
  double m_kDPos;

  double m_kPAng;
  double m_kIAng;
  double m_kDAng;

  void CalcTimes(FFConfig &config, double dist, Times &times);
  double GetSpeed(FFConfig &config, Times &times);

  vec::Vector2D GetPIDTrans(double deltaT, vec::Vector2D curExpectedVel);
  double GetPIDAng(double deltaT);

  vec::Vector2D GetPIDTransVel(double deltaT, vec::Vector2D expectedVel);
  // double GetPIDAngVel(double deltaT, vec::Vector2D expectedAngVel);

  bool AtPosTarget() const;
  bool AtAngTarget() const;
  bool AtPosTarget(double posErrTol, double velErrTol) const;
  bool AtAngTarget(double posErrTol, double velErrTol) const;

  // TEMP
  // double m_dist;
};