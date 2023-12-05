#pragma once

#include <cstddef>
#include <map>
#include <vector>

#include "Drive/DriveConstants.h"

#include "Util/Utils.h"
#include "Util/thirdparty/hermite.hpp"
#include "Util/thirdparty/simplevectors.hpp"

namespace hm = hermite;
namespace vec = svector;

class AutoPath {
  typedef hm::Hermite<1> Hermite1;
  typedef hm::Pose<1> Pose1;
  typedef hm::Hermite<2> Hermite2;
  typedef hm::Pose<2> Pose2;

public:
  enum ExecuteState {
    NOT_EXECUTING,
    EXECUTING_PATH,
    AT_TARGET
  };

  AutoPath();

  void AddPose(AutoPaths::SwervePose pose);
  void AddPoses(std::vector<AutoPaths::SwervePose> poses);
  void ResetPath();
  void ResetMultiplier();
  void SetPosPID(double kP, double kI, double kD);
  void SetAngPID(double kP, double kI, double kD);
  void Stop();
  void StartMove();
  void UpdateOdom(vec::Vector2D curPos, double curAng, vec::Vector2D curWheelVel);
  void Periodic();

  vec::Vector2D GetVel() const;
  double GetAngVel() const;
  ExecuteState GetExecuteState() const;
  double GreatestTime() const;
  double GetMultiplier() const;

private:
  vec::Vector2D m_curPos;
  vec::Vector2D m_curWheelVel;
  double m_curAng;
  int m_multiplier; // markplier???

  vec::Vector2D m_curVel;
  double m_curAngVel;

  double m_startTime;
  double m_expectFinish;
  double m_prevTime;
  double m_prevTimeOdom;

  // Hermite3 m_calc;
  Hermite2 m_calcTrans;
  Hermite1 m_calcAng;

  ExecuteState m_curState;

  // for position PID
  vec::Vector2D m_prevPosErr;
  vec::Vector2D m_totalPosErr;

  double m_prevAngErr;
  double m_totalAngErr;

  double m_kPPos;
  double m_kIPos;
  double m_kDPos;

  double m_kPAng;
  double m_kIAng;
  double m_kDAng;

  vec::Vector2D GetPIDTrans(double deltaT, vec::Vector2D curExpectedPos, vec::Vector2D curExpectedVel); 
  double GetPIDAng(double deltaT, double curExpectedAng);
  bool AtTarget() const;
  bool AtTransTarget(double posErrTol, double velErrTol) const;
  bool AtRotTarget(double posErrTol, double velErrTol) const;
  double GetMultipliedAng() const;
};