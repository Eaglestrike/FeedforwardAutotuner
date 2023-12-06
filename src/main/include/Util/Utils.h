#pragma once

#include <cstddef>
#include <vector>

#include "Drive/DriveConstants.h"
#include "Elevator/BaseElevator/ElevatorConstants.h"
#include "GeneralConstants.h"
#include "thirdparty/simplevectors.hpp"
#include "Util/UtilConstants.h"

namespace vec = svector; //!< vector namespace alias

/**
 * Utility class with static methods
 * 
 * Math + time included
*/
namespace Utils {
  double AbsMin(const double a, const double b);

  vec::Vector2D GetVecAverage(const std::vector<vec::Vector2D>);

  bool NearZero(const double num, const double tolerance = UtilConstants::NEAR_ZERO_TOLERANCE);
  bool NearZero(const vec::Vector2D vec, const double tolerance = UtilConstants::NEAR_ZERO_TOLERANCE);

  double NormalizeAng(const double ang);

  std::size_t GetCurTimeMs();
  double GetCurTimeS();

  double DegToRad(const double deg);
  double RadToDeg(const double rad);

  vec::Vector2D GetUnitVecDir(const double ang);
  vec::Vector2D GetProjection(const vec::Vector2D v, const vec::Vector2D w);
  double GetAngBetweenVec(const vec::Vector2D v1, const vec::Vector2D v2);

  AutoPaths::SwervePose GetRedPose(AutoPaths::SwervePose bluePose);
  std::vector<AutoPaths::SwervePose> GetRedPoses(std::vector<AutoPaths::SwervePose> bluePoses);
  FieldConstants::ScorePair GetScoringPos(int pos, int height, bool red);
  int getPieceHeight(ElevatorConstants::ElevatorTarget target);

  bool IsCone(int pos);

  int GetExpectedTagId(int pos, bool red);

  double sign(double x);
};