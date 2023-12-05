#pragma once

#include <cstddef>
#include <memory>

#include <AHRS.h>

#include "CompFilter.h"
// #include "KalmanFilter.h"
#include "SwerveControl.h"
#include "Util/thirdparty/simplevectors.hpp"

namespace vec = svector;

/**
 * Gets information about robot's velocity, position, and heading
 * 
 * Uses an absolute coordinate system on field, units are in meters.
 * Coordinate system: https://firstfrc.blob.core.windows.net/frc2023/FieldAssets/2023LayoutMarkingDiagram.pdf
 * Apriltag origin = coordinate origin
 * +x is towards red driver stations, +y is towrads human player stations, +z is up
 * 0 degree angle is facing +x direction, 90 degree is facing +y
*/
class Odometry {
public:
  Odometry(vec::Vector2D *posOffset, double *angOffset);
  
  // void SetKFTerms(double E0, double Q, double kAng, double k, double kPosInt, double maxTime);
  void SetAlpha(double alpha);
  void SetMaxTime(double maxTime);
  bool SetCamData(vec::Vector2D camPos, double camAng, std::size_t tagID, std::size_t age, std::size_t uniqueId);
  void SetTrimOffset(vec::Vector2D trimOffset);
  void AddTrimOffset(vec::Vector2D trimOffset);
  void Reset();

  vec::Vector2D GetTrimOffset() const;
  vec::Vector2D GetPosition() const;
  double GetAng() const;

  void Periodic(double ang, vec::Vector2D avgVelocity);

private:
  vec::Vector2D *m_posOffset;
  double *m_angOffset;

  vec::Vector2D m_trimOffset;
  // KalmanFilter m_filter;
  CompFilter m_filter;

  double m_ang;

  long long m_prevId;
};