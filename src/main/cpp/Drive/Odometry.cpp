#include "Drive/Odometry.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>
#include <iostream>

#include "Drive/DriveConstants.h"
#include "GeneralConstants.h"
#include "Util/Utils.h"

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399
#endif

/**
 * Constructor
 * 
 * Initializes starting position and angle to 0, and this can be changed later with SetStart() 
 * 
 * @param posOffset pointer to position offset in robot cpp
 * @param angOffset pointer to angle offset in robot cpp
 */
Odometry::Odometry(vec::Vector2D *posOffset, double *angOffset)
  : m_posOffset{posOffset}, m_angOffset{angOffset}, m_trimOffset{0, 0}, 
    // m_filter{OdometryConstants::E0, OdometryConstants::Q, OdometryConstants::CAM_TRUST_KANG, OdometryConstants::CAM_TRUST_KPOS, OdometryConstants::CAM_TRUST_KPOSINT, OdometryConstants::MAX_TIME, posOffset, angOffset},
    m_filter{OdometryConstants::ALPHA, OdometryConstants::MAX_TIME, m_posOffset},
    m_ang{0},
    m_prevId{-1} {}

/**
 * Sets Kalman filter terms
 * 
 * Use this for debugging
 * 
 * It is recommended to reset odometry after setting terms
 * 
 * All of these must be > 0 or things will break
 * 
 * @param E0 initial error covariance
 * @param Q noise of wheels
 * @param kAng angle constant in logistic function higher = lower trust in camera for higher velocities
 * @param k constant of proportionality between speed and camera noise
 * @param kPosInt constant of camera noise when robot not moving
 * @param maxTime max time before ignore, in s
*/
// void Odometry::SetKFTerms(double E0, double Q, double kAng, double k, double kPosInt, double maxTime) {
//   m_filter.SetTerms(E0, Q, kAng, k, kPosInt, maxTime);
// }

/**
 * Applies corrections from camera data
 * 
 * @param camPos Position data from camera
 * @param camAng Angle from camera (unused, may use later)
 * @param tagID Apriltag ID
 * @param age delay measurement from camera (combined delay from camera to jetson and from jetson to rio through network), ms
 * @param uniqueId unique ID from camera
 * 
 * @returns Wheter data is valid (new uniqueID)
*/
bool Odometry::SetCamData(vec::Vector2D camPos, double camAng, std::size_t tagID, std::size_t age, std::size_t uniqueId)
{
  double angNavX = GetAng();
  vec::Vector2D vecRot = rotate(camPos, angNavX - M_PI / 2);
  vec::Vector2D tagPos;

  // check that ID is actually unique
  if (static_cast<long long>(uniqueId) == m_prevId) {
    return false;
  }
  m_prevId = static_cast<long long>(uniqueId);

  if (tagID < 1 || tagID > 8) {
    return false;
  }
  tagPos = FieldConstants::TAGS[tagID - 1];
  vec::Vector2D robotPos = tagPos - vecRot;

  // reject if april tag pos is too far away
  vec::Vector2D odomPos = GetPosition();
  if (vec::magn(odomPos - robotPos) > OdometryConstants::AT_REJECT) {
    return false;
  }

  // reject if apriltag is not facing the robot (false dtection)
  double angRobot = GetAng();
  vec::Vector2D unitVec = Utils::GetUnitVecDir(angRobot);
  vec::Vector2D vecToTag = tagPos - odomPos;

  // // check that robot is not behind tag
  // if (tagID != 4 && tagID != 5 && (odomPos.x() < FieldConstants::TAG8.x() || odomPos.x() > FieldConstants::TAG1.x())) {
  //   // std::cout << "bad 1" << std::endl;
  //   return false;
  // }
  // if ((tagID == 4 || tagID == 5) && (odomPos.x() < FieldConstants::TAG5.x() || odomPos.x() > FieldConstants::TAG4.x())) {
  //   // std::cout << "bad 2" << std::endl;
  //   return false;
  // }

  double angToCam = Utils::GetAngBetweenVec(unitVec, vecToTag);
  // frc::SmartDashboard::PutString("tag vec", tagPos.toString());
  // frc::SmartDashboard::PutString("cam vec", vecToTag.toString());
  // frc::SmartDashboard::PutNumber("Angle to Cam", angToCam);
  // return false;
  if (std::abs(angToCam) >= M_PI / 2) {
    return false;
  }

  // not using camAng, because it relies on existing odometry measurements to get accurate and ideally it's its own, independent measurement
  // @todo figure out if ^^^ is right
  std::size_t curTimeMs = Utils::GetCurTimeMs();
  //         substituting angnavX here vvvvvv becaues of waht's mentioned in comment above
  // m_filter.UpdateFromCamera(robotPos, Utils::DegToRad(angNavX), age, curTimeMs);
  m_filter.AddCamPos(robotPos, age, curTimeMs);

  return true;
}

/**
 * Resets current position and angle
 * 
 * Resets trim offset as well
*/
void Odometry::Reset() {
  std::size_t curTimeMs = Utils::GetCurTimeMs();
  m_filter.Reset(curTimeMs);
  m_trimOffset = {0, 0};
}

/**
 * Gets current position world frame
 * 
 * @returns current predicted position
*/
vec::Vector2D Odometry::GetPosition() const {
  // return m_filter.GetEstimatedPos() + *m_posOffset;
  return m_filter.GetPos() + m_trimOffset;
}

/**
 * Gets current estimated angle world frame
 * 
 * @returns estimated angle, in radians
*/
double Odometry::GetAng() const {
  // const double ang = m_filter.GetEstimatedAng();
  return Utils::NormalizeAng(m_ang + *m_angOffset);
}

/**
 * Sets alpha term, a term between 0 and 1 that de4termines weight given to wheels
 * 
 * @param alpha Alpha term
*/
void Odometry::SetAlpha(double alpha) {
  m_filter.SetAlpha(alpha);
}

/**
 * Sets maximum time
 * 
 * @param maxTime maximum time before discarding values
*/
void Odometry::SetMaxTime(double maxTime) {
  m_filter.SetMaxTime(maxTime);
}

/**
 * Periodic function
 * 
 * @param ang navX angle of robot, radians
 * @param avgVelocity average velocity world frame
*/
void Odometry::Periodic(double ang, vec::Vector2D avgVelocity) {
  std::size_t curTimeMs = Utils::GetCurTimeMs();
  m_ang = ang;
  // m_filter.PredictFromWheels(avgVelocity, ang + *m_angOffset, curTimeMs);
  m_filter.AddWheelVel(avgVelocity, curTimeMs);
}

/**
 * Sets trim offset
 * 
 * @param trimOffset trim offset
*/
void Odometry::SetTrimOffset(vec::Vector2D trimOffset) {
  m_trimOffset = trimOffset;
}

/**
 * Adds trim offset to what is already there
 * 
 * @param trimOffset trim offset
*/
void Odometry::AddTrimOffset(vec::Vector2D trimOffset) {
  m_trimOffset += trimOffset;
}

/**
 * Gets trim offset
 * 
 * @returns trim offset
*/
vec::Vector2D Odometry::GetTrimOffset() const {
  return m_trimOffset;
}