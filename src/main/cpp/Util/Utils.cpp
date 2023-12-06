#include "Util/Utils.h"

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399
#endif

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "GeneralConstants.h"

/**
 * Gets the value with the minimum absolute value between two numbers
 *
 * @param a One number
 * @param b Another number
 *
 * @returns a or b, depending on which one has a lesser absolute value
 */
double Utils::AbsMin(const double a, const double b)
{
  return std::abs(a) < std::abs(b) ? a : b;
}

/**
 * Averages a std::vector of vec::Vector2D
 *
 * @param vectors the non-zero sized array of vectors
 *
 * @note If the size of vectors is 0, then returns a zero vector.
 *
 * @returns The averaged vector
 */
vec::Vector2D Utils::GetVecAverage(const std::vector<vec::Vector2D> vectors)
{
  vec::Vector2D res;
  if (vectors.size() == 0)
  {
    return res;
  }

  for (auto vec : vectors)
  {
    res += vec;
  }

  res /= vectors.size();

  return res;
}

/**
 * Determines if a number is near zero
 *
 * @param num Number
 * @param tolerance Tolerance for being near zero
 */
bool Utils::NearZero(const double num, const double tolerance)
{
  return std::abs(num) <= tolerance;
}

/**
 * Determines if a vector is near zero
 *
 * @param vec Vector
 * @param tolerance Tolerance for being near zero
 */
bool Utils::NearZero(const vec::Vector2D vec, const double tolerance)
{
  for (auto component : vec)
  {
    if (!NearZero(component, tolerance))
    {
      return false;
    }
  }
  return true;
}


/**
 * Normalizes angle to (180, -180]
 * 
 * @returns normalized angle
*/
double Utils::NormalizeAng(const double ang) {
  double ang2 = std::fmod(ang, M_PI * 2);
  ang2 = std::fmod(ang + M_PI * 2, M_PI * 2);
  if (ang2 > M_PI) {
    ang2 -= M_PI * 2;
  }

  return ang2;
}

/**
 * Gets the current time from robot start in ms
 * 
 * @returns Current robot time in ms
*/
std::size_t Utils::GetCurTimeMs() {
  auto curTime = frc::Timer::GetFPGATimestamp();
  auto timeMs = curTime.convert<units::millisecond>();
  return static_cast<std::size_t>(timeMs.value());
}

/**
 * Gets current time from robot start in s
 * 
 * @returns Current robot time in s
*/
double Utils::GetCurTimeS() {
  auto curTime = frc::Timer::GetFPGATimestamp();
  auto timeS = curTime.convert<units::second>(); 
  return timeS.value();
}

/**
 * Converst degrees to radians
 * 
 * @param deg Angle in degrees
 * 
 * @returns Angle in radians
*/
double Utils::DegToRad(const double deg) {
  return deg * (M_PI / 180.0);
}

/**
 * Converts radians to degrees
 * 
 * @param rad Angle in radians
 * 
 * @returns Angle in degrees
*/
double Utils::RadToDeg(const double rad) {
  return rad * (180.0 / M_PI);
}

/**
 * Gets unit vector from a direction angle
 * 
 * @param ang Direction angle
 * 
 * @returns Unit vector with a direction in the indicated angle
*/
vec::Vector2D Utils::GetUnitVecDir(const double ang) {
  return vec::Vector2D{std::cos(ang), std::sin(ang)};
}

/**
 * Gets component of v in the direction of w
 * 
 * @param v Vector to project
 * @param w Reference vector, must be nonzero
 * 
 * @returns Component of v in the direction of w
*/
vec::Vector2D Utils::GetProjection(const vec::Vector2D v, const vec::Vector2D w) {
  return w * (dot(v, w) / (magn(w) * magn(w)));
}

/**
 * Gets angle between two vectors
 * 
 * @param v1 First vector (must be nonzero)
 * @param v2 Second vector (must be nonzero)
 * 
 * @returns Angle between two vectors
*/
double Utils::GetAngBetweenVec(const vec::Vector2D v1, const vec::Vector2D v2) {
 return std::acos(std::clamp(
      dot(v1, v2) / (magn(v1) * magn(v2)), -1.0, 1.0));
}

/**
 * Gets red side equivalent of a blue pose
 * 
 * @param bluePose a blue pose
 * 
 * @returns Red equivalent of the blue pose
*/
AutoPaths::SwervePose Utils::GetRedPose(AutoPaths::SwervePose bluePose) {
  AutoPaths::SwervePose redPose;

  redPose.time = bluePose.time;
  redPose.y = bluePose.y;
  redPose.vy = bluePose.vy;
  redPose.x = FieldConstants::FIELD_WIDTH - bluePose.x;
  redPose.vx = -bluePose.vx;
  redPose.ang = M_PI - bluePose.ang;
  redPose.angVel = -bluePose.angVel;

  return redPose;
}

/**
 * Gets vector of red side equivalents of a blue pose
 * 
 * @param bluePoses an vector of blue poses
 * 
 * @returns Vector of red equivalents of the blue pose
*/
std::vector<AutoPaths::SwervePose> Utils::GetRedPoses(std::vector<AutoPaths::SwervePose> bluePoses) {
  std::vector<AutoPaths::SwervePose> res;
  for (auto bluePose : bluePoses) {
    res.push_back(GetRedPose(bluePose));
  }

  return res;
}

/**
 * Gets vector pos of scoring position
 * 
 * @param pos Scoring position, number from 1-9 not 0-8
 * @param height Low mid high, number from 1-3 not 0-2
 * @param red If on red side
 * 
 * @returns vector pos of scoring position
*/
FieldConstants::ScorePair Utils::GetScoringPos(int pos, int height, bool red) {
  if (pos < 1 || pos > 9) {
    return {{0, 0}, 0};
  }

  if (height < 1 || height > 3) {
    return {{0, 0}, 0};
  }

  pos--;
  height--;

  int idx = red ? 8 - pos : pos;
  int mult = idx / 3;
  idx = idx % 3;

  FieldConstants::ScorePair scoreP = FieldConstants::BLUE_SCORING_POS[idx][height];
  double newX = red ? FieldConstants::FIELD_WIDTH - x(scoreP.first) : x(scoreP.first);
  double newY = y(scoreP.first) - mult * FieldConstants::DIST_BETWEEN_TAGS;
  scoreP.first = {newX, newY};
  // scoreP.second = red ? LIDAR_MAX_DIST - scoreP.second : scoreP.second;

  return scoreP;
}

int Utils::getPieceHeight(ElevatorConstants::ElevatorTarget target){
  using enum ElevatorConstants::ElevatorTarget;
  switch(target){
    case CUSTOM:
      return 0;
    case LOW:
      return 1;
    case MID:
      return 2;
    case HIGH:
      return 3;
    case STOWED:
      return 0;
  }
}

/**
 * Determines if currently setting a cone from scoring position
*/
bool Utils::IsCone(int pos) {
  if (pos == 0) return false;

  pos--;

  return pos % 3 != 1;
}

/**
 * Gets expected tag ID from button board position
 * 
 * @param pos Position
 * @param red Is red
 * 
 * @returns Tag ID
*/
int Utils::GetExpectedTagId(int pos, bool red) {
  if (pos == 0) {
    return 0;
  }

  pos--;

  int id = pos / 3 + 1;
  if (!red) id += 5;
  return id;
}

/**
 * Sign of number
*/

double Utils::sign(double x){
  if(x > 0.0){
    return 1.0;
  }
  else if(x < 0.0){
    return -1.0;
  }
  else{
    return 0.0;
  }
}