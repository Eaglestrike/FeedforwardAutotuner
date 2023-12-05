#include "Drive/KalmanFilter.h"

#include <cmath>
#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

typedef vec::Vector2D Vector;
/**
 * Constrcutor
 * 
 * @param E0 Initial error value
 * @param Q wheel odometry measurement noise
 * @param kAng angle constant of proportionality for logistic function calculating trust of camera's angle measurement
 * @param k constant of proportionality for linear function calculating noise of camera xy measurement
 * @param kPosInt camera constant noise when robot is not moving
 * @param maxTime maximum time before discarding measurements, in s
 * @param posOffset pointer to Position offset in robot cpp
 * @param angOffset pointer to angle offset in robot cpp
*/
KalmanFilter::KalmanFilter(double E0, double Q, double kAng, double k, double kPosInt, double maxTime, vec::Vector2D *posOffset, double *angOffset)
  : m_E0{E0}, m_Q{Q}, m_kAng{kAng}, m_kPos{k}, m_kPosInt{kPosInt}, m_maxTime{maxTime}, m_posOffset{posOffset}, m_angOffset{angOffset}
{
  // we want to make sure there's at least one value in the map at all times
  m_states[0].E = E0;
  m_states[0].ang = 0;

  if(m_useSmartDashboard){
    frc::SmartDashboard::PutNumber("rug Angle", m_rugConfig.direction.angle());
    frc::SmartDashboard::PutNumber("shiftDistance", m_rugConfig.shiftDistance);
    frc::SmartDashboard::PutNumber("shiftDistanceK", m_rugConfig.shiftDistanceK);
    frc::SmartDashboard::PutNumber("perpShiftDistance", m_rugConfig.perpShiftDistance);
    frc::SmartDashboard::PutNumber("perpShiftDistanceK", m_rugConfig.perpShiftDistanceK);
  }
}

/**
 * Resets all odometry by clearing states map
 * 
 * @param curTime current robot time (from startup), in ms
*/
void KalmanFilter::Reset(std::size_t curTime) {
  m_states.clear();

  // add value; we want to make sure there's at least one value in the map at all times
  m_states[curTime].E = m_E0;
  m_states[curTime].ang = 0;
}

/**
 * Predicts current position and angle given wheel velocities and navx
 * 
 * @param vAvgCur average velocity from wheels, rotated (world frame)
 * @param navXAng current navX angle, in radians
 * @param curTime current robot time (from startup), in ms
*/
void KalmanFilter::PredictFromWheels(vec::Vector2D vAvgCur, double navXAng, std::size_t curTime)
{
  // get previous state variables
  auto lastIt = m_states.rbegin();
  std::size_t kPrev = lastIt->first;
  auto posPrev = lastIt->second.pos;
  auto vAvgPrev = lastIt->second.vAvg;
  double ePrev = lastIt->second.E;

  double timeDiff = static_cast<double>(curTime - kPrev) / 1000.0;

  if(m_useSmartDashboard){
    double newAngle = frc::SmartDashboard::GetNumber("rug Angle", m_rugConfig.direction.angle());
    m_rugConfig.direction = vec::Vector2D{1, 0}.rotate(newAngle); //Where the rug points
    m_rugConfig.perpDirection = m_rugConfig.direction.rotate(M_PI/2.0);
    m_rugConfig.shiftDistance = frc::SmartDashboard::GetNumber("shiftDistance", m_rugConfig.shiftDistance);
    m_rugConfig.shiftDistanceK = frc::SmartDashboard::GetNumber("shiftDistanceK", m_rugConfig.shiftDistanceK);
    m_rugConfig.perpShiftDistance = frc::SmartDashboard::GetNumber("perpShiftDistance", m_rugConfig.perpShiftDistance);
    m_rugConfig.perpShiftDistanceK = frc::SmartDashboard::GetNumber("perpShiftDistanceK", m_rugConfig.perpShiftDistanceK);
  }

  Vector posDiff = vAvgCur * timeDiff;
  //carpet math, different behavior depending on how the drivebase drives on the rug
  double rugAlignment = posDiff.dot(m_rugConfig.direction);
  double rugPerpAlignment = posDiff.dot(m_rugConfig.perpDirection);
  //Moving along = pushing back hairs
  if(rugAlignment > 0){ //If moving in direction of rug
    rugAlignment *= m_rugConfig.perpShiftDistanceK; //wheels are moving more, odometry excess
    if(m_lastRugDir.x() < 0){ //If changed from moving along rug to not, shifts hairs back
       rugAlignment -= m_rugConfig.perpShiftDistance; //Shift odometry back against direction of hairs
    }
  }
  else if(m_lastRugDir.x() > 0){//If changed from moving against rug to not, shifts hairs forward
    rugAlignment += m_rugConfig.perpShiftDistance; //Shift odometry forward with direction of hairs
  }
  //Perpendicular movement (if it exists)
  rugPerpAlignment *= m_rugConfig.perpShiftDistanceK; //Always shifts by a factor (hair can move side to side)
  if(rugPerpAlignment > 0){ //Check change direction
    if(m_lastRugDir.y() < 0){ //If changed from moving along rug to not, shifts hairs back
      rugPerpAlignment -= m_rugConfig.perpShiftDistance; //Shift odometry back against direction of hairs
    }
  }
  else if(m_lastRugDir.y() > 0){//If changed from moving against rug to not, shifts hairs forward
    rugPerpAlignment += m_rugConfig.perpShiftDistance; //Shift odometry forward with direction of hairs
  }
  m_lastRugDir = Vector{rugAlignment, rugPerpAlignment};

  posDiff = (m_rugConfig.direction*rugAlignment) + (m_rugConfig.perpDirection*rugPerpAlignment);

  // position is previous position + time difference
  auto pos = posPrev + posDiff;
  auto vAvg = vAvgCur;
  double ang = navXAng;
  double E = ePrev + m_Q; // error covariance amplifies

  // assign values to map
  m_states[curTime].pos = pos;
  m_states[curTime].ang = ang;
  m_states[curTime].vAvg = vAvgCur;
  m_states[curTime].E = E;

  // delete map values > maxTime seconds ago, unless it's the last value in the map
  auto firstIt = m_states.lower_bound(curTime - static_cast<std::size_t>(m_maxTime * 1000.0)); // first iterator to include
  m_states.erase(m_states.begin(), firstIt);

  // if empty, insert zeroes
  // we want to make sure there is at least one value at all times
  if (m_states.size() == 0) {
    m_states[curTime].E = m_E0;
    m_states[curTime].ang = 0;
  }
  // this will mess up odometry but at least no runtime errors
}

/**
 * Updates data from camera
 * 
 * @param pos Robot's position relative to the field from the camera
 * @param angZ Angle reading from camera (or navX if camera angle is bad), in radians
 * @param timeOffset Difference in time between now and the time that the camera reading was read, in ms
 * @param curTime current robot time (from startup), in ms
*/
void KalmanFilter::UpdateFromCamera(vec::Vector2D pos, double angZ, std::size_t timeOffset, std::size_t curTime) {
  // if > maxTime before, ignore
  // std::cout << "here 1: " << curTime << " " << timeOffset << std::endl;
  if (static_cast<double>(timeOffset) > m_maxTime * 1000.0) {
    return;
  }

  std::size_t measurementTime = curTime - timeOffset; // time of camera measurement, in ms

  // state nearest to measurement time
  auto measurementIt = m_states.lower_bound(measurementTime);
  // make sure iterator exists
  if (measurementIt == m_states.end()) {
    return;
  }

  pos -= *m_posOffset;
  angZ -= *m_angOffset;

  // get predicted state variables
  auto posPred = measurementIt->second.pos;
  auto vAvgPred = measurementIt->second.vAvg;
  auto ePred = measurementIt->second.E;
  auto angPred = measurementIt->second.ang;

   // don't correct if no error covariance
  if (ePred == 0) {
    return;
  }

  // corrects position
  double camNoise = m_kPos * vec::magn(vAvgPred) + m_kPosInt; // cam noise is proportional to robot velocity
  double kalmanGain = ePred / (ePred + camNoise); // calculates kalman gain
  // std::cout << "ePred: " << ePred << " camNoise: " << camNoise << " gain: " << kalmanGain << std::endl;
  vec::Vector2D correctedPos = posPred + (pos - posPred) * kalmanGain; // corrects position
  double e = (1 - kalmanGain) / ePred; // corrects error

  // corrects angle
  double alpha = 0.1 / (1 + std::exp(m_kAng * (vec::magn(vAvgPred) - 0.5))); // how much to trust angle from camera
  double ang = alpha * angZ + (1 - alpha) * angPred;
  double angDiff = ang - angPred;

  // previous states and times
  KalmanState prevState; // used to store previous state of timestep
  prevState.ang = ang;
  prevState.pos = correctedPos;
  prevState.E = e;
  prevState.vAvg = vAvgPred;
  std::size_t prevTime = measurementTime; // used to store previous time

  measurementIt++;

  // updates states from measurementTime to map end
  for (auto it = measurementIt; it != m_states.end(); it++) {
    // time difference
    double timeDiff = static_cast<double>(it->first - prevTime) / 1000.0;

    // average velocity of current time step
    auto vAvgCur = it->second.vAvg;

    // new position is previous position + time difference
    double ang = it->second.ang + angDiff;
    auto pos = prevState.pos + vAvgCur * timeDiff;
    auto vAvg = vAvgCur;
    double E = prevState.E + m_Q; // error covariance amplifies

    // assign values to state
    prevState.pos = pos;
    prevState.ang = ang;
    prevState.vAvg = vAvg;
    prevState.E = E;
    prevTime = it->first;

    // updates state, then prevState is used for next iteration
    it->second = prevState;
  }

  // delete map values > maxTime seconds ago, unless it's the last value in the map
  auto firstIt = m_states.lower_bound(curTime - static_cast<std::size_t>(m_maxTime * 1000.0)); // first iterator to include
  m_states.erase(m_states.begin(), firstIt);

  // if empty, insert zeroes
  // we want to make sure there is at least one value at all times
  if (m_states.size() == 0) {
    m_states[curTime].E = m_E0;
    m_states[curTime].ang = 0;
  }
  // this will mess up odometry but at least no runtime errors
}

/**
 * Sets Kalman Filter terms
 * 
 * It is recommended to reset odometry after setting terms
 * 
 * All of these must be > 0 or things will break
 * 
 * @param E0 initial error covariance
 * @param Q noise of wheels
 * @param kAng angle constant in logistic function higher = lower trust in camera for higher velocities
 * @param k constant of proportionality between speed and camera noise
 * @param kPosInt camera constant noise of camera when not moving
 * @param maxTime max time before ignore, in s
*/
void KalmanFilter::SetTerms(double E0, double Q, double kAng, double k, double kPosInt, double maxTime) {
  m_E0 = E0;
  m_Q = Q;
  m_kAng = kAng;
  m_kPos = k;
  m_kPosInt = kPosInt;
  m_maxTime = maxTime;
}

/**
 * Gets estimated position
 * 
 * @returns Estimated position
*/
vec::Vector2D KalmanFilter::GetEstimatedPos() const {
  auto itLatest = m_states.rbegin();
  return itLatest->second.pos;
}

/**
 * Gets estimated angle
 * 
 * @returns Estimated angle
*/
double KalmanFilter::GetEstimatedAng() const {
  auto itLatest = m_states.rbegin();
  return itLatest->second.ang;
}