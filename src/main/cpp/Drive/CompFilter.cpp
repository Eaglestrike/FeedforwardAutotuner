#include "Drive/CompFilter.h"

#include <algorithm>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Util/Utils.h"

/**
 * Constructor
 * 
 * @param alpha A value between 0 and 1 to determine weight of trust towards wheel odometry
 * @param maxTime maximum time before discarding measurements, in s
 * @param posOffset pointer to Position offset in robot cpp
*/
CompFilter::CompFilter(double alpha, double maxTime, vec::Vector2D *posOffset) 
  : m_alpha{std::clamp(alpha, 0.0, 1.0)}, m_maxTime{maxTime}, m_posOffset{posOffset} {
  m_states[0].pos = *posOffset;
  m_states[0].vel = {0, 0};
}

/**
 * Adds data from wheel odometry
 * 
 * @param worldVel Averaged velocities of wheels in world frame (after rotating navX angle)
 * @param curTime current robot time (from startup), in ms
*/
void CompFilter::AddWheelVel(vec::Vector2D worldVel, std::size_t curTime) {
  auto lastIt = m_states.rbegin(); 

  std::size_t lastTime = lastIt->first;
  vec::Vector2D lastPos = lastIt->second.pos;
  double deltaT = (curTime - lastTime) / 1000.0;

  // integrate velocity to get position
  vec::Vector2D curPos = lastPos + worldVel * deltaT;

  m_states[curTime].pos = curPos;
  m_states[curTime].vel = worldVel;

  // delete map values > maxTime seconds ago, unless it's the last value in the map
  auto firstIt = m_states.lower_bound(curTime - static_cast<std::size_t>(m_maxTime * 1000.0)); // first iterator to include
  if (firstIt != m_states.end()) {
    m_states.erase(m_states.begin(), firstIt);
  }

  // if empty, insert zeroes
  // we want to make sure there is at least one value at all times
  if (m_states.size() == 0) {
    m_states[curTime].pos = *m_posOffset;
    m_states[curTime].vel = {0, 0};
  }
  // this will mess up odometry but at least no runtime errors
}

/**
 * Adds data from camera odometry
 * 
 * @param camPos Position of robot given apriltag data
 * @param timeOffset Difference in time between now and the time that the camera reading was read, in ms
 * @param curTime current robot time (from startup), in ms
*/
void CompFilter::AddCamPos(vec::Vector2D camPos, std::size_t timeOffset, std::size_t curTime) {
  std::size_t timeOfCam = curTime - timeOffset;
  auto itOfCam = m_states.lower_bound(timeOfCam);

  // error checking
  if (itOfCam == m_states.end()) {
    return;
  }

  // correct position at time of receiving data from camera
  // std::size_t timeNearCam = itOfCam->first;
  vec::Vector2D posNearTime = itOfCam->second.pos;
  vec::Vector2D correctedPosCam = posNearTime * m_alpha + camPos * (1 - m_alpha);

  itOfCam->second.pos = correctedPosCam;

  // correct position from iterator to end
  for (auto it = ++itOfCam; it != m_states.end(); it++) {
    auto prevIt = it; 
    prevIt--;

    std::size_t lastTime = prevIt->first;
    std::size_t curTime = it->first;
    vec::Vector2D lastPos = prevIt->second.pos;
    vec::Vector2D curVel = prevIt->second.vel;

    double deltaT = (curTime - lastTime) / 1000.0;

    // integrate velocity to get position
    vec::Vector2D curPos = lastPos + curVel * deltaT;

    State newState;
    newState.pos = curPos;
    newState.vel = curVel;
    
    it->second = newState;
  }

  // delete map values > maxTime seconds ago, unless it's the last value in the map
  auto firstIt = m_states.lower_bound(curTime - static_cast<std::size_t>(m_maxTime * 1000.0)); // first iterator to include
  if (firstIt != m_states.end()) {
    m_states.erase(m_states.begin(), firstIt);
  }

  // if empty, insert zeroes
  // we want to make sure there is at least one value at all times
  if (m_states.size() == 0) {
    m_states[curTime].pos = *m_posOffset;
    m_states[curTime].vel = {0, 0};
  }
  // this will mess up odometry but at least no runtime errors
}

/**
 * Sets trust term for wheels between 0 and 1
 * 
 * @param alpha Trust term, between 0 and 1
*/
void CompFilter::SetAlpha(double alpha) {
  m_alpha = std::clamp(alpha, 0.0, 1.0);
}

/**
 * Sets maximum time before discaarding time
 * 
 * @param maxTime max time
*/
void CompFilter::SetMaxTime(double maxTime) {
  if (maxTime <= 0) {
    return;
  }

  m_maxTime = maxTime;
}

/**
 * Resets all odometry by clearing states map
 * 
 * @param curTime current robot time (from startup), in ms
*/
void CompFilter::Reset(std::size_t curTime) {
  m_states.clear();

  // add value; we want to make sure there's at least one value in the map at all times
  m_states[curTime].pos = *m_posOffset;
  m_states[curTime].vel = {0, 0};
}

/**
 * Gets wheel odometry trust term
 * 
 * @returns Wheel odometry trust term
*/
double CompFilter::GetAlpha() const {
  return m_alpha;
}

/**
 * Gets estimated position
 * 
 * @returns Estimated position
*/
vec::Vector2D CompFilter::GetPos() const {
  auto itLatest = m_states.rbegin();

  if (itLatest == m_states.rend()) {
    return {0, 0};
  }

  return itLatest->second.pos;
}
