/**
 * Compelmentary filter
*/

#pragma once

#include <cstddef>
#include <map>

#include "Util/thirdparty/simplevectors.hpp"

namespace vec = svector;

/**
 * Complimentary Filter for combining camera & wheel data
*/
class CompFilter {
public:
  struct State {
    vec::Vector2D pos;
    vec::Vector2D vel;
  };

  CompFilter(double alpha, double maxTime, vec::Vector2D *posOffset);

  void AddWheelVel(vec::Vector2D worldVel, std::size_t curTime);
  void AddCamPos(vec::Vector2D camPos, std::size_t timeOffset, std::size_t curTime);

  void SetAlpha(double alpha);
  void SetMaxTime(double maxTime);
  void Reset(std::size_t curTime);

  double GetAlpha() const;
  vec::Vector2D GetPos() const;

private:
  double m_alpha;
  double m_maxTime;

  std::map<std::size_t, State> m_states;

  vec::Vector2D curPos;

  vec::Vector2D *m_posOffset;
};