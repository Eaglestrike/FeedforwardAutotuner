/**
 * A file of constants
 *
 * @note General constants are not namespaced
 */

#pragma once

#include <cstddef>
#include <map>

namespace Poses {
    struct Pose1D {
        double velocity = 0.0;
        double acceleration = 0.0;
        double position = 0.0;
    };
};

namespace ElevatorConstants {
  enum ElevatorTarget{
      CUSTOM,
      LOW,
      MID,
      HIGH,
      STOWED
  };
  
  const double TALON_FX_COUNTS_PER_REV = 2048;
  const double ONE_MOTOR_REVOLUTION_TO_DISTANCE_TRAVELLED = 0.0465582;

  // motor ids
  const int LEFT_MOTOR_ID = 22;
  const int RIGHT_MOTOR_ID = 40;

  //Limit switch
  const int LIMIT_SWITCH_ID = 1;

  const double MAX_VOLTS = 6.0;

  // const double MAX_ELEVATOR_VELOCITY = 4.20116;
  const double MAX_VELOCITY = 2.34; //m/s
  const double MAX_ACCELERATION = 1.44; //m/s^2

  const double MAX_EXTENSION = 0.5588; // m

  //Feedforward Constants
  const double KS = 0;
  const double KV = 0;
  const double KA = 0.45;
  const double KG = 1.1;

  const double KP = 8.5;
  const double KI = 0.35;
  const double KD = 5.1;

  // elevator heights

  const double STOWED_HEIGHT = 0.0;
  // TODO: replace these numbers with the actual heights
  const double LOW_HEIGHT = MAX_EXTENSION / 3.0;
  const double MID_HEIGHT = MAX_EXTENSION / 2.0;
  const double HIGH_HEIGHT = MAX_EXTENSION;

  const std::map<ElevatorTarget, double> TARGET_TO_HEIGHT = {
    {LOW, LOW_HEIGHT},
    {MID, MID_HEIGHT},
    {HIGH, HIGH_HEIGHT},
    {STOWED, STOWED_HEIGHT},
    {CUSTOM, 0.0} //For safety reasons
  };


  const struct FeedforwardConfig{
    double ks = KS;
    double kv = KV;
    double ka = KA;
    double kg = KG;
    double maxVel = MAX_VELOCITY;
    double maxAccel = MAX_ACCELERATION;
    double kp = KP;
    double ki = KI;
    double kd = KD;
  } FEEDFORWARD_CONSTANTS;

  const double POSITION_ERROR_TOLERANCE = 0.07; //m
};