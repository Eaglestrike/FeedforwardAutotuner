/**
 * @file Feedforward.h
 * @author Vir Shah (vir.shah@team114.org)
 * @brief Module for feedforward functions, coupled with PID calculator.
 * @version 0.1
 * @date 2023-08-22
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <frc/Timer.h>

#include "ElevatorConstants.h"

class FeedforwardPID
{
public:
    // constructor
    FeedforwardPID(ElevatorConstants::FeedforwardConfig constants, bool shuffleboard = false);

    // main methods to use
    double periodic(Poses::Pose1D current_pose);
    Poses::Pose1D getExpectedPose(double time);

    void start();
    void reset();
    void stop();

    bool isFinished();
    bool atSetpoint(Poses::Pose1D current_pose, double xTol, double vTol);

    // getters and setters
    double getKs();
    double getKv();
    double getKa();
    double getKg();

    double getKp();
    double getKd();

    double getMaxVelocity();
    double getMaxAcceleration();

    bool getReversed();
    double getStartpoint();
    double getSetpoint();

    void setKs(double ks);
    void setKv(double kv);
    void setKa(double ka);
    void setKg(double kg);
    void setTotalDistance(double new_position, double curr_pos);
    void setReversed(bool reversed);

    void setPIDConstants(double kp, double ki, double kd);

    void setMaxVelocity(double max_vel);
    void setMaxAcceleration(double max_acc);

private:
    // member functions
    double sign(double value);
    double calculateFeedforwardVoltage(double velocity, double acceleration);
    double calculatePIDVoltage(Poses::Pose1D expected, Poses::Pose1D current, double dt);
    void recalculateTimes();

    // total distance needed to travel
    double startpoint_;
    double setpoint_;
    double total_distance_;

    // feedforward constants
    double ks, kv, ka, kg;

    // velocity, acceleration constants
    double max_velocity;
    double max_acceleration;

    // pid constants
    double kp;
    double ki;
    double kd;
    double accum_;

    // calculated constants
    double acceleration_time;
    double velocity_time;

    // used to control the timer
    bool isRunning;

    // to control direction of feedforward path
    bool reversed;

    // timer
    frc::Timer timer{};

    bool shuffleboard;

    double prevTime_;
};