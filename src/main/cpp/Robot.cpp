// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

using namespace Actions;

void Robot::RobotInit() {
  try{
      navx_ = new AHRS(RobotConstants::NAVX::PORT);
  }
  catch (const std::exception &e){
      std::cout << e.what() << std::endl;
  }
  navx_->ZeroYaw();
  drive_.setNAVX(navx_);
  drive_.enableShuffleboard(true, true);
  drive_.reset();
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  if(controls_.getPressed(ZERO)){
    drive_.zero();
  }
  drive_.Periodic();
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
}

void Robot::AutonomousPeriodic() {
}

void Robot::TeleopInit() {
  drive_.TeleopInit();
}

void Robot::TeleopPeriodic() {
  double xStrafe = controls_.getWithDeadContinuous(XSTRAFE) * SwerveConstants::DRIVE_MAX_VOLTS;
  double yStrafe = -controls_.getWithDeadContinuous(YSTRAFE) * SwerveConstants::DRIVE_MAX_VOLTS;
  double rotation = controls_.getWithDeadContinuous(ROTATION) * SwerveConstants::TURN_MAX_VOLTS;
  drive_.SetTarget({xStrafe, yStrafe}, rotation); 
  drive_.TeleopPeriodic();
}

void Robot::DisabledInit() {
  drive_.DisabledInit();
}

void Robot::DisabledPeriodic() {
  drive_.DisabledPeriodic();
}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
