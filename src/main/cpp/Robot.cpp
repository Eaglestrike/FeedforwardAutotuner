// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399
#endif

#include <fmt/core.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Drive/DriveConstants.h"

#include "Controller/ControllerMap.h"

#include "Util/Utils.h"

#include "GeneralConstants.h"

using namespace Actions;

Robot::Robot():
      m_prevTime{0},
      m_swerveFr{SwerveConstants::FR_CONFIG, true, false},
      m_swerveBr{SwerveConstants::BR_CONFIG, true, false},
      m_swerveFl{SwerveConstants::FL_CONFIG, true, false},
      m_swerveBl{SwerveConstants::BL_CONFIG, true, false},
      m_startPos{0, 0},
      m_startAng{0},
      m_joystickAng{0},
      m_odometry{&m_startPos, &m_startAng},
      m_lidar{true, false},
      m_client{"10.1.14.21", 5807, 500, 5000},
      m_red{false},
      m_posVal{0},
      m_heightVal{0}
{
  // swerve
  SwerveControl::RefArray<SwerveModule> moduleArray{{m_swerveFr, m_swerveBr, m_swerveFl, m_swerveBl}};
  m_swerveController = new SwerveControl(moduleArray, true, false);

  // navx
  try{
    m_navx = std::make_shared<AHRS>(frc::SerialPort::kUSB2);
  }
  catch (const std::exception &e){
    std::cerr << e.what() << std::endl;
  }

  // lidar
  m_lidar.setAutoRequest(true);

  AddPeriodic([&](){
    // ODOMETRY
    // int expectedId = Utils::GetExpectedTagId(m_posVal, m_red);

    // process camera data
    std::vector<double> camData = m_client.GetData();
    if (!m_isAutoLineup && m_client.HasConn() && !m_client.IsStale()) {
      //int camId = static_cast<int>(camData[0]);
      int tagId = static_cast<int>(camData[1]);
      double x = camData[2];
      double y = camData[3];
      double angZ = camData[4];
      long long age = static_cast<long long>(camData[5]);
      unsigned long long uniqueId = static_cast<unsigned long long>(camData[6]);

      frc::SmartDashboard::PutNumber("camX", x);
      frc::SmartDashboard::PutNumber("camY", y);

      bool res = false;
      if (tagId != 0 && m_isSecondTag) {
        res = m_odometry.SetCamData({x, y}, angZ, tagId, age, uniqueId);
      } 

      frc::SmartDashboard::PutNumber("tag Id", tagId);

      // frc::SmartDashboard::PutBoolean("Good ID", res);
      if (!res) {
        tagId = 0;
      }
      m_isSecondTag = true;
    } else {
      m_isSecondTag = false;
    }
    // frc::SmartDashboard::PutBoolean("Is correcting", isCorrecting);

    // other odometry
    double angNavX = Utils::DegToRad(m_navx->GetYaw());
    double pitch = Utils::DegToRad(m_navx->GetPitch());
    double roll = Utils::DegToRad(m_navx->GetRoll());
    pitch = Utils::NormalizeAng(pitch - SwerveConstants::PITCH_OFFSET);
    roll = Utils::NormalizeAng(roll - SwerveConstants::ROLL_OFFSET);

    vec::Vector2D velWorld = m_swerveController->GetRobotVelocity(angNavX + m_startAng);

    m_odometry.Periodic(angNavX, velWorld);

    vec::Vector2D pos = m_odometry.GetPosition();
    double ang = m_odometry.GetAng();
    vec::Vector2D wheelVel = m_swerveController->GetRobotVelocity(ang);
    m_field.SetRobotPose(units::meter_t{pos.x()}, units::meter_t{pos.y()}, units::radian_t{ang});

    double tilt = -roll * std::sin(angNavX) - pitch * std::cos(angNavX);

    m_autoLineup.UpdateOdom(pos, ang, wheelVel);
    m_autoPath.UpdateOdom(pos, ang, wheelVel);

    frc::SmartDashboard::PutString("Robot pos", pos.toString());
    frc::SmartDashboard::PutNumber("Robot ang", ang);
    frc::SmartDashboard::PutNumber("Robot pitch", pitch);
    frc::SmartDashboard::PutNumber("Robot roll", roll);
    frc::SmartDashboard::PutNumber("Robot tilt", tilt);
    frc::SmartDashboard::PutBoolean("Cam stale", m_client.IsStale());
    frc::SmartDashboard::PutBoolean("Cam connection", m_client.HasConn());
    frc::SmartDashboard::PutData("Field", &m_field);

    // END ODOMETRY
  }, 5_ms, 2_ms);
}

void Robot::RobotInit()
{
  frc::SmartDashboard::PutNumber("Filter Alpha", OdometryConstants::ALPHA);

  // starting position
  m_startPosChooser.SetDefaultOption("Debug", "Debug");
  m_startPosChooser.AddOption("Blue L", "Blue L");
  m_startPosChooser.AddOption("Blue M", "Blue M");
  m_startPosChooser.AddOption("Blue R", "Blue R");
  m_startPosChooser.AddOption("Red L", "Red L");
  m_startPosChooser.AddOption("Red M", "Red M");
  m_startPosChooser.AddOption("Red R", "Red R");
  frc::SmartDashboard::PutData("Starting pos", &m_startPosChooser);

  m_navx->ZeroYaw();
  m_swerveController->ResetAngleCorrection();

  // Starts recording to data log

  m_elevatorIntake.Init();
  m_lidar.Init();
  m_client.Init();
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
  frc::SmartDashboard::PutBoolean("Pos at target", m_autoLineup.GetPosExecuteState() == AutoLineup::AT_TARGET);
  frc::SmartDashboard::PutBoolean("Ang at target", m_autoLineup.GetAngExecuteState() == AutoLineup::AT_TARGET);

  if (m_controller.getPressedOnce(ZERO_DRIVE_PID)){
    double alpha = frc::SmartDashboard::GetNumber("Filter Alpha", OdometryConstants::ALPHA);
    m_odometry.SetAlpha(alpha);

    double tkP = frc::SmartDashboard::GetNumber("ltrans kP", LineupConstants::TRANS_KP);
    double tkI = frc::SmartDashboard::GetNumber("ltrans kI", LineupConstants::TRANS_KI);
    double tkD = frc::SmartDashboard::GetNumber("ltrans kD", LineupConstants::TRANS_KD);
    double akP = frc::SmartDashboard::GetNumber("lang kP", LineupConstants::ANG_KP);
    double akI = frc::SmartDashboard::GetNumber("lang kI", LineupConstants::ANG_KI);
    double akD = frc::SmartDashboard::GetNumber("lang kD", LineupConstants::ANG_KD);

    m_autoLineup.SetPosPID(tkP, tkI, tkD);
    m_autoLineup.SetAngPID(akP, akI, akD);

    double tMaxSp = frc::SmartDashboard::GetNumber("trans maxSp", AutoConstants::TRANS_MAXSP);
    double tMaxAcc = frc::SmartDashboard::GetNumber("trans maxAcc", AutoConstants::TRANS_MAXACC);
    double aMaxSp = frc::SmartDashboard::GetNumber("ang maxSp", AutoConstants::ANG_MAXSP);
    double aMaxAcc = frc::SmartDashboard::GetNumber("ang maxAcc", AutoConstants::ANG_MAXACC);
    m_autoLineup.SetPosFF({tMaxSp, tMaxAcc});
    m_autoLineup.SetAngFF({aMaxSp, aMaxAcc});

    m_elevatorIntake.UpdateShuffleboard();
  }

  if (m_controller.getPressedOnce(ZERO_YAW)){
    m_navx->ZeroYaw();
    m_swerveController->ResetAngleCorrection(m_startAng);
    m_odometry.Reset();
    m_elevatorIntake.ZeroIntake();
  }

  // get position offsets (sorry for bad if statements)
  std::string m_selected = m_startPosChooser.GetSelected();
  if (m_selected == "Debug") {
    m_startAng = FieldConstants::DEBUG_ANG;
    m_startPos = FieldConstants::DEBUG_POS;
    m_joystickAng = FieldConstants::DEBUG_JANG;
    // m_red = false;
  } else if (m_selected == "Blue L") {
    m_startAng = FieldConstants::BL_ANG;
    m_startPos = FieldConstants::BL_POS;
    m_joystickAng = FieldConstants::BL_JANG;
    // m_red = false;
  } else if (m_selected == "Blue M") {
    m_startAng = FieldConstants::BM_ANG;
    m_startPos = FieldConstants::BM_POS;
    m_joystickAng = FieldConstants::BM_JANG;
    // m_red = false;
  } else if (m_selected == "Blue R") {
    m_startAng = FieldConstants::BR_ANG;
    m_startPos = FieldConstants::BR_POS;
    m_joystickAng = FieldConstants::BR_JANG;
    // m_red = false;
  } else if (m_selected == "Red L") {
    m_startAng = FieldConstants::RL_ANG;
    m_startPos = FieldConstants::RL_POS;
    m_joystickAng = FieldConstants::RL_JANG;
    // m_red = true;
  } else if (m_selected == "Red M") {
    m_startAng = FieldConstants::RM_ANG;
    m_startPos = FieldConstants::RM_POS;
    m_joystickAng = FieldConstants::RM_JANG;
    // m_red = true;
  } else if (m_selected == "Red R") {
    m_startAng = FieldConstants::RR_ANG;
    m_startPos = FieldConstants::RR_POS;
    m_joystickAng = FieldConstants::RR_JANG;
    // m_red = true;
  } else {
    m_startAng = FieldConstants::DEBUG_ANG;
    m_startPos = FieldConstants::DEBUG_POS;
    m_joystickAng = FieldConstants::DEBUG_JANG;
    // m_red = false;
  }
  m_red = frc::DriverStation::GetAlliance() == frc::DriverStation::kRed;

  m_elevatorIntake.Periodic();
  m_lidar.Periodic();

  m_elevatorIntake.UpdateLidarData(m_lidar.getData());
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
void Robot::AutonomousInit(){}

void Robot::AutonomousPeriodic(){}

void Robot::TeleopInit() {
  m_swerveController->SetFeedForward(0, 1, 0);
  m_swerveController->SetAngCorrection(true);
  m_posVal = 0;
  m_elevatorIntake.Stow();
}

void Robot::TeleopPeriodic() {
  double t1 = frc::Timer::GetFPGATimestamp().value();
  double curTime = Utils::GetCurTimeS();
  double deltaT = curTime - m_prevTime;

  double lx = m_controller.getWithDeadContinuous(SWERVE_STRAFEX, 0.1);
  double ly = m_controller.getWithDeadContinuous(SWERVE_STRAFEY, 0.1);

  double rx = m_controller.getWithDeadContinuous(SWERVE_ROTATION, 0.1);

  bool fast = !m_controller.getPressed(SLOW);
  double mult = fast ? SwerveConstants::NORMAL_SWERVE_MULT : SwerveConstants::SLOW_SWERVE_MULT;
  double vx = std::clamp(lx, -1.0, 1.0) * mult;
  double vy = std::clamp(ly, -1.0, 1.0) * mult;
  vec::Vector2D setVel = {-vy, -vx};
  double w = -std::clamp(rx, -1.0, 1.0) * mult / 2;

  vec::Vector2D curPos = m_odometry.GetPosition();
  double curYaw = m_odometry.GetAng();

  // get auto lineup pos
  int posVal = m_controller.getValue(ControllerMapData::SCORING_POS, 0);
  if (posVal) {
    m_posVal = posVal;
  }
  int heightVal = m_controller.getValue(ControllerMapData::GET_LEVEL, 0);
  if (heightVal) {
    m_heightVal = heightVal;
  }
  double t2 = frc::Timer::GetFPGATimestamp().value();

  // frc::SmartDashboard::PutNumber("score pos val", m_posVal);
  // frc::SmartDashboard::PutNumber("Height val", m_heightVal);

  vec::Vector2D tgtPos = m_autoLineup.GetTargetPos();
  frc::SmartDashboard::PutString("cur tgt pos", tgtPos.toString());
  frc::SmartDashboard::PutNumber("cur tgt Ang", m_autoLineup.GetTargetAng());

  if (m_posVal && m_heightVal) {
    FieldConstants::ScorePair scorePair = Utils::GetScoringPos(m_posVal, m_heightVal, m_red);
    double ang = Utils::NormalizeAng(m_joystickAng + M_PI);

    vec::Vector2D scorePos = scorePair.first;
    double lidarOffset = scorePair.second;
    double lidarReading = 0;

    if (m_lidar.hasCone()) {
      lidarReading = m_lidar.getConePos() / 100.0;
      scorePos -= {0, m_red ? lidarOffset - lidarReading : lidarReading - lidarOffset};
    } else if (m_lidar.hasCube()) {
      lidarReading = m_lidar.getCubePos() / 100.0;
      scorePos += {0, m_red ? lidarOffset - lidarReading : lidarReading - lidarOffset};
    }
    m_autoLineup.SetPosTarget(scorePos, false);
    m_autoLineup.SetAngTarget(ang, false);
  }
  double t3 = frc::Timer::GetFPGATimestamp().value();

  // trim, offset shold be opposite direction of offset so it moves correct direction
  double trimX = -m_controller.getValueOnce(ControllerMapData::GET_TRIM_X, 0.0);
  double trimY = -m_controller.getValueOnce(ControllerMapData::GET_TRIM_Y, 0.0);
  vec::Vector2D offset = {trimX, trimY};
  offset = rotate(offset, m_red?(M_PI/2.0) : (-M_PI/2.0));

  m_odometry.AddTrimOffset(offset);

  AutoLineup::ExecuteState curPosAutoState = m_autoLineup.GetPosExecuteState();
  AutoLineup::ExecuteState curAngAutoState = m_autoLineup.GetAngExecuteState();
  //                                          don't auto lineup to (0,0)
  if (m_controller.getPressed(AUTO_LINEUP) && m_posVal && m_heightVal) {
    m_swerveController->SetAngCorrection(false);
    // m_isAutoLineup = true;
    double tkP = frc::SmartDashboard::GetNumber("ltrans kP", LineupConstants::TRANS_KP);
    double tkI = frc::SmartDashboard::GetNumber("ltrans kI", LineupConstants::TRANS_KI);
    double tkD = frc::SmartDashboard::GetNumber("ltrans kD", LineupConstants::TRANS_KD);
    m_autoLineup.SetPosPID(tkP, tkI, tkD);

    if (curPosAutoState != AutoLineup::EXECUTING_TARGET) {
      m_autoLineup.StartPosMove();
    }
    if (curAngAutoState != AutoLineup::EXECUTING_TARGET) {
      m_autoLineup.StartAngMove();
    }

    vec::Vector2D driveVel = m_autoLineup.GetVel();
    double angVel = m_autoLineup.GetAngVel();

    m_swerveController->SetFeedForward(SwerveConstants::kS, SwerveConstants::kV, SwerveConstants::kA);
    m_swerveController->SetRobotVelocity(driveVel, angVel, curYaw, deltaT);
  } else {
    m_swerveController->SetAngCorrection(true);
    m_isAutoLineup = false;
    m_autoLineup.StopPos();
    m_autoLineup.StopAng();

    m_swerveController->SetFeedForward(0, 1, 0);
    if (m_controller.getPressed(LOCK_WHEELS)) {
      m_swerveController->Lock();
    } else {
      m_swerveController->SetRobotVelocityTele(setVel, w, curYaw, deltaT, m_joystickAng);
    }
  }
  double t4 = frc::Timer::GetFPGATimestamp().value();

  m_autoLineup.Periodic();
  m_swerveController->Periodic();

  m_autoPath.StartMove();
  m_autoPath.Periodic();

  if (m_controller.getTriggerDown(MANUAL1) && m_controller.getTriggerDown(MANUAL2)) {
    double elH = -m_controller.getWithDeadContinuous(ELEVATOR_H, 0.1);
    double intakeAng = -m_controller.getWithDeadContinuous(INTAKE_ANG, 0.1);
    m_elevatorIntake.ManualPeriodic(elH, intakeAng);
  } else {
    m_elevatorIntake.TeleopPeriodic();
    bool cone = Utils::IsCone(m_posVal);
    m_elevatorIntake.SetCone(cone);
    if(m_controller.getPressedOnce(SCORE_HIGH))
      m_elevatorIntake.ScoreHigh();
    else if (m_controller.getPressedOnce(SCORE_MID))
      m_elevatorIntake.ScoreMid();
    else if (m_controller.getPressedOnce(SCORE_LOW))
      m_elevatorIntake.ScoreLow();
    else if (m_controller.getPressedOnce(STOW))
      m_elevatorIntake.Stow();
    else if (m_controller.getPressedOnce(HP)) {
      m_elevatorIntake.IntakeFromHPS();
      m_posVal = 1;
    }
    else if (m_controller.getPressedOnce(GROUND_INTAKE)) {
      m_elevatorIntake.IntakeFromGround();
      m_posVal = 2;
    }
    else if (m_controller.getPOVDownOnce(INTAKE_FLANGE)){
      m_elevatorIntake.IntakeFlange();
      m_posVal = 1;
    }
  }
  double t5 = frc::Timer::GetFPGATimestamp().value();
  if(t2 - t1 > 0.02){std::cout<<"t1-t2 "<<t2-t1<<std::endl;}
  if(t3 - t2 > 0.02){std::cout<<"t2-t3 "<<t3-t2<<std::endl;}
  if(t4 - t3 > 0.02){std::cout<<"t3-t4 "<<t4-t3<<std::endl;}
  if(t5 - t4 > 0.02){std::cout<<"t4-t5 "<<t5-t4<<std::endl;}
  m_prevTime = curTime;
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
}

void Robot::TestPeriodic() {
}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
