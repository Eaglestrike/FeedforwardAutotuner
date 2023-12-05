#pragma once

#include <ctre/Phoenix.h>

#include "Elevator/ElevatorIntakeConstants.h"
#include "Elevator/Lidar/LidarReader.h"

class Rollers {
public:
  enum RollerState {
    INTAKE,
    INTAKE_STRONG, //Hold and intake strong
    RETAIN,
    STOP,
    OUTTAKE
  };

  Rollers() = default;

  void SetCone(bool cone);
  void UpdateLidarData(LidarReader::LidarData lidardata);

  void Intake(bool force = false);
  void HoldIntake(bool on);
  void Outtake(bool force = false);
  void Stop();
  void Periodic();

private:
  bool m_cone = true, m_hasGamePiece = false;
  double m_hasGamePieceStart = 0;
  RollerState m_state{STOP};
  WPI_TalonFX m_rollerMotor{IntakeConstants::ROLLER_MOTOR_ID};
};