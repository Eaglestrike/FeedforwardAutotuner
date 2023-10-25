#pragma once

#include <ctre/Phoenix.h>

#include <frc2/command/PIDCommand.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <units/voltage.h>

#include <string>
#include <cmath>
#include <iostream>

#include "Geometry/Point.h"
#include "SwervePose.h"
#include "SwerveConstants.h"
#include "ShuffleboardSender/ShuffleboardSender.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846	/* pi */
#endif

class SwerveModule{
    public:
        SwerveModule() = default;
        SwerveModule(SwerveConstants::SwerveStruct swerveMod);

        void Periodic();
        void TeleopInit();
        void TeleopPeriodic();
        void DisabledInit();
        void DisabledPeriodic();

        void zero();

        void setTarget(SwervePose::ModulePose pose, bool volts = true);

        void enableShuffleboard(bool edit = false);
        void disableSuffleboard();

        std::string getName();
        Point getPos();
        Vector getVel();

    private:
        const std::string name_;

        WPI_TalonFX driveMotor_;
        units::volt_t driveVolts_{0.0};
        double maxDriveVolts_ = SwerveConstants::DRIVE_MAX_VOLTS;//volts
        
        WPI_TalonFX turnMotor_;
        units::volt_t turnVolts_{0.0};
        double maxTurnVolts_ = SwerveConstants::TURN_MAX_VOLTS;

        WPI_CANCoder cancoder_;
        double encoderOffset_;
        bool encoderInverted_;

        frc::PIDController turnPID_{0, 0, 0};
        SwervePose::ModulePose targetPose_;//either m/s or volts for drive
        bool volts_; //using volts or nah

        SwervePose::ModulePose currPose_; //normal units (m, rad)
        Vector vel_;

        Point pos_ = {0, 0}; //Position on robot, accessed by swerveDrive, stored in module

        bool inverted_ = false;

        ShuffleboardSender ShuffData_;
};