#pragma once

#include <ctre\Phoenix.h>

#include <frc2\command\PIDCommand.h>
#include <frc\shuffleboard\Shuffleboard.h>
#include <units\voltage.h>

#include <string>
#include <cmath>
#include <iostream>

#include "Util/Point.h"
#include "Util/Mechanism.h"

#include "SwervePose.h"
#include "SwerveConstants.h"
#include "ShuffleboardSender\ShuffleboardSender.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846	/* pi */
#endif

class SwerveModule: public Mechanism{
    public:
        SwerveModule() = default;
        SwerveModule(SwerveConstants::SwerveStruct swerveMod, bool enabled, bool shuffleboard);

        void zero();

        void setTarget(SwervePose::ModulePose pose, bool volts = true);

        void enableShuffleboard(bool edit = false);
        void disableSuffleboard();

        std::string getName();
        Point getPos();
        Vector getVel();

    private:
        void CorePeriodic() override;
        void CoreTeleopInit() override;
        void CoreTeleopPeriodic() override;
        void CoreDisabledInit() override;
        void CoreDisabledPeriodic() override;

        void CoreShuffleboardInit() override;
        void CoreShuffleboardPeriodic() override;

        const std::string name_;

        WPI_TalonFX driveMotor_;
        units::volt_t driveVolts_;
        double maxDriveVolts_ = SwerveConstants::DRIVE_MAX_VOLTS;//volts
        
        WPI_TalonFX turnMotor_;
        units::volt_t turnVolts_;
        double maxTurnVolts_ = SwerveConstants::TURN_MAX_VOLTS;

        WPI_CANCoder cancoder_;
        double encoderOffset_;
        bool encoderInverted_;

        frc::PIDController turnPID_;
        SwervePose::ModulePose targetPose_;//either m/s or volts for drive
        bool volts_; //using volts or nah

        SwervePose::ModulePose currPose_; //normal units (m, rad)
        Vector vel_;

        Point pos_; //Position on robot, accessed by swerveDrive, stored in module

        bool inverted_ = false;
};