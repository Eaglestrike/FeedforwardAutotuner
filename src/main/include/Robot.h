// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>

#include <string>
#include <iostream>

#include <AHRS.h>

#include "RobotConstants.h"
#include "Drive\SwerveControl.h"
#include "FFAutotuner\FFAutotuner.h"
#include "Controller\Controller.h"

#include "ShuffleboardSender\ShuffleboardSender.h"

class Robot : public frc::TimedRobot {
    public:
        void RobotInit() override;
        void RobotPeriodic() override;
        void AutonomousInit() override;
        void AutonomousPeriodic() override;
        void TeleopInit() override;
        void TeleopPeriodic() override;
        void DisabledInit() override;
        void DisabledPeriodic() override;
        void TestInit() override;
        void TestPeriodic() override;
        void SimulationInit() override;
        void SimulationPeriodic() override;

    private:
        SwerveDrive drive_{"Drivebase", true, true, true};
        
        bool tuning = false;
        FFAutotuner tunerX_{"Swerve X", FFAutotuner::SIMPLE};
        FFAutotuner tunerY_{"Swerve Y", FFAutotuner::SIMPLE};
        FFAutotuner tunerAng_{"Swerve Ang", FFAutotuner::SIMPLE};

        AHRS* navx_;
        Controller controls_;

        ShuffleboardSender ShuffData_{"Robot"};
};

