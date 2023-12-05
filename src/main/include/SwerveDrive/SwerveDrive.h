#pragma once

#include <string>
#include <array>
#include <iostream>

#include <AHRS.h>

#include "Util/Mechanism.h"

#include "SwerveModule.h"
#include "SwerveConstants.h"
#include "SwervePose.h"

#include "ShuffleboardSender/ShuffleboardSender.h"

class SwerveDrive : public Mechanism{
    public:
        SwerveDrive(std::string name, bool enabled, bool shuffleboard, bool mShuffleboard = false);
        void reset();
        void zero();

        void drive();
        
        void SetTarget(Vector v, double angV, bool volts = true);

        SwervePose::Pose getCurrPose();
        
        void setNAVX(AHRS* navx) {navx_ = navx;}

    private:
        void CoreInit() override;
        void CorePeriodic() override;
        void CoreTeleopInit() override;
        void CoreTeleopPeriodic() override;
        void CoreDisabledInit() override;
        void CoreDisabledPeriodic() override;

        void CoreShuffleboardInit() override;
        void CoreShuffleboardPeriodic() override;

        void updatePose();

        const std::string name_;
        SwerveModule* modules_[SwerveConstants::NUMSWERVE];

        Point pivot_{0.0, 0.0};

        bool isHoldingAng_;
        double holdingAng_;

        SwervePose::Pose targetPose_;
        SwervePose::Pose currentPose_;
        bool volts_ = true;

        double lastUpdate_;

        AHRS* navx_;
};