#pragma once

#include "TrapezoidalProfile.h"
#include "Poses.h"

#include "ShuffleboardSender/ShuffleboardSender.h"

/**
 * Auto tuner class
 * 
 * Call the getVoltage periodically to start tuning
*/
class FFAutotuner{
    public:
        enum FFType{
            SIMPLE,
            ARM,
            ELEVATOR
        };

        struct FFConfig{
            double kv;
            double ka;
            double ks;
            double kg = 0.0;
        };

        FFAutotuner(std::string name, FFType type, double min = 0.0, double max = 0.0);
        double getVoltage(Poses::Pose1D currPose);

        void zeroBounds(double val = 0.0);
        void expandBounds(double val);
        void setMin(double min);
        void setMax(double max);

    private:
        void resetProfile(Poses::Pose1D currPose);
        void resetError();

        std::string name_;
        FFType ffType_;
        double lastTime_;

        TrapezoidalProfile profile_;
        double expectTime = 10.0;

        struct Bounds{
            double min;
            double max;
        } bounds_;

        FFConfig ffTesting_;
        double s_ = 0.1;

        struct FFError{
            FFConfig gainError;
            Poses::Pose1D totalError;
            Poses::Pose1D absTotalError;
        } error_;

        ShuffleboardSender ShuffData_;
};