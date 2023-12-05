#pragma once

#include "TrapezoidalProfile.h"
#include "Util\Poses.h"

#include "ShuffleboardSender\ShuffleboardSender.h"

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
            double ks;
            double kv;
            double ka;
            double kg = 0.0;
        };

        enum State{
            TUNING,
            HOLDING_POS,
            RECENTER_FROM_MIN,
            RECENTER_FROM_MAX
        };

        struct Bounds{
            double min;
            double max;
        };

        FFAutotuner(std::string name, FFType type, double min = 0.0, double max = 0.0, double targTime = 1.0, double testTime = 0.0);
        double getVoltage(Poses::Pose1D currPose);

        void zeroBounds(double val = 0.0);
        void expandBounds(double val);
        void setMin(double min);
        void setMax(double max);

        FFConfig getFeedforward();
        void setFeedforward(FFConfig config);

        void ShuffleboardUpdate();
    private:
        void resetProfile(bool center);
        void resetError();

        std::string name_;
        FFType ffType_;
        State state_;
        Poses::Pose1D currPose_;
        double lastTime_;

        TrapezoidalProfile profile_;
        double targTime_;
        double testTime_;

        Bounds bounds_;

        FFConfig ffTesting_;
        double s_ = 0.1;

        struct FFError{
            FFConfig gainError;
            Poses::Pose1D totalError;
            Poses::Pose1D absTotalError;
        } error_;

        ShuffleboardSender ShuffData_;
};