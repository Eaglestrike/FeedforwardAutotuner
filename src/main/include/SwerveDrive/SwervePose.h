#pragma once

#include "Util\Point.h"

namespace SwervePose{
    struct Pose{
        Point pos;//meters
        Vector vel;
        Vector accel;

        double ang; //radians
        double angVel;
        double angAccel;
    };

    struct ModulePose{
        double speed;
        double ang;
    };

    void zero(Pose& pose);
}