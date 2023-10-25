#pragma once

#include "Geometry/Point.h"

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

    static void zero(Pose& pose){
        pose.pos = Vector(0.0,0.0);
        pose.vel = Vector(0.0,0.0);
        pose.accel = Vector(0.0,0.0);
        pose.ang = 0.0;
        pose.angVel = 0.0;
        pose.angAccel = 0.0;
    }
}