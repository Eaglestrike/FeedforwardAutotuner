#include "SwerveDrive/SwervePose.h"

void SwervePose::zero(SwervePose::Pose& pose){
    pose.pos = Vector(0.0,0.0);
    pose.vel = Vector(0.0,0.0);
    pose.accel = Vector(0.0,0.0);
    pose.ang = 0.0;
    pose.angVel = 0.0;
    pose.angAccel = 0.0;
}