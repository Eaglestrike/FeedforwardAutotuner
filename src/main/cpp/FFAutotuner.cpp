#include "FFAutotuner.h"

#include <random>

#include "MathUtil.h"

using namespace Poses;
using namespace MathUtil;

FFAutotuner::FFAutotuner(FFType type, double min, double max):
    lastTime(frc::Timer::GetFPGATimestamp().value()),
    ffType_(type),
    profile_(0.0, 0.0),
    bounds_({.min = min, .max = max}),
    ffTesting_({.kv = 0.0, .ka = 0.0, .ks = 0.0, .kg = 0.0})
{
    resetError();
}

double FFAutotuner::getVoltage(Pose1D currPose){
    double dt = frc::Timer::GetFPGATimestamp().value() - lastTime;
    if(profile_.isFinished()){
        resetProfile(currPose);
    }

    Pose1D expectedPose = profile_.currentPose();
    Pose1D error = (expectedPose - currPose)*dt;

    double maxVel = profile_.getMaxVel();
    double maxAcc = profile_.getMaxAcc();

    double velComp = expectedPose.vel / maxVel;
    double accComp = expectedPose.acc / maxAcc;
    double stcComp = sign(expectedPose.vel);
    double grvComp;
    switch(ffType_){
        case SIMPLE:
            grvComp = 0.0;
            break;
        case ARM:
            grvComp = std::cos(expectedPose.pos);
            break;
        case ELEVATOR:
            grvComp = 1.0;
            break;
        default:
            grvComp = 0.0;
    }

    double absVelComp = std::abs(velComp);
    double absAccComp = std::abs(accComp);
    double absStcComp = std::abs(stcComp);
    double absGrvComp = std::abs(grvComp);

    double totAbsComp = absVelComp + absAccComp + absStcComp + absGrvComp;
    
    if(totAbsComp != 0){
        error_.gainError.kv += error.vel * velComp/totAbsComp;
        error_.gainError.ka += error.vel * accComp/totAbsComp;
        error_.gainError.ks += error.vel * stcComp/totAbsComp;
        error_.gainError.kg += error.vel * grvComp/totAbsComp;
    }
    error_.totalError += error;
    error_.absTotalError += abs(error);

    lastTime = frc::Timer::GetFPGATimestamp().value();
    switch(ffType_){
        case SIMPLE:
            return stcComp*ffTesting_.ks + expectedPose.vel*ffTesting_.kv + expectedPose.acc*ffTesting_.ka;
        case ARM:
            return stcComp*ffTesting_.ks + expectedPose.vel*ffTesting_.kv + expectedPose.acc*ffTesting_.ka + grvComp*ffTesting_.kg;
        case ELEVATOR:
            return stcComp*ffTesting_.ks + expectedPose.vel*ffTesting_.kv + expectedPose.acc*ffTesting_.ka + ffTesting_.kg;
        default:
            return 0.0;
    }
}

void FFAutotuner::resetProfile(Pose1D currPose){
    double duration = profile_.getDuration();
    if(duration != 0.0){
        double avgAbsError = error_.absTotalError.vel/duration;

        double dKv = error_.gainError.ks/duration * s_;
        double dKa = error_.gainError.ka/duration * s_;
        double dKs = error_.gainError.ks/duration * s_;
        double dKg = error_.gainError.kg/duration * s_;

        ffTesting_.kv += dKv;
        ffTesting_.ka += dKa;
        ffTesting_.ks += dKs;
        ffTesting_.kg += dKg;
    }
    resetError();

    double maxDist = bounds_.max - bounds_.min;

    double maxVel, maxAcc;
    if(duration != 0.0){
        if((error_.absTotalError.pos/duration < 0.1) && (error_.absTotalError.vel/duration < 0.1)){
            expectTime /= 1.5;
        }
    }
    double maxVel = maxDist/expectTime;
    double maxAcc = maxVel;
    profile_.setMaxVel(maxVel);
    profile_.setMaxAcc(maxAcc);

    std::uniform_real_distribution<double> unif(bounds_.min, bounds_.max);
    std::default_random_engine re;
    double nextTarget = unif(re);
    profile_.setTarget(currPose, {.pos = nextTarget, .vel = 0.0, .acc = 0.0});
}

void FFAutotuner::setMin(double min){
    bounds_.min = min;
}

void FFAutotuner::setMax(double max){
    bounds_.max = max;
}

void FFAutotuner::resetError(){
    error_ = {
        .gainError = {.kv = 0.0, .ka = 0.0, .ks = 0.0, .kg = 0.0},
        .totalError = {.pos = 0.0, .vel = 0.0, .acc = 0.0},
        .absTotalError = {.pos = 0.0, .vel = 0.0, .acc = 0.0}
    };
}