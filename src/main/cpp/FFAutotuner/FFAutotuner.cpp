#include "FFAutotuner/FFAutotuner.h"

#include <random>
#include "Util/Utils.h"

using namespace Poses;

FFAutotuner::FFAutotuner(std::string name, FFType type, double min, double max, double targTime, double testTime):
    name_(name),
    ffType_(type),
    state_(TUNING),
    currPose_({.pos = 0.0, .vel = 0.0, .acc = 0.0}),
    lastTime_(frc::Timer::GetFPGATimestamp().value()),
    profile_(0.0, 0.0),
    targTime_(targTime), testTime_(testTime < targTime? targTime*3.0 : testTime),
    bounds_({.min = min, .max = max}),
    ffTesting_({.ks = 0.0, .kv = 0.0, .ka = 0.0, .kg = 0.0}),
    ShuffData_(name)
{
    resetError();
    
    ShuffData_.add("s", &s_, {2,2,0,0}, true);
    ShuffData_.add("ks", &ffTesting_.ks, {2,1,0,3});
    ShuffData_.add("kv", &ffTesting_.kv, {2,1,2,3});
    ShuffData_.add("ka", &ffTesting_.ka, {2,1,4,3});
    ShuffData_.add("kg", &ffTesting_.kg, {2,1,6,3});

    ShuffData_.add("Target Time", &targTime_, {2,1,2,0}, true);
    ShuffData_.add("Testing Time", &testTime_, {2,1,2,1});
    
    ShuffData_.add("eks", &error_.gainError.ks, {2,1,0,4});
    ShuffData_.add("ekv", &error_.gainError.kv, {2,1,2,4});
    ShuffData_.add("eka", &error_.gainError.ka, {2,1,4,4});
    ShuffData_.add("ekg", &error_.gainError.kg, {2,1,6,4});

    ShuffData_.add("total abs pos error", &error_.absTotalError.pos, {2,1,9,2});
    ShuffData_.add("total abs vel error", &error_.absTotalError.vel, {2,1,9,3});

    ShuffData_.add("min", &bounds_.min, {1,1,5,1});
    ShuffData_.add("max", &bounds_.max, {1,1,6,1});
    
    //ShuffData_.add("State", &state_);
}

double FFAutotuner::getVoltage(Pose1D currPose){
    currPose_ = currPose;
    double dt = frc::Timer::GetFPGATimestamp().value() - lastTime_;

    if(currPose_.pos > bounds_.max && state_ != RECENTER_FROM_MAX){ // Out of bounds
        state_ = RECENTER_FROM_MAX;
        resetProfile(true);
        std::cout<<name_ <<" Recentering"<<std::endl;
    }

    if(currPose_.pos < bounds_.min && state_ != RECENTER_FROM_MIN){ // Out of bounds
        state_ = RECENTER_FROM_MIN;
        resetProfile(true);
        std::cout<<name_ <<" Recentering"<<std::endl;
    }

    if(profile_.isFinished() || dt > 0.1){
        state_ = TUNING;
        resetProfile(false);
    }

    Pose1D expectedPose = profile_.currentPose();
    Pose1D error = (expectedPose - currPose_)*dt;

    double maxVel = profile_.getMaxVel();
    double maxAcc = profile_.getMaxAcc();
    
    double stcComp = Utils::sign(expectedPose.vel);
    double velComp = expectedPose.vel / maxVel;
    double accComp = expectedPose.acc / maxAcc;
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

    
    double absStcComp = std::abs(stcComp);
    double absVelComp = std::abs(velComp);
    double absAccComp = std::abs(accComp);
    double absGrvComp = std::abs(grvComp);

    double totAbsComp = absVelComp + absAccComp + absStcComp + absGrvComp;
    
    if(totAbsComp != 0){
        error_.gainError.ks += error.vel * stcComp/totAbsComp;
        error_.gainError.kv += error.vel * velComp/totAbsComp;
        error_.gainError.ka += error.vel * accComp/totAbsComp;
        error_.gainError.kg += error.vel * grvComp/totAbsComp;
        error_.totalError += error;
        error_.absTotalError += abs(error);
    }

    lastTime_ = frc::Timer::GetFPGATimestamp().value();
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

void FFAutotuner::resetProfile(bool center){
    double duration = profile_.getDuration();
    if(duration != 0.0){
        double dKs = error_.gainError.ks/duration * s_;
        double dKv = error_.gainError.kv/duration * s_;
        double dKa = error_.gainError.ka/duration * s_;
        double dKg = error_.gainError.kg/duration * s_;

        ffTesting_.ks += dKs;
        ffTesting_.kv += dKv;
        ffTesting_.ka += dKa;
        ffTesting_.kg += dKg;
    }

    double maxDist = bounds_.max - bounds_.min;

    if(duration != 0.0){
        if((error_.absTotalError.pos/duration < maxDist/100.0) && (error_.absTotalError.vel/duration < maxDist/100.0)){
            testTime_ = (testTime_ - targTime_)*0.8 + targTime_;
        }
    }
    double maxVel = maxDist/testTime_;
    double maxAcc = maxVel;
    profile_.setMaxVel(maxVel);
    profile_.setMaxAcc(maxAcc);

    double nextTarget;
    if(center){
        nextTarget = (bounds_.min + bounds_.max)/2.0;
    }
    else{
        nextTarget = bounds_.min + (maxDist * (random() % 100000L) / 100000.0);
    }
    profile_.setTarget(currPose_, {.pos = nextTarget, .vel = 0.0, .acc = 0.0});
    std::cout<<"Set " << name_ << " target: " << nextTarget <<std::endl;

    resetError();
}

void FFAutotuner::zeroBounds(double val){
    bounds_.min = val;
    bounds_.max = val;
}

void FFAutotuner::expandBounds(double val){
    if(val > bounds_.max){
        bounds_.max = val;
    }
    if(val < bounds_.min){
        bounds_.min = val;
    }
}

void FFAutotuner::setMin(double min){
    bounds_.min = min;
}

void FFAutotuner::setMax(double max){
    bounds_.max = max;
}

void FFAutotuner::setFeedforward(FFConfig config){
    ffTesting_ = config;
}

void FFAutotuner::ShuffleboardUpdate(){
    ShuffData_.update(true);
}

void FFAutotuner::resetError(){
    error_ = {
        .gainError = {.ks = 0.0, .kv = 0.0, .ka = 0.0, .kg = 0.0},
        .totalError = {.pos = 0.0, .vel = 0.0, .acc = 0.0},
        .absTotalError = {.pos = 0.0, .vel = 0.0, .acc = 0.0}
    };
}