#include "FFAutotuner/FFAutotuner.h"

#include <random>
#include "Util/Utils.h"

using namespace Poses;

FFAutotuner::FFAutotuner(std::string name, FFType type, double min, double max, double targTime, double testTime):
    name_(name),
    ffType_(type),
    state_(IDLE),
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
    ShuffData_.add("ks", &ffTesting_.ks, {2,1,0,3}, true);
    ShuffData_.add("kv", &ffTesting_.kv, {2,1,2,3}, true);
    ShuffData_.add("ka", &ffTesting_.ka, {2,1,4,3}, true);
    ShuffData_.add("kg", &ffTesting_.kg, {2,1,6,3}, true);

    ShuffData_.add("Target Time", &targTime_, {2,1,2,0}, true);
    ShuffData_.add("Testing Time", &testTime_, {2,1,2,1}, true);
    
    ShuffData_.add("eks", &error_.gainError.ks, {2,1,0,4});
    ShuffData_.add("ekv", &error_.gainError.kv, {2,1,2,4});
    ShuffData_.add("eka", &error_.gainError.ka, {2,1,4,4});
    ShuffData_.add("ekg", &error_.gainError.kg, {2,1,6,4});

    ShuffData_.add("total abs pos error", &error_.absTotalError.pos, {2,1,9,2});
    ShuffData_.add("total abs vel error", &error_.absTotalError.vel, {2,1,9,3});

    ShuffData_.add("min", &bounds_.min, {1,1,5,1}, true);
    ShuffData_.add("max", &bounds_.max, {1,1,6,1}, true);
}

void FFAutotuner::setPose(Pose1D currPose){
    if(state_ == IDLE){
        return;
    }
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
    Pose1D error = (expectedPose - currPose_)*dt; //Error*dt

    double maxVel = profile_.getMaxVel();
    double maxAcc = profile_.getMaxAcc();
    
    double velComp = expectedPose.vel / maxVel;
    double stcComp = Utils::sign(expectedPose.vel) - velComp; //static component will be inverted trapezoid
    double accComp = expectedPose.acc / maxAcc;
    double grvComp;
    switch(ffType_){
        case SIMPLE:   grvComp = 0.0;                        break;
        case ARM:      grvComp = std::cos(expectedPose.pos); break;
        case ELEVATOR: grvComp = 1.0;                        break;
        default:       grvComp = 0.0;
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
}

double FFAutotuner::getVoltage(){
    if(state_ == IDLE){
        return 0.0;
    }
    Pose1D expectedPose = profile_.currentPose();
    switch(ffType_){
        case SIMPLE:
            return Utils::sign(expectedPose.vel)*ffTesting_.ks + expectedPose.vel*ffTesting_.kv + expectedPose.acc*ffTesting_.ka;
        case ARM:
            return Utils::sign(expectedPose.vel)*ffTesting_.ks + expectedPose.vel*ffTesting_.kv + expectedPose.acc*ffTesting_.ka + std::cos(expectedPose.pos)*ffTesting_.kg;
        case ELEVATOR:
            return Utils::sign(expectedPose.vel)*ffTesting_.ks + expectedPose.vel*ffTesting_.kv + expectedPose.acc*ffTesting_.ka + ffTesting_.kg;
        default:
            return 0.0;
    }
}

void FFAutotuner::resetProfile(bool center){
    //Update feedforwards gains by average error for each term
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
    //Calculate new profile, making it faster if it is reaching the target
    if(duration != 0.0){
        double averagePosError = error_.absTotalError.pos/duration;
        double averageVelError = error_.absTotalError.vel/duration;
        double precision = precision_;
        if(precision == 0){
            precision = 100.0;
        }
        if((averagePosError < maxDist/precision_) && //Check if round was accurate enough
           (averageVelError < maxDist/precision_)){
            testTime_ = (testTime_ - targTime_)*0.8 + targTime_; //0.8 is decay rate
        }

        double prevPosError = pastPosErrors_.back();
        if(Utils::sign(prevPosError) != Utils::sign(averagePosError)){ //Is oscillating
            s_ *= 0.75; //Scale down step size
        }
        else if(std::abs(averagePosError - prevPosError) < std::abs(prevPosError * 0.001)){ // Is not approaching fast enough
            s_ *= 1.25; //Scale up 
        }

        pastPosErrors_.push_back(averagePosError);
        pastPosErrors_.push_back(averageVelError);
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
        nextTarget = bounds_.min + (maxDist * (random() % 100000L) / 100000.0); //Random next target
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

FFAutotuner::FFConfig FFAutotuner::getFeedforward(){
    return ffTesting_;
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