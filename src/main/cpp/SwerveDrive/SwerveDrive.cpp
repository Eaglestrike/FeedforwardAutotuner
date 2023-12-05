#include "SwerveDrive\SwerveDrive.h"

#include "Util\GeometryHelper.h"
#include "Util\ShuffleboardItems\SI_Point.hpp"

using namespace GeometryHelper;

/**
 * Constructor
 * 
 * @param name name
 * @param enabled enabled
 * @param shuffleboard drivebase shuffleboard
 * @param mShuffleboard module shuffleboard
*/
SwerveDrive::SwerveDrive(std::string name, bool enabled, bool shuffleboard, bool mShuffleboard):
    Mechanism(name, enabled, shuffleboard)
{
    for(int i = 0; i < SwerveConstants::NUMSWERVE; i++){
        modules_[i] = new SwerveModule(SwerveConstants::MODULES[i], enabled, mShuffleboard);
    }
}

void SwerveDrive::CoreInit(){
    for(SwerveModule* module : modules_){
        module->Init();
    }
}

void SwerveDrive::reset(){
    zero();
    SwervePose::zero(targetPose_);
}

void SwerveDrive::zero(){
    SwervePose::zero(currentPose_);
    if(navx_){
        navx_->ZeroYaw();
    }
}

void SwerveDrive::CorePeriodic(){
    for(SwerveModule* module : modules_){
        module->Periodic();
    }
    updatePose();
}

void SwerveDrive::updatePose(){
    double time = frc::Timer::GetFPGATimestamp().value();
    double dt = time - lastUpdate_;

    if(navx_){
        double newAng = toRad(navx_->GetYaw());
        double newAngVel = (newAng - currentPose_.ang) / dt;
        double newAngAccel = (newAngVel - currentPose_.angVel) / dt;
        currentPose_.ang = newAng;
        currentPose_.angVel = newAngVel;
        currentPose_.angAccel = newAngAccel;
    }

    //Average the velocities
    Vector velocity{0.0, 0.0};
    for(SwerveModule* module : modules_){
        velocity += module->getVel();
    }
    velocity /= SwerveConstants::NUMSWERVE;

    //Field-Orient
    velocity.rotateThis(-currentPose_.ang);

    currentPose_.pos += velocity * dt;
    currentPose_.accel = (velocity - currentPose_.vel) / dt;
    currentPose_.vel = velocity;

    lastUpdate_ = frc::Timer::GetFPGATimestamp().value();
}

void SwerveDrive::CoreTeleopInit(){
    for(SwerveModule* module : modules_){
        module->TeleopInit(); //Motors to brake to not be scooted by enemy
    }
}

void SwerveDrive::CoreTeleopPeriodic(){
    drive(); //Set module targets
    for(SwerveModule* module : modules_){
        module->TeleopPeriodic(); //Drive motors and stuff
    }
}

/***
 * Based off the targetPose_, it will assign a target position for the modules
 * i.e. tell what the modules to do
*/
void SwerveDrive::drive(){
    double angVel = targetPose_.angVel;
    if(angVel == 0.0){
        if(!isHoldingAng_){
            isHoldingAng_ = true;
            holdingAng_ = currentPose_.ang;
        }
        double angleError = GeometryHelper::getAngDiff(currentPose_.ang, holdingAng_);
        if(abs(angleError) < 0.3){
            angVel += 3.5 * angleError;
        }
        else{
            isHoldingAng_ = false;
        }
    }
    else{
        isHoldingAng_ = false;
    }
    for(SwerveModule* module : modules_){
        Vector angVelVec = module->getPos() - pivot_; //Get vector from pivot to module
        angVelVec.rotateCounterclockwise90This(); //Set to vector tangent to the path of rotation

        //Adds tangential velocity, which is just the target tangential velocity (rotated by the robot's pose)
        //Adds the rotational velocity, which is the angVelVec times the angular velocity: v = r*w
        Vector moduleVec = targetPose_.vel.rotate(-currentPose_.ang) + (angVelVec*angVel);

        //std::cout<<"Target for "<<module->getName()<<": "<<moduleVec.toString()<<std::endl;
        double speed = moduleVec.originDist();
        if(speed != 0){//Atan2 of 0,0 is very funky
            module->setTarget(SwervePose::ModulePose{speed, moduleVec.getAng()});
        }
        else{
            module->setTarget(SwervePose::ModulePose{0.0, 0.0}); //Maybe lock wheels? maybe after some time
        }
    }
}

void SwerveDrive::CoreDisabledInit(){
    for(SwerveModule* module : modules_){
        module->DisabledInit(); //Sets the motor mode to coast (so robot can be scooted by ppl)
    }
}

void SwerveDrive::CoreDisabledPeriodic(){
    for(SwerveModule* module : modules_){
        module->DisabledPeriodic(); //Tell robot to actively do nothing
    }
}

/***
 * Enables shuffleboard prints
 * @param edit can edit target values
 * @param modules enable module prints
*/
void SwerveDrive::CoreShuffleboardInit(){
    shuff_.add("Current Ang", &currentPose_.ang);
    shuff_.add("Current Ang Vel", &currentPose_.angVel);
    shuff_.add("Current Ang Acc", &currentPose_.angAccel);
    shuff_.add("Current Pos", &currentPose_.pos);
    shuff_.add("Current Vel", &currentPose_.vel);
    shuff_.add("Current Acc", &currentPose_.accel);

    shuff_.add("Target Ang Vel", &targetPose_.angVel);
    shuff_.add("Target Vel", &targetPose_.vel);

    shuff_.add("Holding Ang", &holdingAng_);
    shuff_.add("Is Holding Ang", &isHoldingAng_);
}

void SwerveDrive::CoreShuffleboardPeriodic(){
    
}

void SwerveDrive::SetTarget(Vector v, double angV, bool volts){
    targetPose_.vel = v;
    targetPose_.angVel = angV;
    volts_ = volts;
}

SwervePose::Pose SwerveDrive::getCurrPose(){
    return currentPose_;
}