#include "SwerveDrive/SwerveDrive.h"

#include "Geometry/GeometryHelper.h"
using namespace GeometryHelper;

SwerveDrive::SwerveDrive(std::string name):
    name_(name),
    ShuffData_(name)
{
    for(int i = 0; i < SwerveConstants::NUMSWERVE; i++){
        modules_[i] = new SwerveModule(SwerveConstants::MODULES[i]);
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

void SwerveDrive::Periodic(){
    for(SwerveModule* module : modules_){
        module->Periodic();
    }
    updatePose();

    ShuffData_.update();
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
}

void SwerveDrive::TeleopInit(){
    for(SwerveModule* module : modules_){
        module->TeleopInit(); //Motors to brake to not be scooted by enemy
    }
}

void SwerveDrive::TeleopPeriodic(){
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
    for(SwerveModule* module : modules_){
        Vector angVelVec = module->getPos() - pivot_; //Get vector from pivot to module
        angVelVec.rotateCounterclockwise90This(); //Set to vector tangent to the path of rotation

        //Adds tangential velocity, which is just the target tangential velocity (rotated by the robot's pose)
        //Adds the rotational velocity, which is the angVelVec times the angular velocity: v = r*w
        Vector moduleVec = targetPose_.vel.rotate(-currentPose_.ang) + (angVelVec*targetPose_.angVel);

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

void SwerveDrive::DisabledInit(){
    for(SwerveModule* module : modules_){
        module->DisabledInit(); //Sets the motor mode to coast (so robot can be scooted by ppl)
    }
}

void SwerveDrive::DisabledPeriodic(){
    for(SwerveModule* module : modules_){
        module->DisabledPeriodic(); //Tell robot to actively do nothing
    }
}

/***
 * Enables shuffleboard prints
 * @param edit can edit target values
 * @param modules enable module prints
*/
void SwerveDrive::enableShuffleboard(bool edit, bool modules){
    if(modules){
        for(SwerveModule* module : modules_){
            module->enableShuffleboard(edit);
        }
    }
    if(ShuffData_.isInitialized()){
        ShuffData_.enable(edit);
    }
    ShuffData_.Initialize(edit);
    ShuffData_.add("Current Ang", &currentPose_.ang);
    ShuffData_.add("Current Ang Vel", &currentPose_.angVel);
    ShuffData_.add("Current Ang Acc", &currentPose_.angAccel);
    ShuffData_.add("Current Pos", &currentPose_.pos);
    ShuffData_.add("Current Vel", &currentPose_.vel);
    ShuffData_.add("Current Acc", &currentPose_.accel);

    ShuffData_.add("Target Ang Vel", &targetPose_.angVel);
    ShuffData_.add("Target Vel", &targetPose_.vel);
}

void SwerveDrive::disableSuffleboard(){
    ShuffData_.disable();
}

void SwerveDrive::SetTarget(Vector v, double angV, bool volts){
    targetPose_.vel = v;
    targetPose_.angVel = angV;
    volts_ = volts;
}

SwervePose::Pose SwerveDrive::getCurrPose(){
    return currentPose_;
}