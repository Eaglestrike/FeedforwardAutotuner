#include "SwerveDrive\SwerveModule.h"

#include "SwerveDrive\ShuffleboardItems\SI_SwerveModule.hpp"

#include "Util\GeometryHelper.h"
using namespace GeometryHelper;

SwerveModule::SwerveModule(SwerveConstants::SwerveStruct swerveMod, bool enabled, bool shuffleboard):
    Mechanism(swerveMod.name, enabled, shuffleboard),
    driveMotor_(swerveMod.driveID, SwerveConstants::canBus), driveVolts_(0.0),
    turnMotor_(swerveMod.turnID, SwerveConstants::canBus), turnVolts_(0.0),
    cancoder_(swerveMod.encoderID, SwerveConstants::canBus),
    encoderOffset_(swerveMod.encoderOffset), encoderInverted_(swerveMod.encoderInverted),
    turnPID_(swerveMod.turnPID),
    pos_(swerveMod.pos)
{
    driveMotor_.SetNeutralMode(NeutralMode::Coast);
    turnMotor_.SetNeutralMode(NeutralMode::Coast);
}

void SwerveModule::CorePeriodic(){
    //Calc velocity
    double wheelAng = toRad((encoderInverted_?-1.0:1.0)*cancoder_.GetAbsolutePosition() + encoderOffset_);
    double driveAngVel = driveMotor_.GetSelectedSensorVelocity() * 10.0 / SwerveConstants::TICKS_PER_RADIAN  * SwerveConstants::WHEEL_RADIUS;
    double wheelVel = driveAngVel * SwerveConstants::WHEEL_RADIUS;
    currPose_.ang = wheelAng;
    currPose_.speed = wheelVel;
    vel_ = Vector(cos(wheelAng) * wheelVel , sin(wheelAng) * wheelVel);
}

void SwerveModule::CoreTeleopInit(){
    driveMotor_.SetNeutralMode(NeutralMode::Brake);
    turnMotor_.SetNeutralMode(NeutralMode::Brake);
};

void SwerveModule::CoreTeleopPeriodic(){
    double driveTarg = std::clamp(targetPose_.speed, -maxDriveVolts_, maxDriveVolts_);
    if(inverted_){
        driveTarg = -driveTarg;
    }
    driveVolts_ = units::volt_t(driveTarg);
    driveMotor_.SetVoltage(driveVolts_);

    double ang = currPose_.ang;
    if(inverted_){
        ang += M_PI;
    }
    double turnDiff = getAngDiff(targetPose_.ang, ang);
    if(turnDiff > M_PI/2.0){
        inverted_ = !inverted_;
        turnDiff = -M_PI + turnDiff;
    }
    else if(turnDiff < -M_PI/2.0){
        inverted_ = !inverted_;
        turnDiff = M_PI + turnDiff;
    }
    double turnTarg = turnPID_.Calculate(turnDiff);
    turnTarg = std::clamp(turnTarg, -maxTurnVolts_, maxTurnVolts_);
    turnVolts_ = units::volt_t(turnTarg);
    turnMotor_.SetVoltage(turnVolts_);
}

void SwerveModule::CoreDisabledInit(){
    driveMotor_.SetNeutralMode(NeutralMode::Coast);
    turnMotor_.SetNeutralMode(NeutralMode::Coast);
}

void SwerveModule::CoreDisabledPeriodic(){
    driveMotor_.SetVoltage(units::volt_t{0.0});
    turnMotor_.SetVoltage(units::volt_t{0.0});
}

void SwerveModule::zero(){
    turnMotor_.SetSelectedSensorPosition(0.0);
}

void SwerveModule::setTarget(SwervePose::ModulePose pose, bool volts){
    targetPose_ = pose;
    volts_ = volts;
}

void SwerveModule::CoreShuffleboardInit(){
    shuff_.add("Drive Volts", &driveVolts_);
    shuff_.add("Turn Volts", &turnVolts_);
    shuff_.add("Drive Max Volts", &maxDriveVolts_);
    shuff_.add("Turn Max Volts", &maxTurnVolts_);
    shuff_.add("Turn PID", &turnPID_, true);
    shuff_.add("Target Pose", &targetPose_);
    shuff_.add("Volts", &volts_);
    shuff_.add("Current Pose", &currPose_);
    shuff_.add("Inverted", &inverted_);
}

void SwerveModule::CoreShuffleboardPeriodic(){
    
}

std::string SwerveModule::getName(){
    return name_;
}

Point SwerveModule::getPos(){
    return pos_;
}

Vector SwerveModule::getVel(){
    return vel_;
}