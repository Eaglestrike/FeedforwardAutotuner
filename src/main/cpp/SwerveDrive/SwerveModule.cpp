#include "SwerveDrive\SwerveModule.h"

#include "SwerveDrive\ShuffleboardItems\SI_SwerveModule.h"

#include "Util\GeometryHelper.h"
using namespace GeometryHelper;

SwerveModule::SwerveModule(SwerveConstants::SwerveStruct swerveMod):
    name_(swerveMod.name),
    driveMotor_(swerveMod.driveID, SwerveConstants::canBus), driveVolts_(0.0),
    turnMotor_(swerveMod.turnID, SwerveConstants::canBus), turnVolts_(0.0),
    cancoder_(swerveMod.encoderID, SwerveConstants::canBus),
    encoderOffset_(swerveMod.encoderOffset), encoderInverted_(swerveMod.encoderInverted),
    turnPID_(swerveMod.turnPID),
    pos_(swerveMod.pos),
    ShuffData_(swerveMod.name)
{
    driveMotor_.SetNeutralMode(NeutralMode::Coast);
    turnMotor_.SetNeutralMode(NeutralMode::Coast);
}

void SwerveModule::Periodic(){
    //Calc velocity
    double wheelAng = toRad((encoderInverted_?-1.0:1.0)*cancoder_.GetAbsolutePosition() + encoderOffset_);
    double driveAngVel = driveMotor_.GetSelectedSensorVelocity() * 10.0 / SwerveConstants::TICKS_PER_RADIAN  * SwerveConstants::WHEEL_RADIUS;
    double wheelVel = driveAngVel * SwerveConstants::WHEEL_RADIUS;
    currPose_.ang = wheelAng;
    currPose_.speed = wheelVel;
    vel_ = Vector(cos(wheelAng) * wheelVel , sin(wheelAng) * wheelVel);

    ShuffData_.update();
}

void SwerveModule::TeleopInit(){
    driveMotor_.SetNeutralMode(NeutralMode::Brake);
    turnMotor_.SetNeutralMode(NeutralMode::Brake);
};

void SwerveModule::TeleopPeriodic(){
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

void SwerveModule::DisabledInit(){
    driveMotor_.SetNeutralMode(NeutralMode::Coast);
    turnMotor_.SetNeutralMode(NeutralMode::Coast);
}

void SwerveModule::DisabledPeriodic(){
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

void SwerveModule::enableShuffleboard(bool edit){
    if(ShuffData_.isInitialized()){
        ShuffData_.enable(edit);
    }
    ShuffData_.Initialize(edit);
    ShuffData_.add("Drive Volts", &driveVolts_);
    ShuffData_.add("Turn Volts", &turnVolts_);
    ShuffData_.add("Drive Max Volts", &maxDriveVolts_);
    ShuffData_.add("Turn Max Volts", &maxTurnVolts_);
    ShuffData_.add("Turn PID", &turnPID_, edit = true);
    ShuffData_.add("Target Pose", &targetPose_);
    ShuffData_.add("Volts", &volts_);
    ShuffData_.add("Current Pose", &currPose_);
    ShuffData_.add("Inverted", &inverted_);
}

void SwerveModule::disableSuffleboard(){
    ShuffData_.disable();
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