//
// Created by Vir Shah on 6/14/23.
//
#include "Elevator/BaseElevator/Elevator.h"

#include <stdio.h>
#include <iostream>
#include <cmath>

#include <Util/Utils.h>

#include <frc/smartdashboard/SmartDashboard.h>

/**
 * @brief Construct a new Elevator:: Elevator object
 */
Elevator::Elevator(bool enabled, bool shuffleboard):
    Mechanism("elevator", enabled, shuffleboard),
    left_(ElevatorConstants::LEFT_MOTOR_ID, "rio"), right_(ElevatorConstants::RIGHT_MOTOR_ID, "rio"),
    limit_switch_(ElevatorConstants::LIMIT_SWITCH_ID),
    current_state_(HOLDING_POS),
    current_target_(STOWED),
    feedforward_(ElevatorConstants::FEEDFORWARD_CONSTANTS, shuffleboard),
    max_volts_(ElevatorConstants::MAX_VOLTS)
{
    current_pose_ = {0.0, 0.0, 0.0};

    left_.SetInverted(true);
    right_.Follow(left_, FollowerType::FollowerType_PercentOutput);

    left_.SetNeutralMode(NeutralMode::Brake);
    right_.SetNeutralMode(NeutralMode::Brake);

    feedforward_.reset();

    elevatorTuner_.setMin(0.05);
    elevatorTuner_.setMax(ElevatorConstants::MAX_EXTENSION - 0.05);
    elevatorTuner_.Start();
};

void Elevator::StartTuning(){
    current_state_ = TUNING;
}

void Elevator::UseTuningValues(bool tuningVals){
    if(tuningVals){
        FFAutotuner::FFConfig ff = elevatorTuner_.getFeedforward();
        feedforward_.setKs(ff.ks);
        feedforward_.setKg(ff.kg);
        feedforward_.setKv(ff.kv);
        feedforward_.setKa(ff.ka);
    }
    else{
        feedforward_.setKa(ElevatorConstants::FEEDFORWARD_CONSTANTS.ka);
        feedforward_.setKv(ElevatorConstants::FEEDFORWARD_CONSTANTS.kv);
        feedforward_.setKs(ElevatorConstants::FEEDFORWARD_CONSTANTS.ks);
        feedforward_.setKg(ElevatorConstants::FEEDFORWARD_CONSTANTS.kg);
    }
}

void Elevator::CorePeriodic(){
    current_pose_.pos = getElevatorHeight();
    // dividing by 10 to convert from 100 milliseconds to seconds.
    current_pose_.vel = talonUnitsToMeters(left_.GetSelectedSensorVelocity()) * 10.0;

    elevatorTuner_.ShuffleboardUpdate();
}

/**
 * Called every periodic cycle, handles all movement
 * Acts on the difference between current position and next position
 */
void Elevator::CoreTeleopPeriodic() {
    double motor_output;

    switch(current_state_){
        case MANUAL:
            motor_output = debug_manual_volts_;
            break;
        case HOLDING_POS:
            if (current_target_ == STOWED){
                motor_output = 0.0;
                break;
            }
            [[fallthrough]];
        case MOVING:
            motor_output = feedforward_.periodic(current_pose_);
            if (feedforward_.atSetpoint(current_pose_, 0.04, 0.05)){
                current_state_ = HOLDING_POS;
            }
            if (feedforward_.isFinished() && current_target_ == STOWED){
                current_state_ = HOLDING_POS;
            }
            break;
        case TUNING:
            elevatorTuner_.setPose(current_pose_);
            motor_output = elevatorTuner_.getVoltage();
            break;
        default:
            motor_output = 0.0;
    }

    if(!limit_switch_.Get() && current_state_ != MOVING){
        if (current_pose_.pos < 0.0 || current_pose_.pos > 0.1) {
            zero_motors();
        }
        if(motor_output < feedforward_.getKg() + 0.001){
            motor_output = 0.0;
        }
    }

    if(current_pose_.pos > ElevatorConstants::MAX_EXTENSION - 0.001){
        if(motor_output > feedforward_.getKg() + 0.001){
            motor_output = feedforward_.getKg();
        }
    }

    if (shuff_.isEnabled()) {
        shuff_.PutNumber("El Motor output", motor_output);
        shuff_.PutNumber("El Motor output true", std::clamp(motor_output, -max_volts_, max_volts_));
        shuff_.PutNumber("El max volts", max_volts_);
    }

    left_.SetVoltage(units::volt_t{std::clamp(motor_output, -max_volts_, max_volts_)});
    //right_.SetVoltage(units::volt_t{std::clamp(motor_output, -max_volts_, max_volts_)});
}

// debug getters
Elevator::ElevatorState Elevator::getState() {
    return current_state_;
}

/**
 * @brief Returns the elevator height in meters, calculated per the left motor's position.
 * 
 * @return double height (in meters)
 */
double Elevator::getElevatorHeight() {
    return talonUnitsToMeters(left_.GetSelectedSensorPosition()) + elevatorOffset;
}

/**
 * @brief Given a position, the elevator will move to that position
 * 
 * @param newPos 
 */
void Elevator::ExtendToCustomPos(double newPos) {
    current_state_ = ElevatorState::MOVING;
    current_target_ = ElevatorTarget::CUSTOM;
    feedforward_.setTotalDistance(newPos, getElevatorHeight());
}

ElevatorConstants::ElevatorTarget Elevator::getTarget(){
    return current_target_;
}

void Elevator::Stow(){
    current_state_ = ElevatorState::MOVING;
    current_target_ = ElevatorTarget::STOWED;
    feedforward_.setTotalDistance(ElevatorConstants::TARGET_TO_HEIGHT.at(current_target_), getElevatorHeight());
}

void Elevator::ExtendLow() {
    current_state_ = ElevatorState::MOVING;
    current_target_ = ElevatorTarget::LOW;
    feedforward_.setTotalDistance(ElevatorConstants::TARGET_TO_HEIGHT.at(current_target_), getElevatorHeight());
}

void Elevator::ExtendMid() {
    current_state_ = ElevatorState::MOVING;
    current_target_ = ElevatorTarget::MID;
    feedforward_.setTotalDistance(ElevatorConstants::TARGET_TO_HEIGHT.at(current_target_), getElevatorHeight());
}

void Elevator::ExtendHigh() {
    current_state_ = ElevatorState::MOVING;
    current_target_ = ElevatorTarget::HIGH;
    feedforward_.setTotalDistance(ElevatorConstants::TARGET_TO_HEIGHT.at(current_target_), getElevatorHeight());
}

void Elevator::HoldPosition(){
    if(current_state_== Elevator::MOVING){
        return;
    }
    if(current_state_== Elevator::HOLDING_POS){
        return;
    }
    current_state_ = ElevatorState::HOLDING_POS;
    double height = getElevatorHeight();
    if(Utils::NearZero(height)){
        current_target_ = ElevatorTarget::STOWED;
    }
    else{
        current_target_ = ElevatorTarget::CUSTOM;
    }
    feedforward_.setTotalDistance(height, height);
}

double Elevator::GetPos(){
    return current_pose_.pos;
}

/**
 * @brief Runs a fraction of the max voltage to the elevator
 * 
 * @param range the range of the xbox joystick axis
 * Note: it is assumed that the range will be from -1 to 1.
 */
void Elevator::setManualVolts(double range) {
    current_state_ = ElevatorState::MANUAL;
    debug_manual_volts_ = range * max_volts_;
}

// util methods

/**
 * @brief Resets left and right motor rotation 
 * 
 */
void Elevator::zero_motors() {
    elevatorOffset = -getElevatorHeight();
    // left_.SetSelectedSensorPosition(0);
    // right_.SetSelectedSensorPosition(0);
}

/**
 * @brief This function converts the motor units used by the talon to meters.
 * 
 * @param motor_units the raw sensor units
 * @return double the unit in meters
 */
double Elevator::talonUnitsToMeters(double motor_units) {
    return motor_units / ElevatorConstants::TALON_FX_COUNTS_PER_REV * ElevatorConstants::ONE_MOTOR_REVOLUTION_TO_DISTANCE_TRAVELLED;
}

/**
 * @brief Converts the talon motor units to an angle in degrees
 *  
 * @param motor_units the raw sensor units
 * @return double the angle of the motor in degrees
 */
double Elevator::talonUnitsToAngle(double motor_units) {
    return std::fmod(motor_units * 360.0 / ElevatorConstants::TALON_FX_COUNTS_PER_REV,  360.0);
}

std::string Elevator::getStateString(){
    switch(current_state_){
        case MANUAL:
            return "MANUAL";
        case MOVING:
            return "MOVING";
        case HOLDING_POS:
            return "HOLDING POS";
        default:
            return "NONE";
    }
}

std::string Elevator::getTargetString(){
    switch(current_target_){
        case CUSTOM:
            return "CUSTOM";
        case LOW:
            return "LOW";
        case MID:
            return "MID";
        case HIGH:
            return "HIGH";
        case STOWED:
            return "STOWED";
        default:
            return "NONE";
    }
}

void Elevator::CoreShuffleboardInit(){
    frc::SmartDashboard::PutNumber(name_ + " ks", ElevatorConstants::KS);
    frc::SmartDashboard::PutNumber(name_ + " kv", ElevatorConstants::KV);
    frc::SmartDashboard::PutNumber(name_ + " ka", ElevatorConstants::KA);
    frc::SmartDashboard::PutNumber(name_ + " kg", ElevatorConstants::KG);

    frc::SmartDashboard::PutNumber(name_ + " kp", ElevatorConstants::KP);
    frc::SmartDashboard::PutNumber(name_ + " ki", ElevatorConstants::KI);
    frc::SmartDashboard::PutNumber(name_ + " kd", ElevatorConstants::KD);

    frc::SmartDashboard::PutNumber(name_ + " mv", ElevatorConstants::MAX_VELOCITY);
    frc::SmartDashboard::PutNumber(name_ + " ma", ElevatorConstants::MAX_ACCELERATION);

    frc::SmartDashboard::PutNumber(name_ + " set setPoint", 0.0);

    frc::SmartDashboard::PutNumber(name_ + " volts to use", ElevatorConstants::MAX_VOLTS);

    frc::SmartDashboard::PutString(name_ + " state", getStateString());
    frc::SmartDashboard::PutString(name_ + " target", getTargetString());

    frc::SmartDashboard::PutNumber(name_ + " ff setPoint", feedforward_.getSetpoint());
    frc::SmartDashboard::PutNumber(name_ + " ff startPoint", feedforward_.getStartpoint());

    frc::SmartDashboard::PutNumber(name_ + " out current",  left_.GetOutputCurrent());
    frc::SmartDashboard::PutNumber(name_ + " supply current",  left_.GetSupplyCurrent());
    frc::SmartDashboard::PutNumber(name_ + " left temp", left_.GetTemperature());
    frc::SmartDashboard::PutNumber(name_ + " right temp", right_.GetTemperature());
};

void Elevator::CoreShuffleboardPeriodic(){
    // frc::SmartDashboard::PutNumber(name_ + " lm rotation", getLeftRotation());
    // frc::SmartDashboard::PutNumber(name_ + " rm rotation", getRightRotation());

    frc::SmartDashboard::PutNumber("current ev position", current_pose_.pos);
    frc::SmartDashboard::PutNumber("current ev velocity", current_pose_.vel);

    frc::SmartDashboard::PutString(name_ + " state", getStateString());
    frc::SmartDashboard::PutString(name_ + " target", getTargetString());

    frc::SmartDashboard::PutNumber(name_ + " ff setPoint", feedforward_.getSetpoint());
    frc::SmartDashboard::PutNumber(name_ + " ff startPoint", feedforward_.getStartpoint());

    frc::SmartDashboard::PutBoolean(name_ + " limit switch", !limit_switch_.Get());

    frc::SmartDashboard::PutNumber(name_ + " out current",  left_.GetOutputCurrent());
    frc::SmartDashboard::PutNumber(name_ + " supply current",  left_.GetSupplyCurrent());
    frc::SmartDashboard::PutNumber(name_ + " left temp", left_.GetTemperature());
    frc::SmartDashboard::PutNumber(name_ + " right temp", right_.GetTemperature());
};

void Elevator::CoreShuffleboardUpdate(){
    feedforward_.setKs(frc::SmartDashboard::GetNumber(name_ + " ks", ElevatorConstants::KS));
    feedforward_.setKv(frc::SmartDashboard::GetNumber(name_ + " kv", ElevatorConstants::KV));
    feedforward_.setKa(frc::SmartDashboard::GetNumber(name_ + " ka", ElevatorConstants::KA));
    feedforward_.setKg(frc::SmartDashboard::GetNumber(name_ + " kg", ElevatorConstants::KG));

    feedforward_.setPIDConstants(frc::SmartDashboard::GetNumber(name_ + " kp", ElevatorConstants::KP),
                                 frc::SmartDashboard::GetNumber(name_ + " ki", ElevatorConstants::KI),
                                 frc::SmartDashboard::GetNumber(name_ + " kd", ElevatorConstants::KD));

    feedforward_.setMaxVelocity(frc::SmartDashboard::GetNumber(name_ + " mv", ElevatorConstants::MAX_VELOCITY));
    feedforward_.setMaxAcceleration(frc::SmartDashboard::GetNumber(name_ + " ma", ElevatorConstants::MAX_ACCELERATION));

    double setPoint = frc::SmartDashboard::GetNumber(name_ + " set setPoint", 0);
    if (setPoint < ElevatorConstants::MAX_EXTENSION) {
        ExtendToCustomPos(setPoint);
    }

    max_volts_ = frc::SmartDashboard::GetNumber(name_ + " volts to use", ElevatorConstants::MAX_VOLTS);
};