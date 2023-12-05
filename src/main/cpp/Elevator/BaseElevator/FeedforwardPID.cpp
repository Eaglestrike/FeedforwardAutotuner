#include "Elevator/BaseElevator/FeedforwardPID.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include <Util/Utils.h>

/**
 * @brief Basic constructor meant for feedforward without PID use.
 *
 * @param ks static constant
 * @param kv velocity to volts constant
 * @param ka acceleration to volts constant
 * @param kg constant to account for acceleration of gravity
 */
FeedforwardPID::FeedforwardPID(ElevatorConstants::FeedforwardConfig constants, bool shuffleboard):
    ks(constants.ks), kv(constants.kv), ka(constants.ka), kg(constants.kg),
    max_velocity(constants.maxVel), max_acceleration(constants.maxAccel),
    kp(constants.kp), ki(constants.ki), kd(constants.kd), accum_(0),
    shuffleboard(shuffleboard), prevTime_{0}
{
    isRunning = 0.0;
    reversed = false;
};

/**
 * @brief Runs every periodic cycle.
 *
 * @param current_pose a pair containing the velocity and distance (respectively) of the current system.
 * @return a voltage that a motor is expected to use
 */
double FeedforwardPID::periodic(Poses::Pose1D current_pose)
{
    double curT = Utils::GetCurTimeS();
    double dT = curT - prevTime_;
    prevTime_ = curT;

    if (!isRunning) {
        start();
    }

    Poses::Pose1D expected_pose = getExpectedPose(timer.Get().value());
    double feedforward_voltage = calculateFeedforwardVoltage(expected_pose.velocity, expected_pose.acceleration);
    double pid_voltage = calculatePIDVoltage(expected_pose, current_pose, dT);

    // debug prints
    if(shuffleboard){
        frc::SmartDashboard::PutNumber("timer value: ", timer.Get().value());

        frc::SmartDashboard::PutNumber("expected ev velocity", expected_pose.velocity);
        frc::SmartDashboard::PutNumber("expected ev position", expected_pose.position);
        frc::SmartDashboard::PutNumber("expected acceleration", expected_pose.acceleration);

        frc::SmartDashboard::PutNumber("position error", expected_pose.position - current_pose.position);
        frc::SmartDashboard::PutNumber("velocity error", expected_pose.velocity - current_pose.velocity);
        frc::SmartDashboard::PutNumber("acceleration error", expected_pose.acceleration - current_pose.acceleration);

        frc::SmartDashboard::PutNumber("ff voltage", feedforward_voltage);
        frc::SmartDashboard::PutNumber("pid voltage", pid_voltage);
    }

    return feedforward_voltage + pid_voltage;
}

/**
 * @brief PID calculations to make feedforward loop more accurate
 *
 * @param expected Pose containing information about where system should be
 * @param current Pose containing information about where system actually is
 * @return double voltage to add to feedforward loop to compensate for inaccuracies in velocity/position.
 */
double FeedforwardPID::calculatePIDVoltage(Poses::Pose1D expected, Poses::Pose1D current, double dt)
{
    accum_ += dt * (expected.position - current.position);
    double posErr = expected.position - current.position;
    if (std::abs(posErr) < ElevatorConstants::POSITION_ERROR_TOLERANCE){
        posErr = 0.0;
    }
    return kp * posErr + ki * accum_ + kd * (expected.velocity - current.velocity);
}

/**
 * @brief Returns the voltage based on the feedforward formula
 *
 * @param velocity - expected velocity, based on motion profile
 * @param acceleration - expected acceleration, based on motion profile
 *
 * @return the voltage to move
 */
double FeedforwardPID::calculateFeedforwardVoltage(double velocity, double acceleration)
{
    if(velocity != 0.0){
        return ks * sign(velocity) + kg + kv * velocity + ka * acceleration;
    }
    else{
        return kg + kv * velocity + ka * acceleration;
    }
}

/**
 * @brief The sign of a double
 *
 * @param value the value to get the sign of
 * @return 1.0 if the value is greater than 0, or -1.0 otherwise
 */
double FeedforwardPID::sign(double value)
{
    return value > 0.0 ? 1.0 : -1.0;
}

/**
 * @brief Starts the timer
 *
 */
void FeedforwardPID::start()
{
    timer.Start();
    isRunning = true;
    accum_ = 0;
}

/**
 * @brief Stops the timer
 *  
 */
void FeedforwardPID::stop() {
    timer.Stop();
}

/**
 * Resets and starts the timer 
 * 
 */
void FeedforwardPID::reset() {
    timer.Reset();
    start();
    accum_ = 0;
}

/**
 * @brief A feedforward function that gets the elevator pose based on current time
 *
 * @param time current system time
 * @return pose a pose containing the distance
 */
Poses::Pose1D FeedforwardPID::getExpectedPose(double time)
{
    Poses::Pose1D pose{0.0, 0.0, 0.0};
    if (max_velocity == 0 || max_acceleration == 0) {
        return pose;
    } 

    // whether moving up or down

    if(shuffleboard){
        frc::SmartDashboard::PutBoolean("phase 1", false);
        frc::SmartDashboard::PutBoolean("phase 2", false);
        frc::SmartDashboard::PutBoolean("phase 3", false);
        frc::SmartDashboard::PutBoolean("Reversed?", reversed);
    }

    double reversed_coefficient = reversed? -1.0:1.0;

    // if in the acceleration phase
    if (time < 0){
        pose.position = 0;
        pose.velocity = 0;
        pose.acceleration = 0;
    }
    else if (time < acceleration_time){
        if(shuffleboard){
            frc::SmartDashboard::PutBoolean("phase 1", true);
        }
        pose.acceleration = reversed_coefficient * max_acceleration;
        pose.velocity = reversed_coefficient * max_acceleration * time;
        pose.position = reversed_coefficient * 0.5 * pose.velocity * time;
    }
    // if in the velocity phase
    else if (velocity_time != 0 && time < acceleration_time + velocity_time){
        if(shuffleboard){
            frc::SmartDashboard::PutBoolean("phase 2", true);
        }
        pose.acceleration = 0.0;
        pose.velocity = reversed_coefficient * max_velocity;
        // adds phase 1 to however much of phase 2 has been gone through
        pose.position = reversed_coefficient * 0.5 * max_velocity * acceleration_time + max_velocity * reversed_coefficient * (time - acceleration_time);
    }
    // if in the deceleration phase
    else if (time < velocity_time + acceleration_time*2){
        if(shuffleboard){
            frc::SmartDashboard::PutBoolean("phase 3", true);
        }
        double max_vel = max_velocity;
        if(velocity_time == 0){
            max_vel = max_acceleration * acceleration_time;
        }
        double first_phase_distance = reversed_coefficient * max_vel * acceleration_time;
        double second_phase_distance = reversed_coefficient * max_vel * velocity_time;
        double time_in_triangle = time - (acceleration_time + velocity_time);

        pose.acceleration = -1.0 * reversed_coefficient * max_acceleration;
        pose.velocity = reversed_coefficient * max_vel - (reversed_coefficient * max_acceleration * (time_in_triangle));

        // trapezoidal area
        double third_phase_distance = reversed_coefficient * (max_velocity + pose.velocity) / 2.0 * time_in_triangle;

        pose.position = first_phase_distance + second_phase_distance + third_phase_distance;
    }

    else{
        pose.position = reversed_coefficient * total_distance_;
        pose.velocity = 0.0;
        pose.acceleration = 0.0;
    }

    pose.position += startpoint_;

    return pose;
}

bool FeedforwardPID::isFinished(){
    return timer.Get().value() > velocity_time + acceleration_time*2;
}

bool FeedforwardPID::atSetpoint(Poses::Pose1D current_pose, double xTol, double vTol){
    double xError = current_pose.position - setpoint_;
    double vError = current_pose.velocity;
    return Utils::NearZero(xError, xTol) && Utils::NearZero(vError, vTol);
}
/**
 * @brief Optional method that must be called if PID calculations are wanted.
 *
 * @param kp kp constant, converts change in velocity to voltage
 * @param kd kd constant, converts change in position to voltage
 */
void FeedforwardPID::setPIDConstants(double kp, double ki, double kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

/**
 * @brief Calculates and stores the time spent in the different phases for the feedforward loop
 * 
 */
void FeedforwardPID::recalculateTimes() {
    total_distance_ = setpoint_ - startpoint_;
    reversed = total_distance_ < 0;
    total_distance_ = std::abs(total_distance_);

    // the time spent accelerating (or decelerating)
    if (max_velocity == 0 || max_acceleration == 0) {
        return;
    } else {
        acceleration_time = max_velocity / max_acceleration;

        // the time spent maintaining a constant velocity
        velocity_time = (total_distance_ - max_velocity * acceleration_time) / max_velocity;

        if (velocity_time < 0) {
            velocity_time = 0.0;
            acceleration_time = std::sqrt(total_distance_ / max_acceleration);
        }

        // TODO: double check this following line
        reset();
    }
}

// getters and setters

double FeedforwardPID::getKs() {
    return ks;
}

void FeedforwardPID::setKs(double ks) {
    this->ks = ks;
}

double FeedforwardPID::getKg() {
    return kg;
}

void FeedforwardPID::setKg(double kg) {
    this->kg = kg;
}

double FeedforwardPID::getKv() {
    return kv;
}

void FeedforwardPID::setKv(double kv) {
    this->kv = kv;
}


double FeedforwardPID::getKa() {
    return ka;
}

void FeedforwardPID::setKa(double ka) {
    this->ka = ka;
}

double FeedforwardPID::getKp() {
    return kp;
}

double FeedforwardPID::getKd() {
    return kd;
}

double FeedforwardPID::getMaxAcceleration() {
    return max_acceleration;
}

void FeedforwardPID::setMaxAcceleration(double new_acc) {
    max_acceleration = new_acc;
    recalculateTimes();
}

double FeedforwardPID::getMaxVelocity() {
    return max_velocity;
}

void FeedforwardPID::setMaxVelocity(double new_vel) {
    max_velocity = new_vel;
    recalculateTimes();
}

double FeedforwardPID::getStartpoint() {
    return startpoint_;
}

double FeedforwardPID::getSetpoint() {
    return setpoint_;
}

void FeedforwardPID::setTotalDistance(double new_position, double curr_pos) {
    startpoint_ = curr_pos;
    setpoint_ = new_position;
    recalculateTimes();
}

bool FeedforwardPID::getReversed() {
    return reversed;
}

void FeedforwardPID::setReversed(bool reversed) {
    this->reversed = reversed;
}