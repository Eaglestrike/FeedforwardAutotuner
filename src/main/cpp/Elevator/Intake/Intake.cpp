#include "Elevator/Intake/Intake.h"
#include <iostream>

//constructor j sets motor to brake mode
Intake::Intake() {
    m_wristMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    if (dbg){
        frc::SmartDashboard::PutNumber("Setpoint", 0);
        frc::SmartDashboard::PutBoolean("Deploy", false);
        frc::SmartDashboard::PutNumber("voltage", 0.0);
        frc::SmartDashboard::PutBoolean("Cone", false);
        frc::SmartDashboard::PutBoolean("Outtake", false);
        //m_wristMotor.SetInverted(true);
        frc::SmartDashboard::PutNumber("g", m_g); 
        frc::SmartDashboard::PutNumber("s", m_s); 
        frc::SmartDashboard::PutNumber("v", m_v); 
        frc::SmartDashboard::PutNumber("a", m_a); 
    }
    frc::SmartDashboard::PutNumber("cone spike current", IntakeConstants::CONE_INFO.SPIKE_CURRENT);

    intakeTuner_.setMin(-0.3);
    intakeTuner_.setMax(1.8);
    intakeTuner_.Start();
}

void Intake::Zero() {
    m_absEncoderInit = GetAbsEncoderPos();
    m_relEncoder.SetPosition(0);
}

// needs to be called INSTEAD of teleop periodic
void Intake::ManualPeriodic(double wristVolts){
    m_wristMotor.SetVoltage(units::volt_t(std::clamp(-wristVolts, -IntakeConstants::WRIST_MAX_VOLTS, IntakeConstants::WRIST_MAX_VOLTS)));
}

void Intake::Periodic(){
    UpdatePose();
    intakeTuner_.ShuffleboardUpdate();
}

// teleop periodic runs on state machine
void Intake::TeleopPeriodic(){
    if (dbg){
        debugCurPose();
        debugTargPose();    
        debugPutVoltage();
    }
    
    frc::SmartDashboard::PutNumber("wrist setpt", m_setPt);
    double wristVolts = 0;
    switch (m_state){
        case MOVING:
            UpdateTargetPose(); // bc still using motion profile 
            wristVolts = FFPIDCalculate();
            if (AtSetpoint()){
                m_state = AT_TARGET;
                ResetPID();
                m_targetPos = m_setPt;
                m_targetVel = 0.0;
                m_targetAcc = 0.0;
            }
            break;
        case AT_TARGET:
        {
            wristVolts = FFPIDCalculate();
            IntakeConstants::GamePieceInfo curInfo = IntakeConstants::CUBE_INFO;
            if (m_cone) //{
                curInfo = IntakeConstants::CONE_INFO;
            break;
        }
        case TUNING:
            intakeTuner_.setPose({.pos = m_curPos, .vel = m_curVel, .acc = m_curAcc});
            wristVolts = intakeTuner_.getVoltage();
            break;
        case STOPPED:
            break;
    }
    if (dbg){
        frc::SmartDashboard::PutNumber("wrist volts", wristVolts);
    }
    if (dbg2){
        frc::SmartDashboard::PutBoolean("outtaking", m_outtaking);
        frc::SmartDashboard::PutBoolean("has gp", m_hasGamePiece);
    }
    m_wristMotor.SetVoltage(units::volt_t(std::clamp(-wristVolts, -IntakeConstants::WRIST_MAX_VOLTS, IntakeConstants::WRIST_MAX_VOLTS)));
}

void Intake::UpdateLidarData(LidarReader::LidarData lidarData){
    if (lidarData.isValid) m_hasGamePiece = lidarData.hasCone || lidarData.hasCube;
}

//completely stows the intake at its maximum position
void Intake::Stow(){
    if (m_targState == STOWED) return;
    m_targState = STOWED;
    SetSetpoint(IntakeConstants::STOWED_POS);
    m_rollerVolts = 0;
    m_state = MOVING;
}

// half-stows the intake, moving it out of the way of the elevator
void Intake::HalfStow(){
    if (m_targState == HALFSTOWED) return;
    m_targState = HALFSTOWED;
    SetSetpoint(IntakeConstants::INTAKE_UPRIGHT_ANGLE);
    m_rollerVolts = 0;
    m_state = MOVING;
}

// deploys the intake to intake a cone or cube
void Intake::Deploy(){
    if ((m_targState == DEPLOYED)) return;
    else m_targState = DEPLOYED;
    if (m_customDeployPos == -1)
        m_setPt = IntakeConstants::DEPLOYED_POS;
    else 
        m_setPt = m_customDeployPos;

    SetSetpoint(m_setPt);
    m_state = MOVING;
}

void Intake::StartTuning(){
    m_state = TUNING;
    std::cout<<"Intake Set State Tuning"<<std::endl;
};

void Intake::UseTuningValues(bool tuningVals){
    if(tuningVals){
        FFAutotuner::FFConfig ff = intakeTuner_.getFeedforward();
        m_s = ff.ks;
        m_g = ff.kg;
        m_v = ff.kv;
        m_a = ff.ka;
    }
    else{
        m_kp = IntakeConstants::EXTEND_DEPLOY_P;
        m_ki = IntakeConstants::EXTEND_DEPLOY_I;
        m_kd = IntakeConstants::EXTEND_DEPLOY_D;
        m_s = IntakeConstants::EXTEND_DEPLOY_S;
        m_g = IntakeConstants::EXTEND_DEPLOY_G;
        m_v = IntakeConstants::EXTEND_DEPLOY_V;
        m_a = IntakeConstants::EXTEND_DEPLOY_A;
    }
};

//changes the position the intake will deploy to when a deploy method iscalled
//so not an action method
void Intake::ChangeDeployPos(double newPos){
    m_customDeployPos = std::clamp(newPos, IntakeConstants::MIN_POS, IntakeConstants::MAX_POS);
}

//disables intake
void Intake::Kill(){
    m_state = STOPPED;
}

Intake::MechState Intake::GetState(){
    return m_state;
}

Intake::TargetState Intake::GetTargetState(){
    return m_targState;
}

double Intake::GetPos(){
    return m_curPos;
}

double Intake::GetRelPos() {
    return -m_relEncoder.GetPosition() * IntakeConstants::REL_CONV_FACTOR + m_absEncoderInit;
}

// absolute encoder pos in radians
double Intake::GetAbsEncoderPos() {
    return m_wristEncoder.GetAbsolutePosition() * 2 * M_PI + IntakeConstants::WRIST_ABS_ENCODER_OFFSET;
}

//the following functions are all private methods

//Updates the current position, velocity, and acceleration of the wrist
void Intake::UpdatePose(){
    double newPos = GetRelPos(); // might need to negate or do some wrap around calculations
    double newVel = (newPos - m_curPos)/0.02;
    m_curAcc = (newVel - m_curVel)/0.02;
    m_curVel = newVel;
    m_curPos = newPos;
}

//updates the trapezoidal motion profile
void Intake::UpdateTargetPose(){
    double newP = m_targetPos, newV = m_targetVel, newA = m_targetAcc;
    
    newP += m_targetVel * 0.02;

    if (m_speedDecreasePos < m_setPt){ // if trapezoid is pos
        if (newP > m_speedDecreasePos) // if after turn pt
            newV = std::max(0.0, m_targetVel - IntakeConstants::WRIST_MAX_ACC * 0.02);
        else 
            newV = std::min(IntakeConstants::WRIST_MAX_VEL, m_targetVel + IntakeConstants::WRIST_MAX_ACC * 0.02);
    } else {
        if (newP > m_speedDecreasePos) // if before the turn pt
            newV = std::max(-IntakeConstants::WRIST_MAX_VEL, m_targetVel - IntakeConstants::WRIST_MAX_ACC * 0.02);
        else 
            newV = std::min(0.0, m_targetVel + IntakeConstants::WRIST_MAX_ACC * 0.02);
    }

    if (newV-m_targetVel == 0) newA = 0;
    else if (newV > m_targetVel) newA = IntakeConstants::WRIST_MAX_ACC;
    else newA = -IntakeConstants::WRIST_MAX_ACC;

    m_targetPos = newP;
    m_targetVel = newV;
    m_targetAcc = newA;
}

//calculates voltage output with feedforwardPID
double Intake::FFPIDCalculate(){
    double posErr = m_targetPos - m_curPos, 
    velErr = m_targetVel - m_curVel;
    m_totalErr += posErr * 0.02;
    if (fabs(posErr) <= IntakeConstants::WRIST_POS_TOLERANCE) posErr =0;
    double pid = m_kp*posErr + m_kd*velErr + m_ki*m_totalErr;
    double s = m_s;
    if (m_targetVel < 0) s = -m_s;
    else if (m_targetVel == 0) s = 0;
    double ff = m_g* cos(m_targetPos) + s + m_v*m_targetVel + m_a*m_targetAcc;
    if (dbg){
        frc::SmartDashboard::PutNumber("posErr", posErr); 
        frc::SmartDashboard::PutNumber("velErr", velErr); 
        frc::SmartDashboard::PutNumber("ff out", ff); 
        frc::SmartDashboard::PutNumber("pid out", pid); 
        //see how each term is contributing to ff
        m_g = frc::SmartDashboard::GetNumber("g", m_g); 
        m_s = frc::SmartDashboard::GetNumber("s", m_s); 
        m_v = frc::SmartDashboard::GetNumber("v", m_v); 
        m_a = frc::SmartDashboard::GetNumber("a", m_a); 
    }
        frc::SmartDashboard::PutNumber("pid", pid);
            frc::SmartDashboard::PutNumber("ff", ff);


    return pid+ff;
}

//calculates the position at which the speed will begin decreasing
void Intake::CalcSpeedDecreasePos(){
    double MAX_VEL = IntakeConstants::WRIST_MAX_VEL, MAX_ACC = IntakeConstants::WRIST_MAX_ACC;
    if(fabs(m_setPt - m_curPos) < MAX_VEL*MAX_VEL/MAX_ACC){ // for triangle motion profile
        m_speedDecreasePos = (m_setPt+m_curPos)/2;
    } else if (m_setPt > m_curPos)
        m_speedDecreasePos = m_setPt - MAX_VEL*MAX_VEL/(MAX_ACC*2);
    else 
        m_speedDecreasePos = m_setPt + MAX_VEL*MAX_VEL/(MAX_ACC*2);
}

//changes the setpoint and sets everything up to start the profile
void Intake::SetSetpoint(double setpt){
      m_setPt = setpt;
      m_targetPos = m_curPos;
      m_targetVel = 0.0;
      m_targetAcc = IntakeConstants::WRIST_MAX_ACC;
      if (m_setPt < m_curPos) m_targetAcc *= -1;
      ResetPID();
      CalcSpeedDecreasePos();
}

//returns whether the wrist is at its setpoint
bool Intake::AtSetpoint(){
    if (fabs(m_curPos - m_setPt) <= IntakeConstants::WRIST_POS_TOLERANCE)
        return true;
    return false;
}

void Intake::ResetPID(){
    m_totalErr = 0;
}

// to debug the trapezoidal motion profile
void Intake::debugTargPose(){ 
    ChangeDeployPos(frc::SmartDashboard::GetNumber("Setpoint", m_setPt));
    frc::SmartDashboard::PutNumber("targ vel", m_targetVel);
    frc::SmartDashboard::PutNumber("targ pos", m_targetPos);
    frc::SmartDashboard::PutNumber("targ acc", m_targetAcc);
    bool deploy/*, cone, outtake*/;
    deploy = frc::SmartDashboard::GetBoolean("Deploy", false);
    // cone = frc::SmartDashboard::GetBoolean("Cone", false);
    // outtake = frc::SmartDashboard::GetBoolean("Outtake", false);
    if (deploy){
        Deploy();
        frc::SmartDashboard::PutBoolean("Deploy", false);
    }
}

// to debug the current pose calculations
void Intake::debugCurPose(){
    frc::SmartDashboard::PutNumber("cur vel", m_curVel);
    frc::SmartDashboard::PutNumber("cur pos", m_curPos);
    frc::SmartDashboard::PutNumber("cur acc", m_curAcc);
}

// for tuning, can test constant voltage on wrist or rollers 
// but need to pick which wrist or rollers in the code, since it cant be changed from shuffleboard
void Intake::debugPutVoltage(){
    double voltReq = 0;
    voltReq = frc::SmartDashboard::GetNumber("voltage", voltReq);
    voltReq = std::clamp(voltReq, -IntakeConstants::ROLLER_MAX_VOLTS, IntakeConstants::ROLLER_MAX_VOLTS);
    // if(m_curPos > IntakeConstants::MAX_POS){
    //     voltReq = 0;
    // } else if(m_curPos < IntakeConstants::MIN_POS){
    //     voltReq = 0;
    // }
    std::cout << voltReq << std::endl ;

    // frc::SmartDashboard::PutNumber("roller current", m_rollerMotor.GetOutputCurrent());

    // m_rollerMotor.SetVoltage(units::volt_t(voltReq));
    //m_wristMotor.SetVoltage(units::volt_t(-voltReq));
}
