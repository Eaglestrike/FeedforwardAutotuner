#pragma once

#include <ctre/Phoenix.h>

#include <rev/CANSparkMax.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>

#include "Elevator/ElevatorIntakeConstants.h"
#include "Elevator/Lidar/LidarReader.h"

#include "Util/Utils.h"

class Intake{
    public:
        Intake();

        enum MechState{
            MOVING,
            AT_TARGET,
            STOPPED,
            MANUAL
        };

        enum TargetState{
            STOWED,
            DEPLOYED,
            HALFSTOWED,
            HP
        };
        
        void Zero();
        void ManualPeriodic(double wristVolts);
        void TeleopPeriodic();
        void Periodic();    
        void Stow();
        void HalfStow();
        void DeployNoRollers(bool hp = false);
        void StopRollers();
        void StartRollers(bool outtaking, bool cone);
        void SetHPIntake(bool hp);
        void DeployIntake(bool cone); 
        void DeployOuttake(bool cone);
        void ChangeDeployPos(double newPos); //pos should be in radians, w 0 as extended and parallel to ground
        void ChangeRollerVoltage(double newVoltage); 
        void UpdateLidarData(LidarReader::LidarData lidarData);
        void Kill();
        // for debugging
        MechState GetState();
        TargetState GetTargetState();
        double GetPos();
        double GetRelPos();
        double GetAbsEncoderPos();

    private:
        void UpdatePose();
        void UpdateTargetPose();
        double FFPIDCalculate();
        void CalcSpeedDecreasePos();
        void SetSetpoint(double setpt);
        bool AtSetpoint();
        void ResetPID();

        void debugTargPose();
        void debugCurPose();
        void debugPutVoltage();

        bool dbg = false, dbg2 = true;

        rev::CANSparkMax m_wristMotor{IntakeConstants::WRIST_MOTOR_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
        rev::SparkMaxRelativeEncoder m_relEncoder = m_wristMotor.GetEncoder();
        // WPI_TalonFX m_rollerMotor{IntakeConstants::ROLLER_MOTOR_ID};
        frc::DutyCycleEncoder m_wristEncoder{IntakeConstants::WRIST_ENCODER_CAN_ID};

        TargetState m_targState = TargetState::STOWED;
        MechState m_state = MechState::AT_TARGET;

        double m_kp = IntakeConstants::EXTEND_DEPLOY_P, m_ki = IntakeConstants::EXTEND_DEPLOY_I, 
               m_kd = IntakeConstants::EXTEND_DEPLOY_D, m_s = IntakeConstants::EXTEND_DEPLOY_S,
               m_g = IntakeConstants::EXTEND_DEPLOY_G, m_v = IntakeConstants::EXTEND_DEPLOY_V,
               m_a = IntakeConstants::EXTEND_DEPLOY_A;
        
        double m_setPt= IntakeConstants::STOWED_POS; 
        double m_curPos, m_curVel, m_curAcc; // cur pose
        double m_targetPos = m_setPt, m_targetVel =0 , m_targetAcc = 0; // motion profile 
        double m_speedDecreasePos, // pos in motion profile where start decelerating
               m_totalErr = 0; // integral of position error for PID

        double m_rollerStartTime;

        double m_rollerVolts;
        double m_customDeployPos =-1, 
        m_customRollerVolts = -1;// get rid of this

        bool m_outtaking, m_cone;
        // bool m_hpSt = false;
        bool m_hasGamePiece = false;

        double m_hasConeStartTime = 0;

        double m_absEncoderInit = 0;
        double m_relEncoderInit = 0;
};