#pragma once

#include "Elevator/Intake/Intake.h"
#include "BaseElevator/Elevator.h"
#include "Lidar/LidarReader.h"

class ElevatorIntake{
    public:
        enum MechanismState{
            MOVING,
            STOPPED
        };

        enum MovingState{
            HALFSTOWING,
            ELEVATOR,
            INTAKE,
            DONE
        };

        enum TargetState{
            STOWED,
            LOW,
            MID,
            HIGH,
            HP,
            GROUND
        };

        ElevatorIntake();
        void Init();
        void Periodic();
        void TeleopPeriodic();
        void Kill();
        void ToggleRoller(bool outtaking);
        void DeployElevatorIntake(double elevatorLength, double intakeDeg);
        void Stow();
        void SetCone(bool cone);
        void ScoreHigh();
        void ScoreMid();
        void ScoreLow();
        void IntakeFromGround();
        void IntakeFlange();
        void IntakeFromHPS();
        void UpdateLidarData(LidarReader::LidarData lidarData);
        void UpdateShuffleboard();
        void ManualPeriodic(double elevator, double intake);
        bool CanMoveFast() const;
        bool IsDone() const;
        void ZeroIntake();

    private:
        void DeployElevatorIntake(IntakeElevatorConstants::ElevatorIntakePosInfo scoreInfo);
        IntakeElevatorConstants::GamePieceInfo GetGPI(bool cone);
        void Debug();
        void DebugScoring();
        void CalcIntakeDeployPos();

        // do not put dbg to true, breaks elevatorintake
        bool dbg = false, dbg2= false;
        
        MechanismState m_state = MOVING;
        MovingState m_movingState = DONE;
        TargetState m_targState;
        bool m_cone, m_outtaking;

        bool m_rollers = false;
        double m_targIntakeAng, m_targElevatorPos;

        bool m_useLidar = true;

        Elevator m_elevator{true, false};
        Intake m_intake;

        IntakeElevatorConstants::GamePieceInfo coneinfo = IntakeElevatorConstants::coneScoreInfo;
        IntakeElevatorConstants::GamePieceInfo cubeinfo = IntakeElevatorConstants::cubeScoreInfo;

        // for debug
        IntakeElevatorConstants::GamePieceInfo& curGPInfo = coneinfo;
};