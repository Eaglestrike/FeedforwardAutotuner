#pragma once

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265359
#endif

namespace IntakeElevatorConstants{
    struct ElevatorIntakePosInfo{
        double ELEVATOR_LENG; //vir units
        double INTAKE_ANGLE; //rads
    };

    struct GamePieceInfo{
        ElevatorIntakePosInfo SCORE_LOW;
        ElevatorIntakePosInfo SCORE_MID;
        ElevatorIntakePosInfo SCORE_HIGH;
        ElevatorIntakePosInfo GROUND_INTAKE;
        ElevatorIntakePosInfo FLANGE_INTAKE;
        ElevatorIntakePosInfo HP_INTAKE;
    };

    const GamePieceInfo coneScoreInfo{.SCORE_LOW =      {0.0, 1.4}, // was 0.58
                                      .SCORE_MID =      {0.316, 0.73},
                                      .SCORE_HIGH =     {0.545, 0.8},
                                      .GROUND_INTAKE =  {0.0, 0.183}, // ground intake upright
                                      .FLANGE_INTAKE =  {0.0, 0.0}, // ground intake flange
                                      .HP_INTAKE =      {0.0, 1.43}}; // intake from drop station

    const GamePieceInfo cubeScoreInfo{.SCORE_LOW =      {0.0, 1.4}, // was 0.58
                                      .SCORE_MID =      {0.291, 1.25},
                                      .SCORE_HIGH =     {0.573, 0.8}, // elevator pos seems wayyy to high
                                      .GROUND_INTAKE =  {0.0, 0.13}, // intake from ground
                                      .FLANGE_INTAKE =  {0.0, 0.44}, // NA for cube (set to default?)
                                      .HP_INTAKE =      {0.0, 0.0}}; 
}

namespace IntakeConstants{
    const int WRIST_MOTOR_ID = 4;
    const int ROLLER_MOTOR_ID = 31;
    const int WRIST_ENCODER_CAN_ID = 0;

    //EXTEND DEPLOY FEEDFORWARD-PID

    //feedforward constants
    const double EXTEND_DEPLOY_S = 0.05; // volts
    const double EXTEND_DEPLOY_G = 0.275; // volts needed to resist gravity
    const double EXTEND_DEPLOY_V = 1.0; // volts*seconds/rad
    const double EXTEND_DEPLOY_A = 0.04; // volts*seconds^2/rad

    //pid accounts for velocity and position error
    const double EXTEND_DEPLOY_P = 1.75; // corrects position error
    const double EXTEND_DEPLOY_I = 0.15; // accounts for cumulative pos err
    const double EXTEND_DEPLOY_D = 0.1; // corrects velocity error
    
    //trapezoidal motion profiling
    const double WRIST_MAX_VEL = 5; //rads per sec
    const double WRIST_MAX_ACC = 10; //rads per sec^2

    // all in rads
    const double WRIST_POS_TOLERANCE = 0.1;
    const double WRIST_ABS_ENCODER_OFFSET = - 1.84 - 0.094; //cad says 0.094 should b 0.034 but idk

    // wrist positions in radians, 0.0 is parallel to ground and flipping intake up is positive
    const double MAX_POS = 1.85;
    const double STOWED_POS = 1.84;
    const double INTAKE_UPRIGHT_ANGLE = 1.4; // halfstowed


    //todo:
    const double MIN_POS = 0.0; // this is fine for now bc we currently have no need for negative pos
    const double DEPLOYED_POS = 0.0; // unused

     struct GamePieceInfo {
        double SPIKE_CURRENT;
        double IN_VOLTS;
        double STRONG_IN_VOLTS;
        double KEEP_VOLTS;
        double OUT_VOLTS;
    };

    const GamePieceInfo CONE_INFO = {.SPIKE_CURRENT = 100.0,
                                     .IN_VOLTS = 8.0,
                                     .STRONG_IN_VOLTS = 9.0,
                                     .KEEP_VOLTS = -1.1,
                                     .OUT_VOLTS = 3.0};
    const GamePieceInfo CUBE_INFO = {.SPIKE_CURRENT = 15.0,
                                     .IN_VOLTS = 4.0,
                                     .STRONG_IN_VOLTS = 4.0,
                                     .KEEP_VOLTS = 1.2,
                                     .OUT_VOLTS = 1.7}; // MAYBE LESS
    
    // const double KEEP_CONE_CURRENT = 5.0;
    // const double KEEP_CUBE_CURRENT = 10.0;

    const double ROLLER_MAX_VOLTS = 5.0;

    const double WRIST_MAX_VOLTS = 5.0;

    const double REL_CONV_FACTOR = 1 * (1.0 / 20.0) * (16.0 / 36.0) * (2 * M_PI) * (1.885 / 1.588);
}