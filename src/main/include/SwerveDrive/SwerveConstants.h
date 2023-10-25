#pragma once

#include <string>
#include <cmath>

#include <frc2/command/PIDCommand.h>

#include "Geometry/Point.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846	/* pi */
#endif

namespace SwerveConstants{

    struct SwerveStruct{
        std::string name;
        int driveID;
        int turnID;
        int encoderID;
        double encoderOffset; //Degrees
        Point pos; //Meters
        bool encoderInverted = false; //If the motor/wheel spins opposite to the encoder
        frc::PIDController turnPID = {2.7, 0, 0};
    };
    const std::string canBus = "drivebase";
    //X axis is the side-side axis, back is negative, forwards is positive
    //Y axis is the front-back axis, left is negative, right is positive
     //This way an angle of 90 is to go forward
    const double HALFSIDE_LENGTH = 0.3429;
    //X axis is the side
    //Y axis is front-back
    const SwerveStruct FL = {   "Front Left",       //Name
                                23,                 //Drive ID
                                5,                  //Turn ID
                                2, -185.009 + 180.0,        //Encoder ID, offset
                                {HALFSIDE_LENGTH, HALFSIDE_LENGTH}   //Position
                            };

    const SwerveStruct FR = {   "Front Right",
                                4,
                                10,
                                8, -9.31,
                                {HALFSIDE_LENGTH, -HALFSIDE_LENGTH}
                            };

    const SwerveStruct BL = {   "Back Left",
                                22,
                                19,
                                6, -98.613,
                                {-HALFSIDE_LENGTH, HALFSIDE_LENGTH}
                            };

    const SwerveStruct BR = {   "Back Right",
                                1,
                                7,
                                9, -209.855 + 180.0,
                                {-HALFSIDE_LENGTH, -HALFSIDE_LENGTH}
                            };

    const SwerveStruct MODULES[] = {FL, FR, BL, BR};
    const int NUMSWERVE = 4;

    const double DRIVE_MAX_VOLTS = 4.0; //Volts
    const double TURN_MAX_VOLTS = 12.0; //Volts

    const double WHEEL_RADIUS = 0.0508; //Meters
    const double TICKS_PER_ROTATION = 2048.0;
    const double TICKS_PER_RADIAN = (TICKS_PER_ROTATION / (2.0*M_PI));
};