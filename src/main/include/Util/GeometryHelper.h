#pragma once

#ifndef M_PI
#define M_PI 3.14159265358979323846	/* pi */
#endif

#include "Point.h"

namespace GeometryHelper{
    const static Point ORIGIN{0,0};

    double toDeg(double rad);

    double toRad(double deg);

    /**
     * Input radians, output [0, 2*pi]
    */
    double getPrincipalAng(double ang);

    /**
     * @param ang Input radians
     * @returns output in range [-pi, pi]
    */
    double getPrincipalAng2(double ang);


    double getPrincipalAng2Deg(double ang);

    /**
     * Angle from ang1 to ang2
    */
    double getAngDiff(double ang1, double ang2);

    double getAngDiffDeg(double ang1, double ang2);

    /**
    * @param angMid angle tested in between 1 and 2
    */
    bool angInBetween(double angMid, double ang1, double ang2);

    double getAbsAngDiff(double ang1, double ang2);//Just subtract lmao
}