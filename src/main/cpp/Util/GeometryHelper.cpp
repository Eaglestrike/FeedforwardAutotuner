#include "Util\GeometryHelper.h"

double GeometryHelper::toDeg(double rad){
    return rad / M_PI * 180.0;
}

double GeometryHelper::toRad(double deg){
    return deg * M_PI / 180.0;
}

double GeometryHelper::getPrincipalAng(double ang){
    int div = ang / (2.0*M_PI);
    double multiple = ((double)div) * 2.0 * M_PI;
    double mod = ang - multiple; 
    return mod;
}

double GeometryHelper::getPrincipalAng2(double ang){
    //https://www.desmos.com/calculator/yctu2qoevr
    double div = floor(((ang - M_PI) / (2.0*M_PI)) + 1);
    double multiple = div * 2.0*M_PI;
    double mod = ang - multiple; 
    return mod;
}

double GeometryHelper::getPrincipalAng2Deg(double ang){
    double div = floor(((ang - 180.0)/ (360.0)) + 1);
    double multiple = div * 360.0;
    double mod = ang - multiple; 
    return mod;
}

double GeometryHelper::getAngDiff(double ang1, double ang2){
    return getPrincipalAng2(ang2 - ang1);
}

double GeometryHelper::getAngDiffDeg(double ang1, double ang2){
    return getPrincipalAng2Deg(ang2 - ang1);
}

bool GeometryHelper::angInBetween(double angMid, double ang1, double ang2){
    //https://www.desmos.com/calculator/bo2iuiuukb
    double angDiff12 = getAngDiff(ang1, ang2);
    double angDiff1Mid = getAngDiff(ang1, angMid);
    //std::cout<< "angMid: " << angMid << ", ang1: " << ang1 << ", ang2: " << ang2 << std::endl;
    //std::cout<< "ang diffl2:" <<angDiff12<< ", angDiff1Mid:" << angDiff1Mid << std::endl;
    if(angDiff12 > 0){
        if(angDiff1Mid > 0 && angDiff1Mid < angDiff12){
            return true;
        }
        else{
            return false;
        }
    }
    else{
        if(angDiff1Mid < 0 && angDiff1Mid > angDiff12){
            return true;
        }
        else{
            return false;
        }
    }
}

double GeometryHelper::getAbsAngDiff(double ang1, double ang2){//Just subtract lmao
    return ang2 - ang1;
}