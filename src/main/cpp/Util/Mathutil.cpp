#include "Util/MathUtil.h"

double MathUtil::sign(double x){
    if(x==0){
        return 0;
    }
    return x > 0? 1.0:-1.0;
}