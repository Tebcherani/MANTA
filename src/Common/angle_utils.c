#include <math.h>

#include "angle_utils.h"
#include "common_defs.h"

double wrap_angle(double angle) {
    double ipart, result;

    result = 2*PI*modf(angle/(2*PI), &ipart);

    if (result >= PI) result -= 2*PI;
    if (result < -PI) result += 2*PI;

    return result;
}