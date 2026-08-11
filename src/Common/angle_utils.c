#include <math.h>

#include "angle_utils.h"

double wrap_angle(double angle) {
    double integer_part, result;

    // Remove multiples of 2*PI
    result = 2 * M_PI * modf(angle / (2 * M_PI), &integer_part);

    // Keep the result between -PI and PI
    if (result >= M_PI) result -= 2 * M_PI;
    if (result < -M_PI) result += 2 * M_PI;

    return result;
}
