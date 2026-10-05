#include <math.h>
#include <stdlib.h>

#include "random_utils.h"

double std_normal_random_num(void) {
    static int has_saved_value = 0;
    static double saved_value;

    double scale, radius_squared;
    double x, y;

    // Return the saved value from the previous pair
    if (has_saved_value) {
        has_saved_value = 0;
        return saved_value;
    }

    // Generate a random point inside the unit circle
    do {
        x = 2.0 * drand48() - 1.0;
        y = 2.0 * drand48() - 1.0;
        radius_squared = x * x + y * y;
    } while (radius_squared >= 1.0 || radius_squared == 0.0);

    // Convert the uniform random values to standard normal values
    scale = sqrt(-2.0 * log(radius_squared) / radius_squared);

    saved_value = x * scale;
    has_saved_value = 1;

    return y * scale;
}
