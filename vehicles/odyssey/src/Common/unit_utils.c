#include <math.h>
#include <string.h>

#include "unit_utils.h"

double convert_deg_to_rad(double degree) {
    return degree * M_PI /180.0;
}

double convert_rad_to_deg(double rad) {
    return rad * 180.0 / M_PI;
}

int convert_units(const char *expected_units, const char *given_units, double input_value, double *output_value) {
    if (strcmp(expected_units, given_units) == 0) {
        *output_value = input_value;
        return 1;
    }

    if (!strncmp(expected_units, "(rad", 4) && !strncmp(given_units, "(deg", 4)) *output_value = convert_deg_to_rad(input_value);
    else if (!strncmp(expected_units, "(deg", 4) && !strncmp(given_units, "(rad", 4)) *output_value = convert_rad_to_deg(input_value);
    else if (!strncmp(expected_units, "(m", 2) && !strncmp(given_units, "(km", 3)) *output_value = 1000.0 * input_value;
    else if (!strncmp(expected_units, "(km", 3) && !strncmp(given_units, "(m", 2)) *output_value = 0.001 * input_value;
    else return 0;

    return 1;
}