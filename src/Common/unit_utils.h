#ifndef UNIT_UTILS_H
#define UNIT_UTILS_H

double convert_deg_to_rad(double degree);
double convert_rad_to_deg(double rad);

int convert_units(const char *expected, const char *given, const double inval, double *outval);

#endif