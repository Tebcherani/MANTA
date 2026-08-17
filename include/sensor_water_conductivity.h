#ifndef SENSOR_WATER_CONDUCTIVITY_H
#define SENSOR_WATER_CONDUCTIVITY_H

typedef struct {
    double true_depth;
} water_conductivity_input_t;

double update_water_conductivity(const water_conductivity_input_t *input);

#endif