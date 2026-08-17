#ifndef SENSOR_WATER_TEMP_H
#define SENSOR_WATER_TEMP_H

typedef struct {
    double true_depth;
} water_temp_input_t;

double update_water_temp(const water_temp_input_t *input);

#endif