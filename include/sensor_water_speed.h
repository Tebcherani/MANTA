#ifndef SENSOR_DEPTH_H
#define SENSOR_DEPTH_H

typedef struct {
    double true_speed;
    double noise_std_dev;
} water_speed_input_t;

double update_water_speed(const water_speed_input_t *);

#endif