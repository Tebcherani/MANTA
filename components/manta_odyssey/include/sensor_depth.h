#ifndef SENSOR_DEPTH_H
#define SENSOR_DEPTH_H

typedef struct {
    double true_depth_m;
    double noise_std_dev_m;
} depth_input_t;

double update_depth(const depth_input_t *input);

#endif