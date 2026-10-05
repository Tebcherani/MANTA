#ifndef SENSOR_DEPTH_H
#define SENSOR_DEPTH_H

typedef struct {
    double true_depth;
    double noise_std_dev;
} depth_input_t;

double update_depth(const depth_input_t *input);

#endif