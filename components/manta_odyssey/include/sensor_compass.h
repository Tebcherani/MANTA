#ifndef SENSOR_COMPASS_H
#define SENSOR_COMPASS_H

typedef struct {
    double true_yaw;
    double accel_x;
    double accel_y;
    double magnetic_north;
    double magnetic_east;
    double magnetic_down;
    double timestep;
} compass_input_t;

typedef struct {
    double displacement_x;
    double displacement_y;
    double displacement_rate_x;
    double displacement_rate_y;
    double damping_rate;
    double natural_freq;
    double damped_freq;
} compass_state_t;

int init_compass_state(compass_state_t *);
double update_compass_yaw(compass_state_t *, compass_input_t *);

#endif