#ifndef SENSOR_COMPASS_H
#define SENSOR_COMPASS_H

typedef struct {
    double true_yaw_rad;
    double accel_x_mps2;
    double accel_y_mps2;
    double magnetic_north;
    double magnetic_east;
    double magnetic_down;
    double timestep_s;
} compass_input_t;

typedef struct {
    double displacement_x_m;
    double displacement_y_m;
    double displacement_rate_x_mps;
    double displacement_rate_y_mps;
    double damping_rate_per_s;
    double natural_freq_radps;
    double damped_freq_radps;
} compass_state_t;

int init_compass_state(compass_state_t *);
double update_compass_yaw(compass_state_t *, compass_input_t *);

#endif