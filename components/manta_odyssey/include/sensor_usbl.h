#ifndef SENSOR_USBL_H
#define SENSOR_USBL_H

typedef struct {
    double hit_interval;    // How often the USBL produces a measurement
    double hit_window;      // Allowed timing window for a hit
    double min_direction_cos;   // Min directional cosine for the sensor field of view
} usbl_config_t;

typedef struct {
    // True vehicle position
    double vehicle_north;
    double vehicle_east;
    double vehicle_down;
    double vehicle_yaw;

    // Position of the USBL target/transponder
    double target_north;
    double target_east;
    double target_down;

    double sim_time;
} usbl_input_t;

typedef struct {
    double range;
    double azimuth;
    double elevation;
} usbl_measurement_t;

typedef struct {
    int success;
    int has_measurement;    // A valid update occured during this timestep
    usbl_measurement_t measurement;
} usbl_result_t;

usbl_result_t update_usbl(const usbl_config_t *config, const usbl_input_t *input);

#endif