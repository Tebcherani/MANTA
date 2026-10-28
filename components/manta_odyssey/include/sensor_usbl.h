#ifndef SENSOR_USBL_H
#define SENSOR_USBL_H

typedef struct {
    double hit_interval_s;    // How often the USBL produces a measurement
    double hit_window_s;      // Allowed timing window for a hit
    double min_direction_cos;   // Min directional cosine for the sensor field of view
} usbl_config_t;

typedef struct {
    // True vehicle position
    double vehicle_north_m;
    double vehicle_east_m;
    double vehicle_down_m;
    double vehicle_yaw_rad;

    // Position of the USBL target/transponder
    double target_north_m;
    double target_east_m;
    double target_down_m;

    double sim_time_s;
} usbl_input_t;

typedef struct {
    double range_m;
    double azimuth_rad;
    double elevation_rad;
} usbl_measurement_t;

typedef struct {
    int success;
    int has_measurement;    // A valid update occured during this timestep
    usbl_measurement_t measurement;
} usbl_result_t;

usbl_result_t update_usbl(const usbl_config_t *config, const usbl_input_t *input);

#endif