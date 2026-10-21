#ifndef MANTA_WHALE_ESTIMATOR_TYPES_HPP
#define MANTA_WHALE_ESTIMATOR_TYPES_HPP

#define WHALE_VELOCITY_WINDOW_SIZE 5

/* An exact position fix leaves only velocity variance after each update. */
typedef struct {
    double position;
    double velocity;
    double velocity_variance;
} whale_axis_filter_t;

/* Positions use local north/east metres and times use simulated seconds. */
typedef struct {
    double time;
    double north;
    double east;
} whale_position_fix_t;

typedef struct {
    double velocity_window_max_age_s;
    double acceleration_psd_m2_s3;
    double velocity_return_time_s;
    double nominal_north_velocity_mps;
    double nominal_east_velocity_mps;
    double initial_velocity_variance_m2ps2;
} whale_estimator_config_t;

typedef struct {
    int available_b;
    double measurement_time_s;
    double north_m;
    double east_m;
    double velocity_north_mps;
    double velocity_east_mps;
} whale_prediction_t;

#endif /* MANTA_WHALE_ESTIMATOR_TYPES_HPP */
