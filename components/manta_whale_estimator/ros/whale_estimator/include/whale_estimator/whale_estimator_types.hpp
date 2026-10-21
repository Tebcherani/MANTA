#ifndef MANTA_WHALE_ESTIMATOR_TYPES_HPP
#define MANTA_WHALE_ESTIMATOR_TYPES_HPP

#define WHALE_VELOCITY_WINDOW_SIZE 5

/* An exact position fix leaves only velocity variance after each update. */
typedef struct {
    double position;
    double velocity;
    double velocity_variance;
} whale_estimate_one_axis_t;

typedef struct {
    double time;
    double north;
    double east;
} whale_bearing_position_t;

typedef struct {
    double velocity_window_max_age_s;
    double acceleration_psd_m2_s3;
    double velocity_return_time_s;
    double nominal_north_velocity_mps;
    double nominal_east_velocity_mps;
    double initial_velocity_variance_m2ps2;
} whale_estimator_config_t;

#endif /* MANTA_WHALE_ESTIMATOR_TYPES_HPP */
