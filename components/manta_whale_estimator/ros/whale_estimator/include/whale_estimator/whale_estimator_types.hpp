#ifndef MANTA_WHALE_ESTIMATOR_TYPES_HPP
#define MANTA_WHALE_ESTIMATOR_TYPES_HPP

#define WHALE_VELOCITY_WINDOW_SIZE 5

/* Position, velocity, and their covariance for one north/east filter axis. */
typedef struct {
    double position;
    double velocity;
    double position_variance;
    double position_velocity_covariance;
    double velocity_variance;
} whale_estimate_one_axis_t;

typedef struct {
    double time;
    double north;
    double east;
    double north_variance;
    double east_variance;
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
