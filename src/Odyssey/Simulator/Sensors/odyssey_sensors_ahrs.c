#include <math.h>

#include "angle_utils.h"
#include "odyssey.h"
#include "odyssey_sensors.h"
#include "sensor_accelerometer.h"
#include "sensor_compass.h"
#include "sensor_gyro.h"
#include "sim_log.h"
#include "world_sim.h"

#include "vehicle.h"

static void calculate_compass_acceleration(const accelerometer_measurement_t *, const world_state_t *, double *, double *);
static void update_heading(void);
static accelerometer_result_t update_odyssey_accelerometer(const world_state_t *);
static void update_odyssey_compass(const world_state_t *, double, double, double);
static int update_odyssey_gyro(void);
static int update_odyssey_inclinometer(const world_state_t *, const accelerometer_measurement_t *);

static void calculate_compass_acceleration(const accelerometer_measurement_t *accelerometer, const world_state_t *world_state, double *compass_accel_x, double *compass_accel_y) {
    double body_accel_x, body_accel_y, body_accel_z;
    double pitch, roll;

    pitch = read_sensor(REAL_THETA);
    roll = read_sensor(REAL_PHI);

    // Remove gravity from the accelerometer measurement to recover the non-gravitational acceleration at the AHRS mounting point.
    body_accel_x = accelerometer->accel_x - sin(pitch) * world_state->gravity;
    body_accel_y = accelerometer->accel_y + cos(pitch) * sin(roll) * world_state->gravity;
    body_accel_z = accelerometer->accel_z + cos(pitch) * cos(roll) * world_state->gravity;

    // Transform the non-gravitational acceleration from the vehicle body frame into the local horizontal frame used by the suspended compass model.
    *compass_accel_x = cos(pitch) * body_accel_x + sin(roll) * sin(pitch) * body_accel_y + cos(roll) * sin(pitch) * body_accel_z;
    *compass_accel_y = cos(roll) * body_accel_y - sin(roll) * body_accel_z;
}

static accelerometer_result_t update_odyssey_accelerometer(const world_state_t *world_state) {
    true_vehicle_motion_t vehicle;
    accelerometer_config_t config;

    // Odyssey AHRS mounting location relative to the vehicle reference point
    config.mount_x = 1.390;
    config.mount_y = 0.108;
    config.mount_z = 0.076;

    vehicle.velocity_x = read_sensor(REAL_U);
    vehicle.velocity_y = read_sensor(REAL_V);
    vehicle.velocity_z = read_sensor(REAL_W);

    vehicle.acceleration_x = read_sensor(REAL_UD);
    vehicle.acceleration_y = read_sensor(REAL_VD);
    vehicle.acceleration_z = read_sensor(REAL_WD);

    vehicle.angular_rate_x = read_sensor(REAL_P);
    vehicle.angular_rate_y = read_sensor(REAL_Q);
    vehicle.angular_rate_z = read_sensor(REAL_R);

    vehicle.angular_acceleration_x = read_sensor(REAL_PD);
    vehicle.angular_acceleration_y = read_sensor(REAL_QD);
    vehicle.angular_acceleration_z = read_sensor(REAL_RD);

    vehicle.pitch = read_sensor(REAL_THETA);
    vehicle.roll = read_sensor(REAL_PHI);

    vehicle.gravity = world_state->gravity;

    return update_accelerometer(&config, &vehicle);
}

static void update_odyssey_compass(const world_state_t *world_state, double accel_x, double accel_y, double timestep) {
    compass_state_t *compass;
    double compass_yaw;
    compass_input_t input;

    compass = get_odyssey_compass_state();

    input.true_yaw = read_sensor(REAL_PSI);
    input.accel_x = accel_x;
    input.accel_y = accel_y;
    input.magnetic_north = world_state->magnetic_north;
    input.magnetic_east = world_state->magnetic_east;
    input.magnetic_down = world_state->magnetic_down;
    input.timestep = timestep;

    compass_yaw = update_compass_yaw(compass, &input);
    compass_yaw -= read_sensor(UO_MAGNETIC_VARIATION);
    update_sensor(M_COMPASS_YAW, compass_yaw);
}

static int update_odyssey_gyro(void) {
    gyro_config_t config;
    gyro_input_t input;
    gyro_result_t result;

    config.roll_bias = 4.3663e-4;
    config.pitch_bias = 8.7266e-4;
    config.yaw_bias = -8.7266e-4;

    config.noise_std_dev = 0.00001745;

    input.roll_rate = read_sensor(REAL_P);
    input.pitch_rate = read_sensor(REAL_Q);
    input.yaw_rate = read_sensor(REAL_R);

    result = update_gyro(&config, &input);

    if (!result.success) {
        write_to_sim_log("[odyssey_sensors_ahrs] Failed to update gyro.\n");
        return 0;
    }

    update_sensor(M_ROLL_RATE, result.measurement.roll_rate);
    update_sensor(M_PITCH_RATE, result.measurement.pitch_rate);
    update_sensor(M_YAW_RATE, result.measurement.yaw_rate);

    return 1;
}

static void update_heading(void) {
    double meas_yaw, meas_pitch, meas_roll;
    double meas_magnetic_variation, corrected_yaw;

    meas_yaw = read_sensor(M_COMPASS_YAW);
    meas_magnetic_variation = read_sensor(UO_MAGNETIC_VARIATION);
    corrected_yaw = M_PI + wrap_angle(meas_yaw + meas_magnetic_variation - M_PI);
    update_sensor(M_YAW, corrected_yaw);

    meas_pitch = read_sensor(M_INCLINO_PITCH);
    update_sensor(M_PITCH, meas_pitch);
    
    meas_roll = read_sensor(M_INCLINO_ROLL);
    update_sensor(M_ROLL, meas_roll);
}

static int update_odyssey_inclinometer(const world_state_t *world_state, const accelerometer_measurement_t *accelerometer) {
    inclinometer_state_t *state;
    inclinometer_input_t input;
    inclinometer_result_t result;

    state = get_odyssey_inclinometer_state();

    input.accel_x = accelerometer->accel_x;
    input.accel_y = accelerometer->accel_y;
    input.gravity = world_state->gravity;

    result = update_inclinometer(state, &input);

    if (!result.success) {
        write_to_sim_log("[odyssey_sensors_ahrs] Failed to update inclinometer.\n");
        return 0;
    }

    update_sensor(M_INCLINO_PITCH, result.measurement.pitch);
    update_sensor(M_INCLINO_ROLL, result.measurement.roll);

    return 1;
}

void update_ahrs_sensors(const world_state_t *world_state, double timestep) {
    double compass_accel_x, compass_accel_y;
    accelerometer_result_t accelerometer;

    update_odyssey_gyro();
    accelerometer = update_odyssey_accelerometer(world_state);
    update_odyssey_inclinometer(world_state, &accelerometer.measurement);
    calculate_compass_acceleration(&accelerometer.measurement, world_state, &compass_accel_x, &compass_accel_y);
    update_odyssey_compass(world_state, compass_accel_x, compass_accel_y, timestep);
    update_heading();
}