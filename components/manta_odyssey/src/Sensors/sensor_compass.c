#include <math.h>
#include <stddef.h>

#include "sensor_compass.h"
#include "random_utils.h"
#include "sim_log.h"

int init_compass_state(compass_state_t *compass) {
    const double mass = 0.1;
    const double spring_constant = 100.0;
    const double damping = 0.5;

    if (compass == NULL) {
        write_to_sim_log("[sensor_compass] Failed to initialize compass state: compass_state_t pointer is NULL.\n");
        return 0;
    }

    compass->displacement_x_m = 0.0;
    compass->displacement_y_m = 0.0;
    compass->displacement_rate_x_mps = 0.0;
    compass->displacement_rate_y_mps = 0.0;
    compass->damping_rate = damping / (2.0 * mass);
    compass->natural_freq = sqrt(spring_constant / mass);
    compass->damped_freq = sqrt(compass->natural_freq * compass->natural_freq - compass->damping_rate * compass->damping_rate);

    write_to_sim_log("[sensor_compass] Compass state initialized.\n");
    return 1;
}

double update_compass_yaw(compass_state_t *compass, compass_input_t *input) {
    const double compass_length = 0.015;
    const double compass_noise = 0.001745; /* 0.1 degree in radians */

    // Orientation of the suspended compass mechanism
    double compass_yaw;
    double compass_pitch;
    double compass_roll;

    // Magnetic field measured in the compass frame
    double magnetic_x;
    double magnetic_y;

    // Rotation matrix
    double rotation[4][4];

    if (compass == NULL) {
        write_to_sim_log("[sensor_compass] Failed to update compass state: compass_state_t pointer is NULL.\n");
        return 0.0;
    }

    double cosine = cos(compass->damped_freq * input->timestep_s);
    double sine = sin(compass->damped_freq * input->timestep_s);

    // x-direction
    double cos_coeff = input->accel_x_mps2 / (compass->natural_freq * compass->natural_freq) + compass->displacement_x_m;
    double sin_coeff = (compass->displacement_rate_x_mps + compass->damping_rate * (input->accel_x_mps2 / (compass->natural_freq * compass->natural_freq) + compass->displacement_x_m)) / compass->damped_freq;

    compass->displacement_x_m = exp(-compass->damping_rate * input->timestep_s) * (cos_coeff * cosine + sin_coeff * sine) - input->accel_x_mps2 / (compass->natural_freq * compass->natural_freq);
    compass->displacement_rate_x_mps = -compass->damping_rate * exp(-compass->damping_rate * input->timestep_s) * (cos_coeff * cosine + sin_coeff * sine) + exp(-compass->damping_rate * input->timestep_s) * (-cos_coeff * compass->damped_freq * sine + sin_coeff * compass->damped_freq * cosine);

    // y-direction
    cos_coeff = input->accel_y_mps2 / (compass->natural_freq * compass->natural_freq) + compass->displacement_y_m;
    sin_coeff = (compass->displacement_rate_y_mps + compass->damping_rate * (input->accel_y_mps2 / (compass->natural_freq * compass->natural_freq) + compass->displacement_y_m)) / compass->damped_freq;

    compass->displacement_y_m = exp(-compass->damping_rate * input->timestep_s) * (cos_coeff * cosine + sin_coeff * sine) - input->accel_y_mps2 / (compass->natural_freq * compass->natural_freq);
    compass->displacement_rate_y_mps = -compass->damping_rate * exp(-compass->damping_rate * input->timestep_s) * (cos_coeff * cosine + sin_coeff * sine) + exp(-compass->damping_rate * input->timestep_s) * (-cos_coeff * compass->damped_freq * sine + sin_coeff * compass->damped_freq * cosine);

    compass_pitch = compass->displacement_x_m / compass_length;
    compass_roll = -compass->displacement_y_m / compass_length;

    // Populate rotation matrix
    rotation[1][1] = cos(input->true_yaw_rad) * cos(compass_pitch);
    rotation[1][2] = cos(compass_pitch) * sin(input->true_yaw_rad);
    rotation[1][3] = -sin(compass_pitch);

    rotation[2][1] = -cos(compass_roll) * sin(input->true_yaw_rad) + cos(input->true_yaw_rad) * sin(compass_roll) * sin(compass_pitch);
    rotation[2][2] = cos(compass_roll) * cos(input->true_yaw_rad) + sin(compass_roll) * sin(input->true_yaw_rad) * sin(compass_pitch);
    rotation[2][3] = cos(compass_pitch) * sin(compass_roll);

    rotation[3][1] = sin(compass_roll) * sin(input->true_yaw_rad) + cos(compass_roll) * cos(input->true_yaw_rad) * sin(compass_pitch);
    rotation[3][2] = -cos(input->true_yaw_rad) * sin(compass_roll) + cos(compass_roll) * sin(input->true_yaw_rad) * sin(compass_pitch);
    rotation[3][3] = cos(compass_roll) * cos(compass_pitch);

    magnetic_x = rotation[1][1] * input->magnetic_north + rotation[1][2] * input->magnetic_east + rotation[1][3] * input->magnetic_down;
    magnetic_y = rotation[2][1] * input->magnetic_north + rotation[2][2] * input->magnetic_east + rotation[2][3] * input->magnetic_down;

    // Calculate compass yaw from magnetic field
    if ((magnetic_x > 0.0) && (magnetic_y <= 0.0)) compass_yaw = -atan(magnetic_y / magnetic_x);
    else if ((magnetic_x <= 0.0) && (magnetic_y < 0.0)) compass_yaw = atan(magnetic_x / magnetic_y) + M_PI / 2.0;
    else if ((magnetic_x < 0.0) && (magnetic_y >= 0.0)) compass_yaw = -atan(magnetic_y / magnetic_x) + M_PI;
    else if ((magnetic_x >= 0.0) && (magnetic_y > 0.0)) compass_yaw = atan(magnetic_x / magnetic_y) + 3.0 * M_PI / 2.0;
    else compass_yaw = 0.0;

    compass_yaw += compass_noise * std_normal_random_num();

    return compass_yaw;
}