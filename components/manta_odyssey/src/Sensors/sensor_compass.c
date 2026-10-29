#include <math.h>
#include <stddef.h>

#include "sensor_compass.h"
#include "random_utils.h"
#include "sim_log.h"

int init_compass_state(compass_state_t *compass) {
    const double mass_kg = 0.1;
    const double spring_constant_Npm = 100.0;
    const double damping_kgps = 0.5;

    if (compass == NULL) {
        write_to_sim_log("[sensor_compass] Failed to initialize compass state: compass_state_t pointer is NULL.\n");
        return 0;
    }

    compass->displacement_x_m = 0.0;
    compass->displacement_y_m = 0.0;
    compass->displacement_rate_x_mps = 0.0;
    compass->displacement_rate_y_mps = 0.0;
    compass->damping_rate_per_s = damping_kgps / (2.0 * mass_kg);
    compass->natural_freq_radps = sqrt(spring_constant_Npm / mass_kg);
    compass->damped_freq_radps = sqrt(compass->natural_freq_radps * compass->natural_freq_radps - compass->damping_rate_per_s * compass->damping_rate_per_s);

    write_to_sim_log("[sensor_compass] Compass state initialized.\n");
    return 1;
}

double update_compass_yaw(compass_state_t *compass, compass_input_t *input) {
    const double compass_length_m = 0.015;
    const double compass_noise_rad = 0.001745; /* 0.1 degree in radians */

    // Orientation of the suspended compass mechanism
    double compass_yaw_rad;
    double compass_pitch_rad;
    double compass_roll_rad;

    // Magnetic field measured in the compass frame
    double magnetic_x_gauss;
    double magnetic_y_gauss;

    // Rotation matrix
    double rotation[4][4];

    if (compass == NULL) {
        write_to_sim_log("[sensor_compass] Failed to update compass state: compass_state_t pointer is NULL.\n");
        return 0.0;
    }

    double cosine = cos(compass->damped_freq_radps * input->timestep_s);
    double sine = sin(compass->damped_freq_radps * input->timestep_s);

    // x-direction
    double cos_coeff = input->accel_x_mps2 / (compass->natural_freq_radps * compass->natural_freq_radps) + compass->displacement_x_m;
    double sin_coeff = (compass->displacement_rate_x_mps + compass->damping_rate_per_s * (input->accel_x_mps2 / (compass->natural_freq_radps * compass->natural_freq_radps) + compass->displacement_x_m)) / compass->damped_freq_radps;

    compass->displacement_x_m = exp(-compass->damping_rate_per_s * input->timestep_s) * (cos_coeff * cosine + sin_coeff * sine) - input->accel_x_mps2 / (compass->natural_freq_radps * compass->natural_freq_radps);
    compass->displacement_rate_x_mps = -compass->damping_rate_per_s * exp(-compass->damping_rate_per_s * input->timestep_s) * (cos_coeff * cosine + sin_coeff * sine) + exp(-compass->damping_rate_per_s * input->timestep_s) * (-cos_coeff * compass->damped_freq_radps * sine + sin_coeff * compass->damped_freq_radps * cosine);

    // y-direction
    cos_coeff = input->accel_y_mps2 / (compass->natural_freq_radps * compass->natural_freq_radps) + compass->displacement_y_m;
    sin_coeff = (compass->displacement_rate_y_mps + compass->damping_rate_per_s * (input->accel_y_mps2 / (compass->natural_freq_radps * compass->natural_freq_radps) + compass->displacement_y_m)) / compass->damped_freq_radps;

    compass->displacement_y_m = exp(-compass->damping_rate_per_s * input->timestep_s) * (cos_coeff * cosine + sin_coeff * sine) - input->accel_y_mps2 / (compass->natural_freq_radps * compass->natural_freq_radps);
    compass->displacement_rate_y_mps = -compass->damping_rate_per_s * exp(-compass->damping_rate_per_s * input->timestep_s) * (cos_coeff * cosine + sin_coeff * sine) + exp(-compass->damping_rate_per_s * input->timestep_s) * (-cos_coeff * compass->damped_freq_radps * sine + sin_coeff * compass->damped_freq_radps * cosine);

    compass_pitch_rad = compass->displacement_x_m / compass_length_m;
    compass_roll_rad = -compass->displacement_y_m / compass_length_m;

    // Populate rotation matrix
    rotation[1][1] = cos(input->true_yaw_rad) * cos(compass_pitch_rad);
    rotation[1][2] = cos(compass_pitch_rad) * sin(input->true_yaw_rad);
    rotation[1][3] = -sin(compass_pitch_rad);

    rotation[2][1] = -cos(compass_roll_rad) * sin(input->true_yaw_rad) + cos(input->true_yaw_rad) * sin(compass_roll_rad) * sin(compass_pitch_rad);
    rotation[2][2] = cos(compass_roll_rad) * cos(input->true_yaw_rad) + sin(compass_roll_rad) * sin(input->true_yaw_rad) * sin(compass_pitch_rad);
    rotation[2][3] = cos(compass_pitch_rad) * sin(compass_roll_rad);

    rotation[3][1] = sin(compass_roll_rad) * sin(input->true_yaw_rad) + cos(compass_roll_rad) * cos(input->true_yaw_rad) * sin(compass_pitch_rad);
    rotation[3][2] = -cos(input->true_yaw_rad) * sin(compass_roll_rad) + cos(compass_roll_rad) * sin(input->true_yaw_rad) * sin(compass_pitch_rad);
    rotation[3][3] = cos(compass_roll_rad) * cos(compass_pitch_rad);

    magnetic_x_gauss = rotation[1][1] * input->magnetic_north_gauss + rotation[1][2] * input->magnetic_east_gauss + rotation[1][3] * input->magnetic_down_gauss;
    magnetic_y_gauss = rotation[2][1] * input->magnetic_north_gauss + rotation[2][2] * input->magnetic_east_gauss + rotation[2][3] * input->magnetic_down_gauss;

    // Calculate compass yaw from magnetic field
    if ((magnetic_x_gauss > 0.0) && (magnetic_y_gauss <= 0.0)) compass_yaw_rad = -atan(magnetic_y_gauss / magnetic_x_gauss);
    else if ((magnetic_x_gauss <= 0.0) && (magnetic_y_gauss < 0.0)) compass_yaw_rad = atan(magnetic_x_gauss / magnetic_y_gauss) + M_PI / 2.0;
    else if ((magnetic_x_gauss < 0.0) && (magnetic_y_gauss >= 0.0)) compass_yaw_rad = -atan(magnetic_y_gauss / magnetic_x_gauss) + M_PI;
    else if ((magnetic_x_gauss >= 0.0) && (magnetic_y_gauss > 0.0)) compass_yaw_rad = atan(magnetic_x_gauss / magnetic_y_gauss) + 3.0 * M_PI / 2.0;
    else compass_yaw_rad = 0.0;

    compass_yaw_rad += compass_noise_rad * std_normal_random_num();

    return compass_yaw_rad;
}