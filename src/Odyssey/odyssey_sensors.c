#include <math.h>

#include "common_defs.h"
#include "ody_func.h"
#include "ody_defs.h"
#include "odyssey_sensors.h"
#include "vehicle.h"
#include "odyssey_data.h"

int update_odyssey_sensors(void) {
    update_angular_rate();
    load_accel();
    load_magnetic();
    load_depth();
    load_water_speed();
    load_water_param();

#ifdef WHOI_USBL
    simulate_usbl();
#endif

    handle_modem();
    update_heading();
    simLBL();

    return 0;
}

double gasdev();

void update_angular_rate(void) {
    double real_yaw_rate, real_roll_rate, real_pitch_rate;
    double meas_yaw_rate, meas_roll_rate, meas_pitch_rate;
    double yaw_rate_bias = -8.7266e-4, pitch_rate_bias = 8.7266e-4, roll_rate_bias = 4.3663e-4;
    double sensor_noise, sensor_noise_level = 0.00001745;

    real_roll_rate = sensor(REAL_P);
    sensor_noise = sensor_noise_level * gasdev(); 
    meas_roll_rate = real_roll_rate + sensor_noise + roll_rate_bias;
    update(M_ROLL_RATE, meas_roll_rate);

    real_pitch_rate = sensor(REAL_Q);
    sensor_noise = sensor_noise_level * gasdev(); 
    meas_pitch_rate = real_pitch_rate + sensor_noise + pitch_rate_bias;
    update(M_PITCH_RATE, meas_pitch_rate);

    real_yaw_rate = sensor(REAL_R);
    sensor_noise = sensor_noise_level * gasdev(); 
    meas_yaw_rate = real_yaw_rate + sensor_noise + yaw_rate_bias;
    update(M_YAW_RATE, meas_yaw_rate);
}

void update_heading(void) {
    double meas_yaw, meas_pitch, meas_roll;
    double meas_magnetic_variation, corrected_yaw;

    meas_yaw = sensor(M_COMPASS_YAW);
    meas_magnetic_variation = sensor(UO_MAGNETIC_VARIATION);
    corrected_yaw = PI + modPi(meas_yaw + meas_magnetic_variation - PI);
    update(M_YAW, corrected_yaw);

    meas_pitch = sensor(M_INCLINO_PITCH);
    update(M_PITCH, meas_pitch);
    
    meas_roll = sensor(M_INCLINO_ROLL);
    update(M_ROLL, meas_roll);
}