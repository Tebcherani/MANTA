#include <math.h>

#include "angle_utils.h"
#include "common_defs.h"
#include "ody_func.h"
#include "ody_defs.h"
#include "odyssey_data.h"
#include "odyssey_sensors.h"
#include "vehicle.h"

static void update_heading(void);

int update_odyssey_sensors(void) {
    update_ahrs_sensors();
    update_depth_sensor();
    update_water_sensors();
    simulate_usbl();
    handle_modem();
    update_heading();
    simLBL();

    return 0;
}

static void update_heading(void) {
    double meas_yaw, meas_pitch, meas_roll;
    double meas_magnetic_variation, corrected_yaw;

    meas_yaw = sensor(M_COMPASS_YAW);
    meas_magnetic_variation = sensor(UO_MAGNETIC_VARIATION);
    corrected_yaw = PI + wrap_angle(meas_yaw + meas_magnetic_variation - PI);
    update(M_YAW, corrected_yaw);

    meas_pitch = sensor(M_INCLINO_PITCH);
    update(M_PITCH, meas_pitch);
    
    meas_roll = sensor(M_INCLINO_ROLL);
    update(M_ROLL, meas_roll);
}