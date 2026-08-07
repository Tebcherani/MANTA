#include "common_defs.h"
#include "ody_func.h"
#include "ody_defs.h"
#include "odyssey_sensors.h"
#include "vehicle.h"
#include "odyssey_data.h"

// Direct-copy path: simulated inclinometer -> roll/pitch, simulated compass -> yaw
 void update_heading(void) {
    double flux_yaw = PI + modPi(sensor(M_COMPASS_YAW) + sensor(UO_MAGNETIC_VARIATION) - PI);
    update(M_ROLL, sensor(M_INCLINO_ROLL));
    update(M_PITCH, sensor(M_INCLINO_PITCH));
    update(M_YAW, flux_yaw);
}

int update_odyssey_sensors(void)
{
    // Preserve the original sensor update order
    init_rotation();
    load_angular_rate();
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