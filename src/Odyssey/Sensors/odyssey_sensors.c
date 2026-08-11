#include "odyssey_sensors.h"
#include "vehicle.h"

int update_odyssey_sensors(void) {
    update_ahrs_sensors();
    update_depth_sensor();
    update_water_sensors();
    update_usbl_sensor();
    vvp->modem_send[0] = '\0';
    update_lbl_sensor();

    return 0;
}