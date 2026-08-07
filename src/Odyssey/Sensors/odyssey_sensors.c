#include "odyssey_sensors.h"

int update_odyssey_sensors(void) {
    update_ahrs_sensors();
    update_depth_sensor();
    update_water_sensors();
    update_usbl_sensor();
    handle_modem();
    update_lbl_sensor();

    return 0;
}