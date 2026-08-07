#include <math.h>
#include <stdio.h>

#include "angle_utils.h"
#include "odyssey_data_struct.h"
#include "vehicle.h"

int update_usbl_sensor(void) {
    double xpos, ypos, zpos;
    double range, azim, elev;
    double dock_x, dock_y, dock_z;
    double usbl_hit_time = 0.5;

    dock_x = read_sensor(SIM_DOCK_N);
    dock_y = read_sensor(SIM_DOCK_E);
    dock_z = read_sensor(SIM_DOCK_D);

    xpos = read_sensor(REAL_X);
    ypos = read_sensor(REAL_Y);
    zpos = read_sensor(REAL_Z);

    range = sqrt(pow(dock_x - xpos,2.0) + pow(dock_y - ypos,2.0) + pow(dock_z - zpos,2));
    azim = wrap_angle(atan2(dock_y - ypos, dock_x - xpos) - read_sensor(M_YAW));
    elev = atan2(dock_z - zpos, sqrt(pow(dock_x - xpos,2.0) + pow(dock_y - ypos,2.0)));

    if ((fmod(read_sensor(M_PRESENT_TIME),usbl_hit_time) < .02) && ((cos(elev) > 0.707) && (cos(azim) > 0.707))) {
	    update_sensor(M_USBL_RANGE, range);
	    update_sensor(M_USBL_AZIMUTH, azim);
	    update_sensor(M_USBL_ELEVATION, elev);
    }

    return 0;
}