#include <math.h>

#include "odyssey_data_struct.h"
#include "odyssey_nav.h"
#include "vehicle.h"

void update_dead_reckoned_pos(void) {
    double ds, dn, de;

    ds = read_sensor(M_DT) * read_sensor(M_WATER_SPEED) * cos(read_sensor(M_PITCH));
    dn = ds * cos(read_sensor(M_YAW));
    de = ds * sin(read_sensor(M_YAW));
	
    // Store DR deltas for all to use
    update_sensor(M_DR_DN, dn);
    update_sensor(M_DR_DE, de);
	
    // Update DR position
    add_to_sensor(M_DR_NORTH, dn);
    add_to_sensor(M_DR_EAST, de);
}