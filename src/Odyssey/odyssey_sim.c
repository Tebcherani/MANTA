#include "odyssey_sim.h"

#include "odyssey_control.h"
#include "odyssey_nav.h"
#include "odyssey_sensors.h"

int update_odyssey_cycle(void) {
    if (update_odyssey_sensors() != 0) return -1;
    if (update_odyssey_nav() != 0) return -1;
    if (update_odyssey_control() != 0) return -1;

    return 0;
}