#include <math.h>
#include <stdlib.h>

#include "odyssey_control.h"
#include "odyssey_data_struct.h"
#include "odyssey_model.h"
#include "odyssey_mission.h"
#include "odyssey_nav.h"
#include "odyssey_sensors.h"
#include "odyssey_sim.h"
#include "sim_log.h"
#include "vehicle.h"

const char *ody_data_dir;
const char *ody_mission_dir;
const char *ody_sfault_dir;
const char *ody_vehicle_dir;

int init_odyssey_sim(void) {
    ody_data_dir = getenv("ODYSSEY_DATA_DIR");
    if (ody_data_dir == NULL || ody_data_dir[0] == '\0') {
        write_to_sim_log("[odyssey_sim] ODYSSEY_DATA_DIR is not set.\n");
        return 0;
    }

    ody_mission_dir = getenv("ODYSSEY_MISSION_DIR");
    if (ody_mission_dir == NULL || ody_mission_dir[0] == '\0') {
        write_to_sim_log("[odyssey_sim] ODYSSEY_MISSION_DIR is not set.\n");
        return 0;
    }

    ody_sfault_dir = getenv("ODYSSEY_FAULT_DIR");
    if (ody_sfault_dir == NULL || ody_sfault_dir[0] == '\0') {
        write_to_sim_log("[odyssey_sim] ODYSSEY_FAULT_DIR is not set.\n");
        return 0;
    }

    write_to_sim_log("[odyssey_sim] Ody configured.\n");
    return 1;
}

int update_odyssey_sim_cycle(void) {
    if (update_odyssey_sensors() != 0) return -1;
    if (update_odyssey_nav() != 0) return -1;
    if (update_odyssey_control() != 0) return -1;
    return update_odyssey_mission();
}