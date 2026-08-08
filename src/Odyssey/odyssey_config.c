#include <stdlib.h>

#include "odyssey_config.h"
#include "sim_log.h"

const char *ody_data_dir;
const char *ody_mission_dir;
const char *ody_sfault_dir;
const char *ody_vehicle_dir;

int init_odyssey_config(void) {
    ody_data_dir = getenv("ODYSSEY_DATA_DIR");
    if (ody_data_dir == NULL || ody_data_dir[0] == '\0') {
        write_to_sim_log("[odyssey_config] ODYSSEY_DATA_DIR is not set.\n");
        return 0;
    }

    ody_mission_dir = getenv("ODYSSEY_MISSION_DIR");
    if (ody_mission_dir == NULL || ody_mission_dir[0] == '\0') {
        write_to_sim_log("[odyssey_config] ODYSSEY_MISSION_DIR is not set.\n");
        return 0;
    }

    ody_sfault_dir = getenv("ODYSSEY_FAULT_DIR");
    if (ody_sfault_dir == NULL || ody_sfault_dir[0] == '\0') {
        write_to_sim_log("[odyssey_config] ODYSSEY_FAULT_DIR is not set.\n");
        return 0;
    }

    write_to_sim_log("[odyssey_config] Ody configured.\n");
    return 1;
}