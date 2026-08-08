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

static int init_odyssey_dirs(odyssey_dirs_t *dirs);

static int init_odyssey_dirs(odyssey_dirs_t *dirs) {
    if (dirs == NULL) {
        write_to_sim_log("[odyssey_sim] Directory config is NULL.\n");
        return 0;
    }

    dirs->data_dir = getenv("ODYSSEY_DATA_DIR");
    if (dirs->data_dir == NULL || dirs->data_dir[0] == '\0') {
        write_to_sim_log("[odyssey_sim] ODYSSEY_DATA_DIR is not set.\n");
        return 0;
    }

    dirs->mission_dir = getenv("ODYSSEY_MISSION_DIR");
    if (dirs->mission_dir == NULL || dirs->mission_dir[0] == '\0') {
        write_to_sim_log("[odyssey_sim] ODYSSEY_MISSION_DIR is not set.\n");
        return 0;
    }

    dirs->fault_dir = getenv("ODYSSEY_FAULT_DIR");
    if (dirs->fault_dir == NULL || dirs->fault_dir[0] == '\0') {
        write_to_sim_log("[odyssey_sim] ODYSSEY_FAULT_DIR is not set.\n");
        return 0;
    }

    write_to_sim_log("[odyssey_sim] Odyssey directories configured.\n");
    return 1;
}

int init_odyssey_sim(odyssey_dirs_t *dirs) {
    if (!init_odyssey_dirs(dirs)) return 0;
    if (!init_data_structure()) return 0;

    return 1;
}

int update_odyssey_sim_cycle(void) {
    if (update_odyssey_sensors() != 0) return -1;
    if (update_odyssey_nav() != 0) return -1;
    if (update_odyssey_control() != 0) return -1;
    return update_odyssey_mission();
}