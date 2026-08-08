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

static int init_odyssey_config(odyssey_config_t *config);

static int init_odyssey_config(odyssey_config_t *config) {
    if (config == NULL) {
        write_to_sim_log("[odyssey_sim] Directory config is NULL.\n");
        return 0;
    }

    config->data_dir = getenv("ODYSSEY_DATA_DIR");
    if (config->data_dir == NULL || config->data_dir[0] == '\0') {
        write_to_sim_log("[odyssey_sim] ODYSSEY_DATA_DIR is not set.\n");
        return 0;
    }

    config->mission_dir = getenv("ODYSSEY_MISSION_DIR");
    if (config->mission_dir == NULL || config->mission_dir[0] == '\0') {
        write_to_sim_log("[odyssey_sim] ODYSSEY_MISSION_DIR is not set.\n");
        return 0;
    }

    config->fault_dir = getenv("ODYSSEY_FAULT_DIR");
    if (config->fault_dir == NULL || config->fault_dir[0] == '\0') {
        write_to_sim_log("[odyssey_sim] ODYSSEY_FAULT_DIR is not set.\n");
        return 0;
    }

    config->model_path = getenv("VEHICLE_PATH");
    if (config->model_path == NULL || config->model_path[0] == '\0') {
        write_to_sim_log("[odyssey_sim] VEHICLE_PATH is not set.\n");
        return 0;
    }

    write_to_sim_log("[odyssey_sim] Ody configuration complete.\n");
    return 1;
}

int init_odyssey_sim(odyssey_config_t *config, const command_line_options_t *options) {
    if (!init_odyssey_config(config)) return 0;
    if (!init_odyssey_data_structure()) return 0;
    if (!init_odyssey_model(config->model_path)) return 0;
    if (!init_odyssey_mission(options->mission_file, config->mission_dir)) return 0;

    return 1;
}

int update_odyssey_sim_cycle(void) {
    if (update_odyssey_sensors() != 0) return -1;
    if (update_odyssey_nav() != 0) return -1;
    if (update_odyssey_control() != 0) return -1;
    return update_odyssey_mission();
}