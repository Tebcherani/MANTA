#include <stdlib.h>

#include "load_fault.h"
#include "odyssey.h"
#include "odyssey_model.h"
#include "odyssey_sensors.h"
#include "odyssey_sim.h"
#include "sim_log.h"

#include "vehicle.h"

static int init_odyssey_config(void);

static odyssey_config_t config;

static int init_odyssey_config(void) {
    config.data_dir = getenv("ODYSSEY_DATA_DIR");
    if (config.data_dir == NULL || config.data_dir[0] == '\0') {
        write_to_sim_log("[odyssey_sim] ODYSSEY_DATA_DIR is not set.\n");
        return 0;
    }

    config.mission_path = getenv("MISSION_PATH");
    if (config.mission_path == NULL || config.mission_path[0] == '\0') {
        write_to_sim_log("[odyssey_sim] MISSION_PATH is not set.\n");
        return 0;
    }

    config.fault_path = getenv("FAULT_PATH");
    if (config.fault_path == NULL) {
        config.fault_path = "";
        write_to_sim_log("[odyssey_sim] FAULT_PATH is not set.\n");
        return 0;
    }

    config.vehicle_id = getenv("ODYSSEY_VEHICLE_ID");
    if (config.vehicle_id == NULL || config.vehicle_id[0] == '\0') {
        write_to_sim_log("[odyssey_sim] ODYSSEY_VEHICLE_ID is not set.\n");
        return 0;
    }

    config.model_path = getenv("VEHICLE_PATH");
    if (config.model_path == NULL || config.model_path[0] == '\0') {
        write_to_sim_log("[odyssey_sim] VEHICLE_PATH is not set.\n");
        return 0;
    }

    write_to_sim_log("[odyssey_sim] Odyssey configuration complete.\n");
    return 1;
}

int init_odyssey_sim() {
    if (!init_odyssey_config()) return 0;
    if (!init_odyssey()) return 0;
    if (!init_odyssey_sensors()) return 0;

    return 1;
}

int start_odyssey_sim(time_t calendar_start_time) {
    if (!start_odyssey(config.vehicle_id, config.mission_path, config.fault_path, config.data_dir, calendar_start_time)) return 0;
    if (!init_odyssey_model(config.model_path)) return 0;
    if (!load_faults(config.fault_path)) return 0;
    if (!init_odyssey_state()) return 0;
    if (!init_odyssey_dynamics()) return 0;

    return 1;
}

int update_odyssey_sim_state(const world_state_t *world_state, double timestep) {
    if (update_odyssey_state(world_state, timestep) != 0) return 0;

    return 1;
}

odyssey_update_status_t update_odyssey_sim_cycle(const world_state_t *world_state, double timestep) {
    odyssey_environment_t environment;
    odyssey_update_status_t status;

    environment.seawater_density = world_state->seawater_density;

    if (!update_odyssey_sensors(world_state, timestep)) return ODYSSEY_UPDATE_ERROR;        
    return update_odyssey(world_state->seawater_density, timestep);
}

int update_odyssey_sim_time(double sim_time) {
    update_sensor(M_PRESENT_TIME, sim_time);
    sensor_time(M_PRESENT_TIME) = (long)(1000.0 * sim_time);

    return 1;
}

int end_odyssey_sim(const char *behavior_dump_file) {
    return end_odyssey(behavior_dump_file, config.data_dir);
}