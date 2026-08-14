#include <stdlib.h>

#include "load_fault.h"
#include "odyssey.h"
#include "odyssey_model.h"
#include "odyssey_sensors.h"
#include "odyssey_sim.h"
#include "sim_log.h"

#include "odyssey_behavior_logger.h"
#include "odyssey_control.h"
#include "odyssey_data_logger.h"
#include "odyssey_mission.h"
#include "odyssey_nav.h"
#include "ody_var.h"
#include "vehicle.h"

static int init_odyssey_config(void);

static FILE *datafile = NULL;
static odyssey_config_t config;

static int init_odyssey_config(void) {
    config.data_dir = getenv("ODYSSEY_DATA_DIR");
    if (config.data_dir == NULL || config.data_dir[0] == '\0') {
        write_to_sim_log("[odyssey_sim] ODYSSEY_DATA_DIR is not set.\n");
        return 0;
    }

    config.mission_dir = getenv("ODYSSEY_MISSION_DIR");
    if (config.mission_dir == NULL || config.mission_dir[0] == '\0') {
        write_to_sim_log("[odyssey_sim] ODYSSEY_MISSION_DIR is not set.\n");
        return 0;
    }

    config.fault_dir = getenv("ODYSSEY_FAULT_DIR");
    if (config.fault_dir == NULL || config.fault_dir[0] == '\0') {
        write_to_sim_log("[odyssey_sim] ODYSSEY_FAULT_DIR is not set.\n");
        return 0;
    }

    config.model_path = getenv("VEHICLE_PATH");
    if (config.model_path == NULL || config.model_path[0] == '\0') {
        write_to_sim_log("[odyssey_sim] VEHICLE_PATH is not set.\n");
        return 0;
    }

    write_to_sim_log("[odyssey_sim] Ody configuration complete.\n");
    return 1;
}

int init_odyssey_sim() {
    if (!init_odyssey_config()) return 0;
    if (!init_odyssey()) return 0;

    return 1;
}

int start_odyssey_sim(const char *mission_file, const char *fault_file, time_t calendar_start_time) {
    if (!start_odyssey(mission_file, config.mission_dir, calendar_start_time)) return 0;
    if (!init_odyssey_model(config.model_path)) return 0;
    if (!load_faults(fault_file ? fault_file : vvp->fault_file_name, &config)) return 0;
    if (!init_odyssey_state()) return 0;
    if (!init_odyssey_data_logger(&datafile, &config, mission_file)) return 0;
    if (!init_odyssey_dynamics()) return 0;

    return 1;
}

int update_odyssey_sim_state(const world_state_t *world_state, double timestep) {
    if (update_odyssey_state(world_state, timestep) != 0) return 0;

    return 1;
}

int update_odyssey_sim_cycle(const world_state_t *world_state, double control_dt) {
    odyssey_environment_t environment;

    environment.seawater_density = world_state->seawater_density;

    if (!update_odyssey_sensors(world_state)) return 0;
    if (!update_odyssey(world_state->seawater_density, control_dt)) return 0;

    capture_behavior_activity();
	log_data(datafile);
        
    return 1;
}

int update_odyssey_sim_time(double sim_time) {
    update_sensor(M_PRESENT_TIME, sim_time);
    sensor_time(M_PRESENT_TIME) = (long)(1000.0 * sim_time);

    return 1;
}

int end_odyssey_sim(const char *behavior_dump_file) {
    int cleanup_failed = 0;

    if (behavior_dump_file != NULL && behavior_dump_file[0] != '\0') {
        dump_behavior_log_json(behavior_dump_file);
    }

    if (datafile != NULL) {
        if (fflush(datafile) != 0)
            cleanup_failed = 1;

        if (fclose(datafile) != 0)
            cleanup_failed = 1;
    }

    write_to_sim_log("SIM: mission finished -- data file: %s\n",
                     vvp->data_file_name);

    if (write_tab(&config) != 0)
        cleanup_failed = 1;

    return cleanup_failed ? 0 : 1;
}