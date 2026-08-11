#include <stdlib.h>

#include "odyssey_behavior_logger.h"
#include "odyssey_control.h"
#include "odyssey_data_logger.h"
#include "odyssey_data_struct.h"
#include "odyssey_model.h"
#include "odyssey_mission.h"
#include "odyssey_nav.h"
#include "odyssey_sensors.h"
#include "odyssey_sim.h"
#include "ody_var.h"
#include "ody_func.h"
#include "sim_log.h"
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

int init_odyssey_sim(const world_state_t *world_state) {
    if (world_state == NULL) return 0;

    if (!init_odyssey_config()) return 0;
    if (!init_odyssey_data_structure()) return 0;

    return 1;
}

int start_odyssey_sim(const char *mission_file, const char *fault_file, time_t calendar_start_time) {
    if (!init_mission_time(calendar_start_time)) return 0;
    if (!init_odyssey_model(config.model_path)) return 0;
    if (!init_odyssey_mission(mission_file, config.mission_dir)) return 0;
    if (!load_faults(fault_file ? fault_file : vvp->fault_file_name, &config)) return 0;
    if (!init_odyssey_state()) return 0;
    if (!init_odyssey_data_logger(&datafile, &config, mission_file)) return 0;
    if (!init_odyssey_dynamics()) return 0;

    return 1;
}

int update_odyssey_sim_state(double timestep) {
    if (update_odyssey_state(timestep) != 0) return 0;

    return 1;
}

int update_odyssey_sim_cycle(double control_dt) {
    if (update_odyssey_sensors() != 0) return 0;
    if (update_odyssey_nav() != 0) return 0;
    if (update_odyssey_control(control_dt) != 0) return 0;
    if (update_odyssey_mission() != 0) return 0;

    capture_behavior_activity();
	log_data(datafile);
        
    return 1;
}

int update_odyssey_sim_time(double sim_time) {
    update_sensor(M_PRESENT_TIME, sim_time);
    sensor_time(M_PRESENT_TIME) = (long)(1000.0 * sim_time);

    return 1;
}

int odyssey_abort_requested(void) {
    return vvp->abort_mission;
}

int end_odyssey_sim(const char *behavior_dump_file) {
    int cleanup_failed = 0;

    vvp->modem_send[0] = '\0';

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