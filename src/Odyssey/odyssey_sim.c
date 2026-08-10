#include <math.h>
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

int init_odyssey_sim(odyssey_config_t *config) {
    if (!init_odyssey_config(config)) return 0;
    if (!init_odyssey_data_structure()) return 0;

    return 1;
}

int start_odyssey_sim(const odyssey_config_t *config, const command_line_options_t *options, time_t calendar_start_time, char *data_file_name, FILE **data_file) {
    if (!init_mission_time(calendar_start_time)) return 0;
    if (!init_odyssey_model(config->model_path)) return 0;
    if (!init_odyssey_mission(options->mission_file, config->mission_dir)) return 0;
    if (!load_faults(options->fault_file ? options->fault_file : vvp->fault_file_name, config)) return 0;
    if (!init_odyssey_state()) return 0;
    if (!init_odyssey_data_logger(data_file_name, data_file, config, options->mission_file)) return 0;
    if (!init_odyssey_dynamics()) return 0;

    return 1;
}

int update_odyssey_sim_timestep(double timestep) {
    if (update_odyssey_state(timestep) != 0) return -1;
}

int update_odyssey_sim_cycle(double control_dt, FILE *datafile) {
    if (update_odyssey_sensors() != 0) return -1;
    if (update_odyssey_nav() != 0) return -1;
    if (update_odyssey_control(control_dt) != 0) return -1;
    if (update_odyssey_mission() != 0) return -1;

    capture_behavior_activity();
	log_data(datafile);
        
    return 1;
}

int odyssey_abort_requested(void) {
    return vvp->abort_mission;
}

int end_odyssey_sim(FILE *datafile, const odyssey_config_t *config, const command_line_options_t *options) {
    int cleanup_failed = 0;

    if (handle_modem() != 0)
        cleanup_failed = 1;

    if (options != NULL &&
        options->behavior_dump_file[0] != '\0') {
        dump_behavior_log_json(options->behavior_dump_file);
    }

    if (datafile != NULL) {
        if (fflush(datafile) != 0)
            cleanup_failed = 1;

        if (fclose(datafile) != 0)
            cleanup_failed = 1;
    }

    write_to_sim_log("SIM: mission finished -- data file: %s\n",
                     vvp->data_file_name);

    if (write_tab(config) != 0)
        cleanup_failed = 1;

    return cleanup_failed ? 0 : 1;
}