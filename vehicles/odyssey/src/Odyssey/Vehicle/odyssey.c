#include <stdio.h>

#include "odyssey.h"
#include "odyssey_data_logger.h"
#include "odyssey_data_struct.h"
#include "odyssey_mission.h"
#include "odyssey_nav.h"
#include "odyssey_control.h"
#include "ody_var.h"
#include "sim_log.h"

static FILE *datafile = NULL;
static unsigned int data_log_decimation = 1U;
static unsigned int data_log_counter = 0U;

int init_odyssey(void) {
    if (!init_odyssey_data_structure()) return 0;

    return 1;
}

int start_odyssey(const char *vehicle_id, const char *mission_path, const char *fault_path, const char *data_dir, time_t calendar_start_time) {
    if (!init_mission_time(calendar_start_time)) return 0;
    if (!init_odyssey_mission(mission_path)) return 0;
    if (!init_odyssey_data_logger(&datafile, vehicle_id, data_dir, mission_path, fault_path)) return 0;

    return 1;
}

void set_odyssey_log_decimation(unsigned int decimation) {
    data_log_decimation = decimation > 0U ? decimation : 1U;
    data_log_counter = 0U;
}

odyssey_update_status_t update_odyssey(double seawater_density, double control_dt) {    
    if (!update_odyssey_nav()) return ODYSSEY_UPDATE_ERROR;
    if (!update_odyssey_control(seawater_density, control_dt)) return ODYSSEY_UPDATE_ERROR;
    if (update_odyssey_mission() != 0) return ODYSSEY_UPDATE_MISSION_COMPLETE;

    capture_behavior_activity();

    if (data_log_counter++ % data_log_decimation == 0U) log_data(datafile);

    return ODYSSEY_UPDATE_OK;
}

int end_odyssey(const char *behavior_dump_file, const char *data_dir) {
    int cleanup_failed = 0;

    if (behavior_dump_file != NULL && behavior_dump_file[0] != '\0') {
        dump_behavior_log_json(behavior_dump_file);
    }

    if (datafile != NULL) {
        if (fflush(datafile) != 0) cleanup_failed = 1;
        if (fclose(datafile) != 0) cleanup_failed = 1;

        datafile = NULL;
    }

    write_to_sim_log("SIM: mission finished -- data file: %s\n", get_odyssey_data_file_name());

    return cleanup_failed ? 0 : 1;
}

int odyssey_abort_requested(void) {
    return vvp->abort_mission;
}

const char *get_odyssey_data_file_name(void) {
    return vvp->data_file_name;
}