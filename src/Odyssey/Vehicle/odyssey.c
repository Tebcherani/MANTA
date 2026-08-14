#include "odyssey.h"
#include "odyssey_data_struct.h"
#include "odyssey_mission.h"
#include "odyssey_nav.h"
#include "odyssey_control.h"
#include "ody_var.h"

int init_odyssey(void) {
    if (!init_odyssey_data_structure()) return 0;

    return 1;
}

int start_odyssey(const char *mission_file, const char *mission_dir, time_t calendar_start_time) {
    if (!init_mission_time(calendar_start_time)) return 0;
    if (!init_odyssey_mission(mission_file, mission_dir)) return 0;

    return 1;
}

int update_odyssey(double seawater_density, double control_dt) {
    if (!update_odyssey_nav()) return 0;
    if (!update_odyssey_control(seawater_density, control_dt)) return 0;
    if (update_odyssey_mission() != 0) return 0;

    return 1;
}

int odyssey_abort_requested(void) {
    return vvp->abort_mission;
}

const char *get_odyssey_data_file_name(void) {
    return vvp->data_file_name;
}