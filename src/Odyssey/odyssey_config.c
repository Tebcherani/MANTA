#include <stdlib.h>
#include <string.h>

#include "odyssey_config.h"
#include "sim_log.h"

char ody_data_dir[ODY_PATH_MAX];
char ody_mission_dir[ODY_PATH_MAX];
char ody_sfault_dir[ODY_PATH_MAX];

static int copy_env_path(char *destination, int destination_size, const char *env_name) {
    const char *value;

    value = getenv(env_name);

    if (value == NULL || value[0] == '\0') {
        write_to_sim_log("Failed to initialize Odyssey configuration: %s is not set.\n", env_name);
        return -1;
    }

    strncpy(destination, value, destination_size - 1);
    destination[destination_size - 1] = '\0';

    return 0;
}

int init_odyssey(void) {
    if (copy_env_path(ody_data_dir, ODY_PATH_MAX, "ODYSSEY_DATA_DIR") != 0) return -1;
    write_to_sim_log("Odyssey data directory initialized: %s\n", ody_data_dir);

    if (copy_env_path(ody_mission_dir, ODY_PATH_MAX, "ODYSSEY_MISSION_DIR") != 0) return -1;
    write_to_sim_log("Odyssey mission directory initialized: %s\n", ody_mission_dir);

    if (copy_env_path(ody_sfault_dir, ODY_PATH_MAX, "ODYSSEY_FAULT_DIR") != 0) return -1;
    write_to_sim_log("Odyssey fault directory initialized: %s\n", ody_sfault_dir);

    return 0;
}