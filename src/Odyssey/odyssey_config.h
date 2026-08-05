#ifndef ODYSSEY_CONFIG_H
#define ODYSSEY_CONFIG_H

#include "common_defs.h"

#define DATA_DIR ody_data_dir
#define MISSION_DIR ody_mission_dir
#define SFAULT_DIR ody_sfault_dir
#define VEHICLE_DIR ody_vehicle_dir

extern char ody_data_dir[PATH_SIZE];
extern char ody_mission_dir[PATH_SIZE];
extern char ody_sfault_dir[PATH_SIZE];
extern char ody_vehicle_dir[PATH_SIZE];

int init_odyssey_config(void);

#endif