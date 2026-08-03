#ifndef ODYSSEY_CONFIG_H
#define ODYSSEY_CONFIG_H

#define DATA_DIR ody_data_dir
#define MISSION_DIR ody_mission_dir
#define ODY_PATH_MAX 512
#define SFAULT_DIR ody_sfault_dir

extern char ody_data_dir[ODY_PATH_MAX];
extern char ody_mission_dir[ODY_PATH_MAX];
extern char ody_sfault_dir[ODY_PATH_MAX];

int init_odyssey(void);

#endif