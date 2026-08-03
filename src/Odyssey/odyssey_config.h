#ifndef ODYSSEY_CONFIG_H
#define ODYSSEY_CONFIG_H

extern char ody_data_dir[512];
extern char ody_sfault_dir[512];

#define DATA_DIR ody_data_dir
#define SFAULT_DIR ody_sfault_dir

int init_odyssey(void);

#endif