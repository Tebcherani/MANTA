#ifndef ODYSSEY_SIM_H
#define ODYSSEY_SIM_H

#define DATA_DIR ody_data_dir
#define MISSION_DIR ody_mission_dir
#define SFAULT_DIR ody_sfault_dir
#define VEHICLE_DIR ody_vehicle_dir

extern const char *ody_data_dir;
extern const char *ody_mission_dir;
extern const char *ody_sfault_dir;
extern const char *ody_vehicle_dir;

int init_odyssey_sim(void);
int update_odyssey_cycle(void);

#endif /* ODYSSEY_SIM_H */