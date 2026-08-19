#ifndef ODYSSEY_SIM_H
#define ODYSSEY_SIM_H

#include <time.h>

#include "odyssey.h"
#include "world_sim.h"

typedef struct {
    const char *data_dir;
    const char *fault_dir;
    const char *mission_path;
    const char *model_path;
    const char *vehicle_id;
} odyssey_config_t;

int init_odyssey_sim();
int start_odyssey_sim(const char *, time_t);
int update_odyssey_sim_state(const world_state_t *, double);
odyssey_update_status_t update_odyssey_sim_cycle(const world_state_t *, double);
int update_odyssey_sim_time(double);
int end_odyssey_sim(const char *);

#endif /* ODYSSEY_SIM_H */