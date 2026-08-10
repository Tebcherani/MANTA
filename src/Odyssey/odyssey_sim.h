#ifndef ODYSSEY_SIM_H
#define ODYSSEY_SIM_H

#include <time.h>

#include "command_line.h"

typedef struct {
    const char *data_dir;
    const char *mission_dir;
    const char *fault_dir;
    const char *model_path;
} odyssey_config_t;

int init_odyssey_sim(odyssey_config_t *);
int start_odyssey_sim(const odyssey_config_t *, const command_line_options_t *, time_t, char *, FILE **);
int update_odyssey_sim_timestep(double);
int update_odyssey_sim_cycle(double control_dt);
int odyssey_abort_requested(void);

#endif /* ODYSSEY_SIM_H */