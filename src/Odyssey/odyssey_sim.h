#ifndef ODYSSEY_SIM_H
#define ODYSSEY_SIM_H

#include <time.h>

typedef struct {
    const char *data_dir;
    const char *mission_dir;
    const char *fault_dir;
    const char *model_path;
} odyssey_config_t;

int init_odyssey_sim(odyssey_config_t *);
int start_odyssey_sim(const odyssey_config_t *, const char *, const char *, time_t, char *, FILE **);
int update_odyssey_sim_timestep(double);
int update_odyssey_sim_cycle(double, FILE *);
int odyssey_abort_requested(void);
int end_odyssey_sim(FILE *, const odyssey_config_t *, const char *);

#endif /* ODYSSEY_SIM_H */