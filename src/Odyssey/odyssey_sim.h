#ifndef ODYSSEY_SIM_H
#define ODYSSEY_SIM_H

typedef struct {
    const char *data_dir;
    const char *mission_dir;
    const char *fault_dir;
} odyssey_dirs_t;

int init_odyssey_sim(odyssey_dirs_t *dirs);
int update_odyssey_sim_cycle(void);

#endif /* ODYSSEY_SIM_H */