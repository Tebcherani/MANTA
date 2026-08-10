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

int init_odyssey_sim(odyssey_config_t *config);
int start_odyssey_sim(const odyssey_config_t *config, const command_line_options_t *options, time_t calendar_start_time, char *data_file_name, FILE **data_file);
int update_odyssey_sim_cycle(void);

#endif /* ODYSSEY_SIM_H */