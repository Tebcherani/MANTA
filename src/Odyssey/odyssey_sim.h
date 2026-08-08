#ifndef ODYSSEY_SIM_H
#define ODYSSEY_SIM_H

#include "command_line.h"

typedef struct {
    const char *data_dir;
    const char *mission_dir;
    const char *fault_dir;
    const char *model_path;
} odyssey_config_t;

int init_odyssey_sim(odyssey_config_t *config, const command_line_options_t *options, char *data_file_name, size_t data_file_name_size, FILE **data_file);
int update_odyssey_sim_cycle(void);

#endif /* ODYSSEY_SIM_H */