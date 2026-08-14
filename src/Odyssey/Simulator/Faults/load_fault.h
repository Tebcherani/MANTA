#ifndef LOAD_FAULT_H
#define LOAD_FAULT_H

#include "odyssey_sim.h"

int load_faults(const char *fname, const odyssey_config_t *config);
void apply_faults(void);
const char *get_fault_file_name(void);
const char *get_fault_file_path(void);

#endif