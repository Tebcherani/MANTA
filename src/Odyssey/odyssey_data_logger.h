#ifndef ODYSSEY_DATA_LOGGER_H
#define ODYSSEY_DATA_LOGGER_H

#include <stdio.h>

#include "odyssey_sim.h"

int init_odyssey_data_logger(char *data_file_name, FILE **data_file, const odyssey_dirs_t *dirs);

#endif