#ifndef ODYSSEY_DATA_LOGGER_H
#define ODYSSEY_DATA_LOGGER_H

#include <stdio.h>

#include "odyssey_sim.h"

int init_odyssey_data_logger(char *, FILE **, const odyssey_config_t *, const char *);
int log_data(FILE *);
int write_tab(const odyssey_config_t *);

#endif