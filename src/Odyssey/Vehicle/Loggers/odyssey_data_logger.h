#ifndef ODYSSEY_DATA_LOGGER_H
#define ODYSSEY_DATA_LOGGER_H

#include <stddef.h>
#include <stdio.h>

#include "odyssey_sim.h"

int init_odyssey_data_logger(FILE **, const char *, const char *, const char *, const char *);
int log_data(FILE *);

#endif