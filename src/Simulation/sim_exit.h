#ifndef SHUTDOWN_H
#define SHUTDOWN_H

#include <stdio.h>

#include "odyssey_sim.h"

void exit_sim(int exit_status, int mission_started, const odyssey_dirs_t *dirs);

#endif