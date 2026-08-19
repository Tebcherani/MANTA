#include <stdlib.h>

#include "sim_exit.h"
#include "sim_log.h"

void exit_sim(int exit_status) {
    if (close_sim_log() != 0) exit(EXIT_FAILURE);
    exit(exit_status);
}