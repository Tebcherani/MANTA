#include <stdio.h>
#include <stdlib.h>

#include "sim_exit.h"
#include "sim_log.h"

void exit_sim(int exit_status, int mission_started, FILE *datafile, const odyssey_config_t *config, const char *behavior_dump_file) {
    int cleanup_failed = 0;

    if (mission_started) {
        if (!end_odyssey_sim(datafile, config, behavior_dump_file))
            cleanup_failed = 1;
    }
    else if (datafile != NULL) {
        if (fflush(datafile) != 0)
            cleanup_failed = 1;

        if (fclose(datafile) != 0)
            cleanup_failed = 1;
    }

    if (close_sim_log() != 0)
        cleanup_failed = 1;

    if (cleanup_failed)
        exit(EXIT_FAILURE);

    exit(exit_status);
}