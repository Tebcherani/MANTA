#include <stdio.h>
#include <stdlib.h>

#include "odyssey_log.h"
#include "sim_exit.h"
#include "sim_log.h"

extern FILE *datafile;

void exit_sim(int exit_status) {
    int cleanup_failed = 0;

    if (datafile != NULL) {
        if (fflush(datafile) != 0) cleanup_failed = 1;
        if (fclose(datafile) != 0) cleanup_failed = 1;

        datafile = NULL;
    }

    if (close_odyssey_log() != 0) cleanup_failed = 1;
    if (close_sim_log() != 0) cleanup_failed = 1;

    if (cleanup_failed) exit(EXIT_FAILURE);
    exit(exit_status);
}