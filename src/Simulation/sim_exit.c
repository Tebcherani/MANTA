#include <stdio.h>
#include <stdlib.h>

#include "command_line.h"
#include "sim_behavior_logger.h"
#include "sim_exit.h"
#include "sim_log.h"
#include "vehicle.h"

extern FILE *datafile;
extern command_line_options_t options;

int handle_modem(void);
int write_tab(void);

void exit_sim(int exit_status, int mission_started) {
    int cleanup_failed = 0;

    if (datafile != NULL) {
        if (fflush(datafile) != 0) cleanup_failed = 1;
        if (fclose(datafile) != 0) cleanup_failed = 1;

        datafile = NULL;
    }

    if (mission_started) {
        handle_modem();
        if (options.behavior_dump_file[0] != '\0') dump_behavior_log_json(options.behavior_dump_file);
        write_to_sim_log("SIM: mission finished -- data file: %s\n", vvp->data_file_name);
        write_tab();
    }

    if (close_sim_log() != 0) cleanup_failed = 1;

    if (cleanup_failed) exit(EXIT_FAILURE);
    exit(exit_status);
}