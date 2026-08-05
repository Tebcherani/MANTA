#include <stdio.h>

#include "odyssey_data_logger.h"
#include "sim_log.h"

/* Legacy Odyssey function. */
FILE *init_data_logging(char *data_file_name);

int init_odyssey_data_logger(int logging_enabled, char *data_file_name, FILE **data_file) {
    if (data_file == NULL || data_file_name == NULL) {
        write_to_sim_log("Failed to initialize Odyssey data logger: invalid argument.\n");
        return -1;
    }

    *data_file = NULL;

    if (!logging_enabled) {
        write_to_sim_log("Odyssey data logging disabled (--nolog mode).\n");
        return 0;
    }

    *data_file = init_data_logging(data_file_name);

    if (*data_file == NULL) {
        write_to_sim_log("Failed to initialize Odyssey data logging.\n");
        return -1;
    }

    return 0;
}