#include <stdio.h>

#include "odyssey_data_logger.h"
#include "odyssey_sim.h"
#include "sim_log.h"

/* Legacy Odyssey function. */
FILE *init_data_logging(char *data_file_name, const odyssey_config_t *config);

int init_odyssey_data_logger(char *data_file_name, FILE **data_file, const odyssey_config_t *config) {
    if (data_file == NULL || data_file_name == NULL || config == NULL) {
        write_to_sim_log("[odyssey_data_logger] Invalid data logger arguments.\n");
        return 0;
    }

    *data_file = init_data_logging(data_file_name, config);
    if (*data_file == NULL) {
        write_to_sim_log("[odyssey_data_logger] Failed to initialize Ody data logger.\n");
        return 0;
    }

    write_to_sim_log("[odyssey_data_logger] Ody data logger initialized.\n");
    return 1;
}