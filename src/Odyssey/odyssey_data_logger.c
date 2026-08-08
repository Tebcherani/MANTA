#include <stdio.h>

#include "odyssey_data_logger.h"
#include "sim_log.h"

/* Legacy Odyssey function. */
FILE *init_data_logging(char *data_file_name);

int init_odyssey_data_logger(char *data_file_name, FILE **data_file)
{
    if (data_file == NULL || data_file_name == NULL) {
        write_to_sim_log("[odyssey_data_logger.c] Invalid data logger arguments.\n");
        return 0;
    }

    *data_file = init_data_logging(data_file_name);
    if (*data_file == NULL) {
        write_to_sim_log("[odyssey_data_logger.c] Failed to initialize Ody data logger.\n");
        return 0;
    }

    write_to_sim_log("[odyssey_data_logger.c] Ody data logger initialized.\n");
    return 1;
}