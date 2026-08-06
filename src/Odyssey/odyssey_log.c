#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "odyssey_log.h"
#include "sim_log.h"

FILE *syslog = NULL;

int open_odyssey_log(void) {
    const char *log_path;

    log_path = getenv("ODYSSEY_LOG_PATH");

    if (log_path == NULL || log_path[0] == '\0') {
        write_to_sim_log("Failed to initialize Odyssey log: ODYSSEY_LOG_PATH is not set.\n");
        return -1;
    }

    syslog = fopen(log_path, "w");

    if (syslog == NULL) {
        write_to_sim_log("Failed to open Odyssey log: %s\n", log_path);
        return -1;
    }

    write_to_sim_log("Odyssey log opened: %s\n", log_path);

    return 0;
}

int close_odyssey_log(void) {
    if (syslog == NULL) return 0;

    if (fclose(syslog) != 0) {
        write_to_sim_log("Failed to close Odyssey log.\n");
        return -1;
    }

    syslog = NULL;
    write_to_sim_log("Odyssey log closed.\n");

    return 0;
}