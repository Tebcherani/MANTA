#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "sim_log.h"

static FILE *file = NULL;
static char path[512];

int open_sim_log(void) {
    // Get simulation log file path from Docker compose
    const char *path_ptr = getenv("SIM_LOG_PATH");
    if (path_ptr == NULL || path_ptr[0] == '\0') {
        fprintf(stderr, "SIM_LOG_PATH environment variable is not set.\n");
        return -1;
    }

    int result = snprintf(path, sizeof(path), "%s", path_ptr);
    if (result < 0 || (size_t)result >= sizeof(path)) {
        fprintf(stderr, "SIM_LOG_PATH exceeds the maximum supported length.\n");
        return -1;
    }

    // Open simulation log file
    file = fopen(path, "w");
    if (file == NULL) {
        perror(path);
        return -1;
    }

    if (write_to_sim_log("Simulation log opened: %s\n", path) != 0) {
        fclose(file);
        file = NULL;
        return -1;
    }

    return 0;
}

int close_sim_log(void) {
    if (file == NULL) return 0;

    // Close file
    if (fclose(file) != 0) {
        perror(path);
        file = NULL;
        return -1;
    }
    file = NULL;

    fprintf(stderr, "Simulation log closed.\n");

    return 0;
}

int write_to_sim_log(const char *fmt, ...) {
    va_list args_terminal;
    va_list args_log;

    if (file == NULL) {
        fprintf(stderr, "Simulation log is not open.\n");
        return -1;
    }

    if (fmt == NULL) {
        fprintf(stderr, "Simulation log format string is NULL.\n");
        return -1;
    }

    va_start(args_terminal, fmt);
    va_copy(args_log, args_terminal);

    int terminal_result = vfprintf(stdout, fmt, args_terminal);
    va_end(args_terminal);

    if (terminal_result < 0) {
        va_end(args_log);
        perror("stdout");
        return -1;
    }

    if (fflush(stdout) != 0) {
        va_end(args_log);
        perror("stdout");
        return -1;
    }

    int log_result = vfprintf(file, fmt, args_log);
    va_end(args_log);

    if (log_result < 0) {
        perror(path);
        return -1;
    }

    if (fflush(file) != 0) {
        perror(path);
        return -1;
    }

    return 0;
}