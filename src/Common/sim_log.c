#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "sim_log.h"

static FILE *file = NULL;
static const char *path = NULL;

int open_sim_log(void) {
    path = getenv("SIM_LOG_PATH");

    if (path == NULL || path[0] == '\0') {
        fprintf(stderr, "\n[sim_log] SIM_LOG_PATH is not set.\n");
        return 0;
    }

    file = fopen(path, "w");
    if (file == NULL) {
        perror(path);
        return 0;
    }

    write_to_sim_log("\n[sim_log] Sim log opened.\n");

    return 1;
}

int close_sim_log(void) {
    if (file == NULL) return 0;

    if (fclose(file) != 0) {
        perror(path);
        file = NULL;
        return 0;
    }

    file = NULL;

    fprintf(stderr, "Simulation log closed.\n");

    return 1;
}

int write_to_sim_log(const char *fmt, ...) {
    va_list args_terminal;
    va_list args_log;

    if (file == NULL) {
        fprintf(stderr, "Simulation log is not open.\n");
        return 0;
    }

    if (fmt == NULL) {
        fprintf(stderr, "Simulation log format string is NULL.\n");
        return 0;
    }

    va_start(args_terminal, fmt);
    va_copy(args_log, args_terminal);

    int terminal_result = vfprintf(stdout, fmt, args_terminal);
    va_end(args_terminal);

    if (terminal_result < 0) {
        va_end(args_log);
        perror("stdout");
        return 0;
    }

    if (fflush(stdout) != 0) {
        va_end(args_log);
        perror("stdout");
        return 0;
    }

    int log_result = vfprintf(file, fmt, args_log);
    va_end(args_log);

    if (log_result < 0) {
        perror(path);
        return 0;
    }

    if (fflush(file) != 0) {
        perror(path);
        return 0;
    }

    return 1;
}