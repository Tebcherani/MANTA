#ifndef COMMAND_LINE_H
#define COMMAND_LINE_H

#include "common_defs.h"

typedef struct {
    int show_help;

    const char *mission_file;
    const char *fault_file;

    char behavior_dump_file[PATH_SIZE];
} command_line_options_t;

int parse_command_line(int, char **, command_line_options_t *);
void print_command_line_help(const char *);

#endif