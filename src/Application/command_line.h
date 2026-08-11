#ifndef COMMAND_LINE_H
#define COMMAND_LINE_H

typedef struct {
    int show_help;

    const char *mission_file;
    const char *fault_file;
    const char *behavior_dump_file;
} command_line_options_t;

int parse_command_line(int, char **, command_line_options_t *);
void print_command_line_help(const char *);

#endif