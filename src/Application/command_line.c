#include <stdio.h>
#include <string.h>

#include "command_line.h"

void print_command_line_help(const char *program_name) {
    fprintf(stderr, "usage: %s mission [fault_file]\n", program_name);
    fprintf(stderr, "       %s --parse [mission_file]\n", program_name);
    fprintf(stderr, "       %s --nolog [mission_file]\n", program_name);
    fprintf(stderr, "       %s --dump-behavior FILE [mission_file]\n", program_name);
    fprintf(stderr, "       %s --help|-h\n", program_name);
}

int parse_command_line(int argc, char *argv[], command_line_options_t *options) {
    int argument_index;

    if (options == NULL) return -1;

    options->nolog = 0;
    options->show_help = 0;
    options->mission_file = NULL;
    options->fault_file = NULL;
    options->behavior_dump_file[0] = '\0';

    argument_index = 1;

    while (argument_index < argc && argv[argument_index][0] == '-') {
        if (!strcmp(argv[argument_index], "--nolog")) {
            options->nolog = 1;
            argument_index++;
        } else if (!strcmp(argv[argument_index], "--dump-behavior")) {
            if (argument_index + 1 >= argc) {
                fprintf(stderr, "Error: --dump-behavior requires a filename.\n");
                return -1;
            }

            strncpy(options->behavior_dump_file, argv[argument_index + 1], PATH_SIZE - 1);
            options->behavior_dump_file[PATH_SIZE - 1] = '\0';

            argument_index += 2;
        }
        else if (!strcmp(argv[argument_index], "--help") || !strcmp(argv[argument_index], "-h")) {
            options->show_help = 1;
            return 0;
        }
        else {
            fprintf(stderr, "Unknown option: %s\n", argv[argument_index]);
            return -1;
        }
    }

    if (argument_index >= argc) {
        fprintf(stderr, "Error: mission file is required.\n");
        return -1;
    }

    options->mission_file = argv[argument_index];
    argument_index++;

    if (argument_index < argc) {
        options->fault_file = argv[argument_index];
        argument_index++;
    }

    if (argument_index < argc) {
        fprintf(stderr, "Error: unexpected argument: %s\n", argv[argument_index]);
        return -1;
    }

    return 0;
}