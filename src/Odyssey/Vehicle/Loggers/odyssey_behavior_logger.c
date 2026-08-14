#include <stdio.h>

#include "odyssey_behavior_logger.h"
#include "sim_log.h"
#include "vehicle.h"
#include "odyssey_data_struct.h"

#define BEHAVIOR_LOG_SIZE 100000

typedef struct {
    double time;
    int state;
    int priority;
    int behavior_id;
    int heading_mode;
    int depth_mode;
    int speed_mode;
    double c0_heading;
    double c0_depth;
    double c0_speed;
} behavior_log_entry_t;

static behavior_log_entry_t behavior_log[BEHAVIOR_LOG_SIZE];
static int behavior_log_count = 0;

void capture_behavior_activity(void) {
    int command_index;
    behavior_log_entry_t *entry;

    if (behavior_log_count >= BEHAVIOR_LOG_SIZE) return;

    entry = &behavior_log[behavior_log_count];

    entry->time = read_sensor(M_PRESENT_TIME);
    entry->state = vvp->cstate;

    entry->heading_mode = vvp->c[0].heading_mode;
    entry->depth_mode = vvp->c[0].depth_mode;
    entry->speed_mode = vvp->c[0].speed_mode;

    entry->c0_heading = vvp->c[0].heading[0];
    entry->c0_depth = vvp->c[0].depth[0];
    entry->c0_speed = vvp->c[0].speed[0];

    entry->behavior_id = -1;
    entry->priority = -1;

    // Record the active behavior with the highest priority.
    for (command_index = 0; command_index < MAX_COMMAND; command_index++) {
        if (vvp->sta[vvp->cstate].c[command_index].behavior != -1) {
            entry->behavior_id = vvp->sta[vvp->cstate].c[command_index].behavior;
            entry->priority = command_index;
            break;
        }
    }

    behavior_log_count++;
}

int dump_behavior_log_json(const char *filename) {
    FILE *file;
    int entry_index;

    if (filename == NULL || filename[0] == '\0') return -1;

    file = fopen(filename, "w");

    if (file == NULL) return -1;

    fprintf(file, "[\n");

    for (entry_index = 0; entry_index < behavior_log_count; entry_index++) {
        fprintf(file, "  {\n");
        fprintf(file, "    \"time\": %.2f,\n", behavior_log[entry_index].time);
        fprintf(file, "    \"state\": %d,\n", behavior_log[entry_index].state);
        fprintf(file, "    \"behavior_id\": %d,\n", behavior_log[entry_index].behavior_id);
        fprintf(file, "    \"priority\": %d,\n", behavior_log[entry_index].priority);
        fprintf(file, "    \"heading_mode\": %d,\n", behavior_log[entry_index].heading_mode);
        fprintf(file, "    \"depth_mode\": %d,\n", behavior_log[entry_index].depth_mode);
        fprintf(file, "    \"speed_mode\": %d,\n", behavior_log[entry_index].speed_mode);
        fprintf(file, "    \"c0_heading\": %.4f,\n", behavior_log[entry_index].c0_heading);
        fprintf(file, "    \"c0_depth\": %.4f,\n", behavior_log[entry_index].c0_depth);
        fprintf(file, "    \"c0_speed\": %.4f\n", behavior_log[entry_index].c0_speed);
        fprintf(file, "  }");

        if (entry_index < behavior_log_count - 1)
            fprintf(file, ",");

        fprintf(file, "\n");
    }

    if (fclose(file) != 0) return -1;

    return 0;
}