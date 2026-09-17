#ifndef ODYSSEY_H
#define ODYSSEY_H

#include <time.h>

typedef enum {
    ODYSSEY_UPDATE_ERROR = 0,
    ODYSSEY_UPDATE_OK,
    ODYSSEY_UPDATE_MISSION_COMPLETE
} odyssey_update_status_t;

typedef struct {
    double seawater_density;
} odyssey_environment_t;

int init_odyssey(void);
int start_odyssey(const char *, const char *, const char *, const char *, time_t);
odyssey_update_status_t update_odyssey(double, double);
int end_odyssey(const char *, const char *);

int odyssey_abort_requested(void);
const char *get_odyssey_data_file_name(void);

// odyssey_data_struct
double read_sensor(int);
void update_sensor(int, double); 

// odyssey_behavior_logger
void capture_behavior_activity(void);
void dump_behavior_log_json(const char *file_name);

void set_odyssey_log_decimation(unsigned int decimation);

#endif