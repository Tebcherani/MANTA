#ifndef ODYSSEY_H
#define ODYSSEY_H

#include <time.h>

typedef struct {
    double seawater_density;
} odyssey_environment_t;

int init_odyssey(void);
int start_odyssey(const char *, const char *, time_t);
int update_odyssey(double, double);

int odyssey_abort_requested(void);
const char *get_odyssey_data_file_name(void);

// odyssey_data_struct
double read_sensor(int);
void update_sensor(int, double); 

// odyssey_behavior_logger
void capture_behavior_activity(void);
void dump_behavior_log_json(const char *file_name);

#endif