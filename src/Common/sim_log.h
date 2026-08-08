#ifndef SIM_LOG_H
#define SIM_LOG_H

#include <stdio.h>

#define PATH_SIZE 512

int close_sim_log(void);
int open_sim_log(void);
int write_to_sim_log(const char *fmt, ...);

#endif