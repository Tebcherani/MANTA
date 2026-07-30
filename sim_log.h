#ifndef SIM_LOG_H
#define SIM_LOG_H

#include <stdio.h>

int open_sim_log(void);
int close_sim_log(void);

int write_to_sim_log(const char *fmt, ...);

#endif