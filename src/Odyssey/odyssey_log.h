#ifndef ODYSSEY_LOG_H
#define ODYSSEY_LOG_H

#include <stdio.h>

extern FILE *syslog;

int open_odyssey_log(void);
int close_odyssey_log(void);

#endif