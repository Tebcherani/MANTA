#ifndef LAYERED_CONTROL_H
#define LAYERED_CONTROL_H

#include "odyssey_behaviors.h"

struct behavior;

void layered_control(void);
void resolve(int priority);
void shut_down(int);
int manage_time(struct behavior *b, double duration);

#endif