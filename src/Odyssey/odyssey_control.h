#ifndef ODYSSEY_CONTROL_H
#define ODYSSEY_CONTROL_H

#include "common_defs.h"
#include "odyssey_behaviors.h"

typedef enum {D_ELEVATOR, D_DEPTH, D_PITCH} dpt_mode_t;
typedef enum {H_RUDDER, H_HEADING} hdt_mode_t;
typedef enum {S_CURRENT, S_SPEED} spd_mode_t;

struct command {
    int behavior;
    struct behavior b;
    dpt_mode_t depth_mode;
    double depth[C_ARG_SIZE];
    hdt_mode_t heading_mode;
    double heading[C_ARG_SIZE];
    spd_mode_t speed_mode;
    double speed[C_ARG_SIZE];
};

int stack_busy(struct command *cmd);
int update_odyssey_control(void);

#endif /* ODYSSEY_CONTROL_H */