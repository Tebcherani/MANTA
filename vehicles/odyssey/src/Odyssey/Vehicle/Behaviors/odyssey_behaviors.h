#ifndef ODYSSEY_BEHAVIORS_H
#define ODYSSEY_BEHAVIORS_H

#include "odyssey_defs.h"

// The state of a behavior and its universe of allowed values
typedef enum {
    B_UNINITIALIZED, B_ACTIVE, B_COMPLETE
} b_state_t;

struct b_arg {
    char name[NAME_SIZE];   // Name AND UNITS!!
    double x;			    // Present value
};

struct behavior {
    char name[NAME_SIZE];
    b_state_t state;    // State it's in, see above enum typedef
    double start_time;  // When it started
    double end_time;    // When it will end
	
    int (*fcn)(struct behavior *, int, int);    // The function that this behavior executes
    struct b_arg arg[B_ARG_SIZE];   // Its vector of arguments
};

int assign_behavior_fcn_ptr(const char *, struct behavior *);

int dock(struct behavior *, int, int);
int undock(struct behavior *, int, int);
int wiggle(struct behavior *, int, int);
int whale_follow(struct behavior *, int, int);

void set_whale_follow_target(double north, double east);

#endif /* ODYSSEY_BEHAVIORS_H */