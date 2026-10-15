#ifndef ODYSSEY_SIM_H
#define ODYSSEY_SIM_H

#include <time.h>

#include "odyssey.h"
#include "world_sim.h"

typedef struct {
    const char *data_dir;
    const char *fault_path;
    const char *mission_path;
    const char *model_path;
    const char *vehicle_id;
} odyssey_config_t;

int init_odyssey_sim();
int start_odyssey_sim(time_t);
int update_odyssey_sim_state(const world_state_t *, double);
odyssey_update_status_t update_odyssey_sim_cycle(const world_state_t *, double);
int update_odyssey_sim_time(double);
int end_odyssey_sim(const char *);

int get_odyssey_metrics(odyssey_metrics_t *);
double get_odyssey_sim_north(void);
double get_odyssey_sim_east(void);
double get_odyssey_sim_depth(void);
int get_odyssey_sim_whale_prediction(double prediction_time, double *measurement_time, double *north, double *east);

#endif /* ODYSSEY_SIM_H */