#ifndef SIM_CONFIG_H
#define SIM_CONFIG_H

typedef struct {
    double timestep_s;
    int num_cycles;
    int control_period; // Run the sensor processing and control code once every N simulation cycles
} sim_config_t;

int init_sim_config(sim_config_t *);

#endif