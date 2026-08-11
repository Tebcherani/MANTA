#ifndef WORLD_CONFIG_H
#define WORLD_CONFIG_H

typedef struct {
    double gravity;
    double magnetic_north;
    double magnetic_east;
    double magnetic_down;
    double seawater_density;
} world_config_t;

int init_world_config(world_config_t *config);

#endif