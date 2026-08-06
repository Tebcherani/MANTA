#ifndef WORLD_PROTOCOL_H
#define WORLD_PROTOCOL_H

typedef struct {
    double timestep;
} world_step_request_t;

typedef struct {
    int status;
} world_result_t;

#endif /* WORLD_PROTOCOL_H */