#include <math.h>
#include <stdlib.h>

#include "load_fault.h"
#include "odyssey.h"
#include "odyssey_behaviors.h"
#include "odyssey_model.h"
#include "odyssey_sensors.h"
#include "odyssey_sim.h"
#include "sim_log.h"

#include "vehicle.h"

static int init_odyssey_config(void);

static odyssey_config_t config;
static odyssey_environment_t environment;
static double water_speed_sum = 0.0;
static unsigned long water_speed_samples = 0;

static int init_odyssey_config(void) {
    config.data_dir = getenv("ODYSSEY_DATA_DIR");
    if (config.data_dir == NULL || config.data_dir[0] == '\0') {
        write_to_sim_log("[odyssey_sim] ODYSSEY_DATA_DIR is not set.\n");
        return 0;
    }

    config.mission_path = getenv("MISSION_PATH");
    if (config.mission_path == NULL || config.mission_path[0] == '\0') {
        write_to_sim_log("[odyssey_sim] MISSION_PATH is not set.\n");
        return 0;
    }

    config.fault_path = getenv("FAULT_PATH");
    if (config.fault_path == NULL) {
        config.fault_path = "";
        write_to_sim_log("[odyssey_sim] FAULT_PATH is not set.\n");
        return 0;
    }

    config.vehicle_id = getenv("ODYSSEY_VEHICLE_ID");
    if (config.vehicle_id == NULL || config.vehicle_id[0] == '\0') {
        write_to_sim_log("[odyssey_sim] ODYSSEY_VEHICLE_ID is not set.\n");
        return 0;
    }

    config.model_path = getenv("VEHICLE_PATH");
    if (config.model_path == NULL || config.model_path[0] == '\0') {
        write_to_sim_log("[odyssey_sim] VEHICLE_PATH is not set.\n");
        return 0;
    }

    write_to_sim_log("[odyssey_sim] Odyssey configuration complete.\n");
    return 1;
}

int init_odyssey_sim() {
    if (!init_odyssey_config()) return 0;
    if (!init_odyssey()) return 0;
    if (!init_odyssey_sensors()) return 0;

    return 1;
}

int start_odyssey_sim(time_t calendar_start_time) {
    if (!start_odyssey(config.vehicle_id, config.mission_path, config.fault_path, config.data_dir, calendar_start_time)) return 0;
    if (!init_odyssey_model(config.model_path)) return 0;
    if (!load_faults(config.fault_path)) return 0;
    if (!init_odyssey_state()) return 0;
    if (!init_odyssey_dynamics()) return 0;

    environment.gravity = 0.0;
    environment.seawater_density = 0.0;
    water_speed_sum = 0.0;
    water_speed_samples = 0;

    return 1;
}

int update_odyssey_sim_state(const world_state_t *world_state, double timestep) {
    if (world_state == NULL) return 0;

    environment.gravity = world_state->gravity;
    environment.seawater_density = world_state->seawater_density;

    if (update_odyssey_state(world_state, timestep) != 0) return 0;

    return 1;
}

odyssey_update_status_t update_odyssey_sim_cycle(const world_state_t *world_state, double timestep) {
    double water_speed;

    if (world_state == NULL) return ODYSSEY_UPDATE_ERROR;
    
    environment.gravity = world_state->gravity;
    environment.seawater_density = world_state->seawater_density;

    if (!update_odyssey_sensors(world_state, timestep)) return ODYSSEY_UPDATE_ERROR;        
    
    water_speed = fabs(read_sensor(M_WATER_SPEED));
    water_speed_sum += water_speed;
    water_speed_samples++;

    return update_odyssey(world_state->seawater_density, timestep);
}

int update_odyssey_sim_time(double sim_time) {
    //update_sensor(M_PRESENT_TIME, sim_time);
    vvp->s[M_PRESENT_TIME].x = sim_time;
    sensor_time(M_PRESENT_TIME) = (long)(1000.0 * sim_time);

    return 1;
}

int end_odyssey_sim(const char *behavior_dump_file) {
    return end_odyssey(behavior_dump_file, config.data_dir);
}

int get_odyssey_metrics(odyssey_metrics_t *metrics) {
    if (metrics == NULL) return 0;
    if (environment.gravity <= 0.0) return 0;

    metrics->mass = read_sensor(REAL_VEH_W) / environment.gravity;
    metrics->energy_consumed = read_sensor(REAL_BATTERY_ENERGY_USED) / 3600.0;

    if (water_speed_samples > 0) metrics->mean_speed = water_speed_sum / (double)water_speed_samples;
    else metrics->mean_speed = 0.0;

    return 1;
}

double get_odyssey_sim_north(void) {
    return read_sensor(REAL_X);
}

double get_odyssey_sim_east(void) {
    return read_sensor(REAL_Y);
}

double get_odyssey_sim_depth(void) {
    return read_sensor(REAL_Z);
}

int get_odyssey_sim_whale_prediction(double prediction_time, double *measurement_time, double *north, double *east) {
    return get_whale_trailing_prediction(prediction_time, measurement_time, north, east);
}