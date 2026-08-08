#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ody_defs.h"
#include "odyssey_data_struct.h"
#include "ody_func.h"
#include "odyssey_model.h"
#include "ody_var.h"
#include "sim_log.h"
#include "unit_utils.h"
#include "vehicle.h"

static odyssey_state_t state;

int init_odyssey_model(const char *path) {
    FILE *fptr;
    char line[MAX_LINE], name[NAME_SIZE], header[NAME_SIZE];
    char *expect, *actual;
    double value;
    int index, header_seen = 0;

    if (path == NULL || path[0] == '\0') {
        write_to_sim_log("[odyssey_model] Vehicle path is invalid.\n");
        return 0;
    }

    fptr = fopen(path, "r");
    if (fptr == NULL) {
        write_to_sim_log("[odyssey_model] Failed to open vehicle definition.\n");
        return 0;
    }

    while (fgets(line, sizeof(line), fptr) != NULL) {
        if (line[0] == '\n' || line[0] == '\r' ||
            line[0] == '#' || line[0] == '%' || line[0] == '*')
            continue;

        if (!header_seen) {
            if (sscanf(line, " vehicle: %s", header) != 1) {
                fclose(fptr);
                write_to_sim_log("[odyssey_model] Invalid vehicle definition header.\n");
                return 0;
            }

            if (strcmp(header, VEHICLE_NAME) != 0) {
                fclose(fptr);
                write_to_sim_log("[odyssey_model] Vehicle definition does not match VEHICLE_NAME.\n");
                return 0;
            }

            header_seen = 1;
            continue;
        }

        if (sscanf(line, " %s %lg", name, &value) != 2) {
            fclose(fptr);
            write_to_sim_log("[odyssey_model] Invalid vehicle parameter line.\n");
            return 0;
        }

        if (strncmp(name, "vs_", 3) != 0) {
            fclose(fptr);
            write_to_sim_log("[odyssey_model] Invalid non-vs_ vehicle parameter.\n");
            return 0;
        }

        index = match_sensor(name);
        if (index < 0) {
            fclose(fptr);
            write_to_sim_log("[odyssey_model] Unknown vehicle parameter.\n");
            return 0;
        }

        expect = strchr(vvp->s[index].name, '(');
        actual = strchr(name, '(');

        if (expect && actual &&
            convert_units(expect, actual, value, &value)) {
            fclose(fptr);
            write_to_sim_log("[odyssey_model] Failed to convert vehicle parameter units.\n");
            return 0;
        }

        update_sensor(index, value);
    }

    fclose(fptr);

    if (!header_seen) {
        write_to_sim_log("[odyssey_model] Vehicle definition header not found.\n");
        return 0;
    }

    write_to_sim_log("[odyssey_model] Ody model initialized.\n");
    return 1;
}

int init_odyssey_state(void) {
    int i;
    
    for (i=0; i<7; i++) {
        state.force_moment[i] = 0.0;
        state.position_orientation[i] = 0.0;
    }

    update_sensor(REAL_X, read_sensor(M_POS_N));
    update_sensor(REAL_Y, read_sensor(M_POS_E));
    update_sensor(REAL_Z, read_sensor(M_DEPTH));

    state.position_orientation[1] = read_sensor(REAL_X);
    state.position_orientation[2] = read_sensor(REAL_Y);
    state.position_orientation[3] = read_sensor(REAL_Z);
    state.position_orientation[6] = read_sensor(REAL_PSI); 
    
    state.force_moment[1] = read_sensor(REAL_U);
    state.force_moment[2] = read_sensor(REAL_V);
    state.force_moment[3] = read_sensor(REAL_W);
    
    write_to_sim_log("[odyssey_model] Ody state initialized.\n");
    return 1;
}

int update_odyssey_state(double timestep) {
    double thruster_current;
    double rudder_angle;
    double elevator_angle;
    
    thruster_current = read_sensor(C_THRUSTER_CURRENT);
    rudder_angle = read_sensor(C_RUDDER_ANGLE);
    elevator_angle = read_sensor(C_ELEVATOR_ANGLE);

    motion(
        state.force_moment,
        state.position_orientation,
        thruster_current,
        rudder_angle,
        elevator_angle,
        timestep
    );

    if (fabs(state.force_moment[1]) +
        fabs(state.force_moment[2]) +
        fabs(state.force_moment[3]) +
        fabs(state.force_moment[4]) +
        fabs(state.force_moment[5]) +
        fabs(state.force_moment[6]) > 1.e2
    ) return -1;

    return 0;
}