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
#include "vehicle.h"

static odyssey_state_t state;

static int load_vehicle_def(const char *fname);

int convert_units(const char *expected, const char *given, const double inval, double *outval);
int match_sensor(const char *name);

/*-------------------------------------------------------------------------
  LOAD_VEHICLE_DEF: load vehicle physical parameters (vs_*) from a
  vehicle definition (.vd) file.  Call BEFORE load_mission().

  File format mirrors mission-file sensor lines:

      # comments start with # % or *
      vehicle: <name>            <- must match VEHICLE_NAME or load fails
      vs_<name>(<units>) <value>
      vs_<name>(<units>) <value>
      ...

  Rules:
   - Header 'vehicle:' must match this build's VEHICLE_NAME.
   - Names must begin with 'vs_' (catches typos that would otherwise
     silently clobber an m_/c_/u_ sensor).
   - Unknown vs_* name -> hard error (catches stale references).
   - vs_* not mentioned in the file -> keeps its compile-time default
     from edit_struct.c (so old .vd files keep working as the data
     structure grows).
  -----------------------------------------------------------------------*/

static int load_vehicle_def(const char *fname) {
    FILE *fptr;
    char line[MAX_LINE], name[NAME_SIZE], header[NAME_SIZE];
    char *expect, *actual;
    double value;
    int index, lineno = 0, n_set = 0, header_seen = 0;

    if ((fptr = fopen(fname, "r")) == NULL) return -1;

    while (fgets(line, sizeof(line), fptr) != NULL) {
        lineno++;
		
        /* skip blank lines and comments */
        if (line[0] == '\n' || line[0] == '\r' || line[0] == '#'  || line[0] == '%' || line[0] == '*') continue;

		/* first real line must be the vehicle header */
        if (!header_seen) {
            if (sscanf(line, " vehicle: %s", header) != 1) {
                fclose(fptr);
                return -1;
            }
            if (strcmp(header, VEHICLE_NAME) != 0) {
                fclose(fptr);
                return -1;
            }
            header_seen = 1;
            continue;
        }

		/* parse "<name> <value>" */
        if (sscanf(line, " %s %lg", name, &value) != 2) {
            fclose(fptr);
            return -1;
        }

		/* only vs_* fields allowed */
        if (strncmp(name, "vs_", 3) != 0) {
            fclose(fptr);
            return -1;
        }

		/* look up by name */
        if ((index = match_sensor(name)) < 0) {
            fclose(fptr);
            return -1;
        }

		/* convert units if given */
        expect = strchr(vvp->s[index].name, '(');
        actual = strchr(name, '(');
        if (expect && actual && convert_units( expect, actual, value, &value)) {
            fclose(fptr);
            return -1;
        }

        update_sensor(index, value);
        n_set++;
    }

    fclose(fptr);

    if (!header_seen) return -1;

    return 0;
}

int init_odyssey_model(const char *path) {
    if (path == NULL || path[0] == '\0') {
        write_to_sim_log("[odyssey_model] Vehicle path is invalid.\n");
        return 0;
    }

    if (load_vehicle_def(path) != 0) {
        write_to_sim_log("[odyssey_model] Failed to load vehicle definition.\n");
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