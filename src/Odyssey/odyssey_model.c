#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ody_defs.h"
#include "odyssey_model.h"
#include "ody_var.h"
#include "sim_log.h"

void update(int index, double val);
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
static int load_odyssey_model(const char *model_path) {
    char header[64], line[196], name[64];
    char *actual_unit, *expected_unit;
    double value;
    FILE *model_file;
    int header_seen = 0, line_num = 0, num_params = 0;
    int index;

    // Open model file
    if ((model_file = fopen(model_path, "r")) == NULL) {
        write_to_sim_log(
            "Failed to open Odyssey model '%s': %s\n", model_path,
            strerror(errno)
        );
        return -1;
    }

    while (fgets(line, sizeof(line), model_file) != NULL ) {
        line_num++;

		// Skip blank lines and comments
        if (line[0] == '\n' || line[0] == '\r' ||
            line[0] == '#'  || line[0] == '%' || line[0] == '*')
            continue;

		// The first non-comment line must contain the vehicle header
        if (!header_seen) {
            if (sscanf(line, " vehicle: %63s", header) != 1) {
                write_to_sim_log("Odyssey model '%s' is missing the 'vehicle:' header at line %d.\n", model_path, line_num);
                fclose(model_file);
                return -1;
            }

            if (strcmp(header, VEHICLE_NAME) != 0) {
                write_to_sim_log("Odyssey model '%s' defines vehicle '%s', but this build expects '%s'.\n", model_path, header, VEHICLE_NAME);
                fclose(model_file);
                return -1;
            }

            header_seen = 1;
            continue;
        }

		// Parse "<name> <value>"
        if (sscanf(line, " %63s %lg", name, &value) != 2) {
            write_to_sim_log("Failed to parse Odyssey model '%s' at line %d: %s", model_path, line_num, line);
            fclose(model_file);
            return -1;
        }

		// Only vs_* fields are allowed
        if (strncmp(name, "vs_", 3) != 0) {
            write_to_sim_log("Invalid parameter in Odyssey model '%s' at line %d: '%s' is not a vs_* parameter.\n", model_path, line_num, name);
            fclose(model_file);
            return -1;
        }

		// Look up the parameter by name
        if ((index = match_sensor(name)) < 0) {
            write_to_sim_log("Unknown parameter in Odyssey model '%s' at line %d: '%s'.\n", model_path, line_num, name);
            fclose(model_file);
            return -1;
        }

		// Convert units if given
        expected_unit = strchr(vvp->s[index].name, '(');
        actual_unit = strchr(name, '(');

        if (expected_unit && actual_unit && convert_units(expected_unit, actual_unit, value, &value)) {
            write_to_sim_log("Unknown units in Odyssey model '%s' at line %d: '%s'.\n", model_path, line_num, name);
            fclose(model_file);
            return -1;
        }

        update(index, value);
        num_params++;
    }

    // Close model file
    fclose(model_file);

    if (!header_seen) {
        write_to_sim_log("Odyssey model '%s' is empty or contains no vehicle header.\n", model_path);
        return -1;
    }

    write_to_sim_log("Loaded %d Odyssey model parameters from '%s'.\n", num_params, model_path);

    return 0;
}

int init_odyssey_model(void) {
    const char *model_path;

    // Get model path
    model_path = getenv("VEHICLE_PATH");

    if (model_path == NULL || model_path[0] == '\0') {
        write_to_sim_log("Failed to initialize Odyssey model: VEHICLE_PATH is not set.\n");
        return -1;
    }

    // Load model
    if (load_odyssey_model(model_path) != 0) {
        write_to_sim_log("Failed to load Odyssey model: %s\n", model_path);
        return -1;
    }

    write_to_sim_log("Odyssey model initialized from %s\n", model_path);

    return 0;
}