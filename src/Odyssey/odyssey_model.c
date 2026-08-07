#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ody_defs.h"
#include "odyssey_model.h"
#include "ody_var.h"
#include "sim_log.h"

int convert_units(const char *expected, const char *given, const double inval, double *outval);
int match_sensor(const char *name);
int load_vehicle_def(const char *fname);

int init_odyssey_model(void) {
    const char *model_path;

    model_path = getenv("VEHICLE_PATH");

    if (model_path == NULL || model_path[0] == '\0') {
        write_to_sim_log("Failed to initialize Odyssey model: VEHICLE_PATH is not set.\n");
        return -1;
    }

    if (load_vehicle_def(model_path) != 0) {
        write_to_sim_log("Failed to initialize Odyssey model from '%s'.\n", model_path);
        return -1;
    }

    write_to_sim_log("Odyssey model initialized from '%s'.\n", model_path);

    return 0;
}