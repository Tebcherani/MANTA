#include "dynamic_control.h"
#include "layered_control.h"
#include "odyssey_control.h"
#include "odyssey_data_struct.h"
#include "vehicle.h"

int update_odyssey_control(double seawater_density, double control_dt) {
    apply_faults();
    layered_control();
    dynamic_control(seawater_density);

    // Update clock for next control cycle
    update_sensor(M_DT, control_dt);

    return 1;
}