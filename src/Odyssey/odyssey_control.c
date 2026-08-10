#include "odyssey_control.h"
#include "vehicle.h"

#include "ody_func.h"

int update_odyssey_control(double control_dt) {
    apply_faults();
    layered_control();
    dynamic_control();

    // Update clock for next control cycle
    update_sensor(M_DT, control_dt);

    return 0;
}