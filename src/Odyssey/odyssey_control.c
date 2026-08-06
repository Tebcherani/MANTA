#include "odyssey_control.h"

#include "ody_func.h"

int update_odyssey_control(void) {
    apply_faults();
    layered_control();
    dynamic_control();

    return 0;
}