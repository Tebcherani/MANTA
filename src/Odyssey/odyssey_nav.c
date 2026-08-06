#include "odyssey_nav.h"

#include "ody_func.h"

int update_odyssey_nav(void) {
    simple_dead_reckoning();
    LBL_position();

    return 0;
}