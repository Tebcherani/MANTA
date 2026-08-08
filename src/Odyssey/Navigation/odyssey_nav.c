#include "ody_func.h"
#include "odyssey_nav.h"

int update_odyssey_nav(void) {
    update_dead_reckoned_pos();
    LBL_position();

    return 0;
}