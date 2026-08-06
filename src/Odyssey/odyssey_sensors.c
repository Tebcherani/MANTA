#include "odyssey_sensors.h"

#include "ody_func.h"
#include "ody_defs.h"

int update_odyssey_sensors(void)
{
    // Preserve the original sensor update order
    init_rotation();
    load_angular_rate();
    load_accel();
    load_magnetic();
    load_depth();
    load_water_speed();
    load_water_param();

#ifdef WHOI_USBL
    simulate_usbl();
#endif

    handle_modem();
    heading_reference();
    simLBL();

    return 0;
}