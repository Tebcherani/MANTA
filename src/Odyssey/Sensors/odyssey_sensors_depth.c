#include "ody_defs.h"
#include "odyssey_data_struct.h"
#include "odyssey_sensors.h"
#include "vehicle.h"

void update_depth_sensor(void) {
    double noise = std_normal_random_num()*sensor(REAL_DEPTH_NOISE);
    update(M_DEPTH, sensor(REAL_Z) + noise);
}
