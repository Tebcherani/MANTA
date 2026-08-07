#include "ody_defs.h"
#include "odyssey_data_struct.h"
#include "odyssey_sensors.h"
#include "random_utils.h"
#include "vehicle.h"

void update_depth_sensor(void) {
    double noise = std_normal_random_num()*read_sensor(REAL_DEPTH_NOISE);
    update_sensor(M_DEPTH, read_sensor(REAL_Z) + noise);
}
