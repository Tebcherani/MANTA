#include "common_defs.h"
#include "ody_func.h"
#include "ody_defs.h"
#include "odyssey_sensors.h"
#include "vehicle.h"
#include "odyssey_data.h"

int update_odyssey_sensors(void) {
    update_rotation_matrix();
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
    update_heading();
    simLBL();

    return 0;
}

void update_heading(void) {
    // Direct-copy path: simulated inclinometer -> roll/pitch, simulated compass -> yaw
    double flux_yaw = PI + modPi(sensor(M_COMPASS_YAW) + sensor(UO_MAGNETIC_VARIATION) - PI);
    update(M_ROLL, sensor(M_INCLINO_ROLL));
    update(M_PITCH, sensor(M_INCLINO_PITCH));
    update(M_YAW, flux_yaw);
}

void update_rotation_matrix(void) {
    double rotation_matrix[4][4];
  
    double yaw_angle = sensor(REAL_PSI);
    double pitch_angle = sensor(REAL_THETA);
    double roll_angle = sensor(REAL_PHI);
    
    /* To find the body frame representation of a vector in the earth frame do:                         */
    /* xb[i] = rotation_matrix[i][1]*xe[1] + rotation_matrix[i][2]*xe[2] + rotation_matrix[i][3]*xe[3]  */
    /*                                                                                                  */
    /* To find the earth frame representation of a vector in the body frame do:                         */
    /* xe[i] = rotation_matrix[1][i]*xb[1] + rotation_matrix[2][i]*xb[2] + rotation_matrix[3][i]*xb[3]  */
    /*                                                                                                  */
    
    rotation_matrix[1][1] = cos(yaw_angle) * cos(pitch_angle);
    rotation_matrix[1][2] = cos(pitch_angle) * sin(yaw_angle);
    rotation_matrix[1][3] = -sin(pitch_angle);
    
    rotation_matrix[2][1] = -(cos(roll_angle) * sin(yaw_angle)) + cos(yaw_angle) * sin(roll_angle) * sin(pitch_angle);
    rotation_matrix[2][2] = cos(roll_angle) * cos(yaw_angle) + sin(roll_angle) * sin(yaw_angle) * sin(pitch_angle);
    rotation_matrix[2][3] = cos(pitch_angle) * sin(roll_angle); 
    
    rotation_matrix[3][1] = sin(roll_angle) * sin(yaw_angle) + cos(roll_angle) * cos(yaw_angle) * sin(pitch_angle);
    rotation_matrix[3][2] = -(cos(yaw_angle) * sin(roll_angle)) + cos(roll_angle) * sin(yaw_angle) * sin(pitch_angle);
    rotation_matrix[3][3] = cos(roll_angle) * cos(pitch_angle);
}