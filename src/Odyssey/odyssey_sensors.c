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
    double psi, theta, phi; // Yaw, pitch and roll angles, respectively
    double rotation_matrix[4][4];
  
    psi = sensor(REAL_PSI);
    theta = sensor(REAL_THETA);
    phi = sensor(REAL_PHI);
    
    /* To find the body frame representation of a vector in the earth frame do:                         */
    /* xb[i] = rotation_matrix[i][1]*xe[1] + rotation_matrix[i][2]*xe[2] + rotation_matrix[i][3]*xe[3]  */
    /*                                                                                                  */
    /* To find the earth frame representation of a vector in the body frame do:                         */
    /* xe[i] = rotation_matrix[1][i]*xb[1] + rotation_matrix[2][i]*xb[2] + rotation_matrix[3][i]*xb[3]  */
    /*                                                                                                  */
    
    rotation_matrix[1][1] = cos(psi)*cos(theta);
    rotation_matrix[1][2] = cos(theta)*sin(psi);
    rotation_matrix[1][3] = -sin(theta);
    
    rotation_matrix[2][1] = -(cos(phi)*sin(psi)) + cos(psi)*sin(phi)*sin(theta);
    rotation_matrix[2][2] = cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta);
    rotation_matrix[2][3] = cos(theta)*sin(phi); 
    
    rotation_matrix[3][1] = sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta);
    rotation_matrix[3][2] = -(cos(psi)*sin(phi)) + cos(phi)*sin(psi)*sin(theta);
    rotation_matrix[3][3] = cos(phi)*cos(theta);
}