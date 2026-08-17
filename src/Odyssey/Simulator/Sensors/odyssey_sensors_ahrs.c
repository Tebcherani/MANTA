#include <math.h>

#include "angle_utils.h"
#include "odyssey.h"
#include "odyssey_sensors.h"
#include "random_utils.h"
#include "sensor_compass.h"
#include "world_sim.h"

#include "vehicle.h"

static void inclinometer_low_pass_filter(double s[2], double sf[2]);
static void update_acceleration(const world_state_t *, double *, double *, double *, double *);
static void update_angular_rate(void);
static void update_inclinometers(const world_state_t *, double, double);
static void update_heading(void);
static void update_odyssey_compass(const world_state_t *, double, double, double);

/*-------------------------------------------------------------------------
Low-pass filter for inclinometers simulation
cut-off frequency=1Hz, for a sampling frequency of 50Hz
J. Vaganay - 5/12/95
-------------------------------------------------------------------------*/
static void inclinometer_low_pass_filter( double s[2], double sf[2] ) {
    static int count=1;
    static double sp[2][3];  /* previous non filtered signals */
    static double sfp[2][3]; /* previous filtered signals */
    static double a[3],b[3]; /* filter coefficients */
    int i,j;

    for (i=0;i<=1;i++) {
        if(count==1) {
            sp[i][1]=s[i];
            sfp[i][1]=s[i];
            sf[i]=s[i];
            a[1]=-1.82269492519631; 
            a[2]=0.83718165127602;
            b[0]=0.00362168151493;  
            b[1]=0.00724336302986;
            b[2]=0.00362168151493;
        }
      
        if(count==2) {
            sp[i][2]=sp[i][1];
            sfp[i][2]=sfp[i][1];
            sp[i][1]=s[i];
            sfp[i][1]=s[i];
            sf[i]=s[i];
        }
        
        if(count>2) {
            sf[i]=b[0]*s[i];
            for(j=1;j<=2;j++)
                sf[i]+=b[j]*sp[i][j]-a[j]*sfp[i][j];
            for(j=2;j>1;j--) {
                sp[i][j]=sp[i][j-1];
                sfp[i][j]=sfp[i][j-1];
            }
            sp[i][1]=s[i];
            sfp[i][1]=sf[i];
        }
    }
    if(count<=2) count++;
}

static void update_acceleration(const world_state_t *world_state, double *ax, double *ay, double *aax, double *aay) {
    static double dx=1.390,dy=0.108,dz=0.076;
    double az;
    double p,q,r,pdot,qdot,rdot;
    double u,v,w,udot,vdot,wdot;
    double theta,phi;
		
    // Load real velocities and acceleration (linear and angular)
    u    = read_sensor(REAL_U);     v = read_sensor(REAL_V);     w = read_sensor(REAL_W);
    udot = read_sensor(REAL_UD); vdot = read_sensor(REAL_VD); wdot = read_sensor(REAL_WD);
    p    = read_sensor(REAL_P);     q = read_sensor(REAL_Q);     r = read_sensor(REAL_R);
    pdot = read_sensor(REAL_PD); qdot = read_sensor(REAL_QD); rdot = read_sensor(REAL_RD);

	// Non-gravitational acceleration at location of AHRS in vehicle frame
    *ax = udot + q*w-r*v + qdot*dz-rdot*dy - (q*q+r*r)*dx + p*q*dy + p*r*dz;
    *ay = vdot + r*u-p*w + rdot*dx-pdot*dz - (p*p+r*r)*dy + p*q*dx + q*r*dz;
    az = wdot + p*v-q*u + pdot*dy-qdot*dx - (p*p+q*q)*dz + q*r*dy + p*r*dx;

	// Compute non-gravitational acc in local horizontal plane for compass
    theta = read_sensor(REAL_THETA);
    phi = read_sensor(REAL_PHI);
    *aax = cos(theta) * (*ax) + sin(phi)*sin(theta) * (*ay) + cos(phi)*sin(theta)*az;
    *aay = cos(phi) * (*ay) - sin(phi)*az;

	// Add gravity
    *ax += sin(theta) * world_state->gravity;
    *ay -= cos(theta) * sin(phi) * world_state->gravity;
    az -= cos(theta) * cos(phi) * world_state->gravity;
}

static void update_angular_rate(void) {
    double real_yaw_rate, real_roll_rate, real_pitch_rate;
    double meas_yaw_rate, meas_roll_rate, meas_pitch_rate;
    double yaw_rate_bias = -8.7266e-4, pitch_rate_bias = 8.7266e-4, roll_rate_bias = 4.3663e-4;
    double sensor_noise, sensor_noise_level = 0.00001745;

    real_roll_rate = read_sensor(REAL_P);
    sensor_noise = sensor_noise_level * std_normal_random_num(); 
    meas_roll_rate = real_roll_rate + sensor_noise + roll_rate_bias;
    update_sensor(M_ROLL_RATE, meas_roll_rate);

    real_pitch_rate = read_sensor(REAL_Q);
    sensor_noise = sensor_noise_level * std_normal_random_num(); 
    meas_pitch_rate = real_pitch_rate + sensor_noise + pitch_rate_bias;
    update_sensor(M_PITCH_RATE, meas_pitch_rate);

    real_yaw_rate = read_sensor(REAL_R);
    sensor_noise = sensor_noise_level * std_normal_random_num(); 
    meas_yaw_rate = real_yaw_rate + sensor_noise + yaw_rate_bias;
    update_sensor(M_YAW_RATE, meas_yaw_rate);
}

static void update_odyssey_compass(const world_state_t *world_state, double accel_x, double accel_y, double timestep) {
    compass_state_t *compass;
    double compass_yaw;
    compass_input_t input;

    compass = get_odyssey_compass_state();

    input.true_yaw = read_sensor(REAL_PSI);
    input.accel_x = accel_x;
    input.accel_y = accel_y;
    input.magnetic_north = world_state->magnetic_north;
    input.magnetic_east = world_state->magnetic_east;
    input.magnetic_down = world_state->magnetic_down;
    input.timestep = timestep;

    compass_yaw = update_compass_yaw(compass, &input);
    compass_yaw -= read_sensor(UO_MAGNETIC_VARIATION);
    update_sensor(M_COMPASS_YAW, compass_yaw);
}

static void update_heading(void) {
    double meas_yaw, meas_pitch, meas_roll;
    double meas_magnetic_variation, corrected_yaw;

    meas_yaw = read_sensor(M_COMPASS_YAW);
    meas_magnetic_variation = read_sensor(UO_MAGNETIC_VARIATION);
    corrected_yaw = M_PI + wrap_angle(meas_yaw + meas_magnetic_variation - M_PI);
    update_sensor(M_YAW, corrected_yaw);

    meas_pitch = read_sensor(M_INCLINO_PITCH);
    update_sensor(M_PITCH, meas_pitch);
    
    meas_roll = read_sensor(M_INCLINO_ROLL);
    update_sensor(M_ROLL, meas_roll);
}

static void update_inclinometers(const world_state_t *world_state, double ax, double ay ) {
    double inclino_noise=0.001745; /* 0.1 degree */
    double raw_pitch_roll[2],filtered_pitch_roll[2];

    raw_pitch_roll[0]=asin(ax/world_state->gravity);
    raw_pitch_roll[1]=-asin(ay/(world_state->gravity*cos(raw_pitch_roll[0])));
    inclinometer_low_pass_filter(raw_pitch_roll,filtered_pitch_roll);

    filtered_pitch_roll[0]+=inclino_noise*std_normal_random_num();
    filtered_pitch_roll[1]+=inclino_noise*std_normal_random_num();

    update_sensor(M_INCLINO_PITCH,filtered_pitch_roll[0]);
    update_sensor(M_INCLINO_ROLL,filtered_pitch_roll[1]);
}

void update_ahrs_sensors(const world_state_t *world_state, double timestep) {
    double inclinometer_accel_x, inclinometer_accel_y;
    double compass_accel_x, compass_accel_y;

    update_angular_rate();
    update_acceleration(world_state,&inclinometer_accel_x, &inclinometer_accel_y, &compass_accel_x, &compass_accel_y);
    update_inclinometers(world_state, inclinometer_accel_x, inclinometer_accel_y);
    update_odyssey_compass(world_state, compass_accel_x, compass_accel_y, timestep);
    update_heading();
}