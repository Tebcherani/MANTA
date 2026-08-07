#include <math.h>

#include "angle_utils.h"
#include "common_defs.h"
#include "ody_defs.h"
#include "odyssey_data_struct.h"
#include "odyssey_sensors.h"
#include "random_utils.h"
#include "vehicle.h"

static void inclinometer_low_pass_filter(double s[2], double sf[2]);
static void update_acceleration(double *aax, double *aay);
static void update_angular_rate(void);
static void update_compass_yaw(double aax, double aay);
static void update_inclinometers(double ax, double ay);
static void update_heading(void);

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

static void update_acceleration(double *aax, double *aay) {
    static double dx=1.390,dy=0.108,dz=0.076;
    double ax,ay,az;
    double p,q,r,pdot,qdot,rdot;
    double u,v,w,udot,vdot,wdot;
    double theta,phi;
		
    // Load real velocities and acceleration (linear and angular)
    u    = sensor(REAL_U);     v = sensor(REAL_V);     w = sensor(REAL_W);
    udot = sensor(REAL_UD); vdot = sensor(REAL_VD); wdot = sensor(REAL_WD);
    p    = sensor(REAL_P);     q = sensor(REAL_Q);     r = sensor(REAL_R);
    pdot = sensor(REAL_PD); qdot = sensor(REAL_QD); rdot = sensor(REAL_RD);

	// Non-gravitational acceleration at location of AHRS in vehicle frame
    ax = udot + q*w-r*v + qdot*dz-rdot*dy - (q*q+r*r)*dx + p*q*dy + p*r*dz;
    ay = vdot + r*u-p*w + rdot*dx-pdot*dz - (p*p+r*r)*dy + p*q*dx + q*r*dz;
    az = wdot + p*v-q*u + pdot*dy-qdot*dx - (p*p+q*q)*dz + q*r*dy + p*r*dx;

	// Compute non-gravitational acc in local horizontal plane for compass
    theta = sensor(REAL_THETA);
    phi = sensor(REAL_PHI);
    *aax = cos(theta)*ax + sin(phi)*sin(theta)*ay + cos(phi)*sin(theta)*az;
    *aay = cos(phi)*ay - sin(phi)*az;

	// Add gravity
    ax = ax + sin(theta)*GRAVITY;
    ay = ay - cos(theta)*sin(phi)*GRAVITY;
    az = az - cos(theta)*cos(phi)*GRAVITY;

    update_inclinometers(ax, ay);
}

static void update_angular_rate(void) {
    double real_yaw_rate, real_roll_rate, real_pitch_rate;
    double meas_yaw_rate, meas_roll_rate, meas_pitch_rate;
    double yaw_rate_bias = -8.7266e-4, pitch_rate_bias = 8.7266e-4, roll_rate_bias = 4.3663e-4;
    double sensor_noise, sensor_noise_level = 0.00001745;

    real_roll_rate = sensor(REAL_P);
    sensor_noise = sensor_noise_level * std_normal_random_num(); 
    meas_roll_rate = real_roll_rate + sensor_noise + roll_rate_bias;
    update(M_ROLL_RATE, meas_roll_rate);

    real_pitch_rate = sensor(REAL_Q);
    sensor_noise = sensor_noise_level * std_normal_random_num(); 
    meas_pitch_rate = real_pitch_rate + sensor_noise + pitch_rate_bias;
    update(M_PITCH_RATE, meas_pitch_rate);

    real_yaw_rate = sensor(REAL_R);
    sensor_noise = sensor_noise_level * std_normal_random_num(); 
    meas_yaw_rate = real_yaw_rate + sensor_noise + yaw_rate_bias;
    update(M_YAW_RATE, meas_yaw_rate);
}

/*-------------------------------------------------------------------------*
(JV 5/31/95)
Simulation of the magnetic heading measured by the KVH DGC
  . a 3-axis magnetometer (s3am) suspended in the DGC housing is considered.
  . the s3am pitches and rolls with respect to the horizontal because
    of the vehicle accelerations (linear and angular)
  . to compute the s3am pitch and roll, the motion of a 2D linear damped
    oscillator is first determined and the x-y displcaements converted
    into angles.
*-------------------------------------------------------------------------*/
static void update_compass_yaw(double aax, double aay) {
    static int first=1;
    static double compass_noise=0.001745; /* 0.1 deg */
    static double dx=0.0, dy=0.0, dxp=0.0, dyp=0.0;
    static double beta, omega0, omega1;
    double dt=0.02, mass=0.1, kx=100.0, bxp=0.5, length=0.015;
    double psi, theta, phi;
    double mx, my;
    double mg_yaw;
    double cst1, cst2, cc, ss;
    double Mmg[4][4];

    if (first) {
       first = 0;
       beta = bxp/(2.0*mass); 
       omega0=sqrt(kx/mass);
       omega1 = sqrt(omega0*omega0-beta*beta);
    }

    // Compute pitch and roll of mg sensor given accelerations 
    cc = cos(omega1*dt);
    ss = sin(omega1*dt);

    cst1 = aax/(omega0*omega0)+dx;
    cst2 = (dxp+beta*(aax/(omega0*omega0)+dx))/omega1;
    dx = exp(-beta*dt)*(cst1*cc+cst2*ss)-aax/(omega0*omega0);
    dxp = -beta*exp(-beta*dt)*(cst1*cc+cst2*ss) + exp(-beta*dt)*(-cst1*omega1*ss+cst2*omega1*cc);

    cst1 = aay/(omega0*omega0)+dy;
    cst2 = (dyp+beta*(aay/(omega0*omega0)+dy))/omega1;
    dy = exp(-beta*dt)*(cst1*cc+cst2*ss)-aay/(omega0*omega0);
    dyp = -beta*exp(-beta*dt)*(cst1*cc+cst2*ss) + exp(-beta*dt)*(-cst1*omega1*ss+cst2*omega1*cc);

    theta = dx/length; phi = -dy/length;

    // Compute the magnetic readings given the orientation of mg sensor 
    psi=sensor(REAL_PSI);
    Mmg[1][1] = cos(psi)*cos(theta);
    Mmg[1][2] = cos(theta)*sin(psi);
    Mmg[1][3] = -sin(theta);
    Mmg[2][1] = -(cos(phi)*sin(psi)) + cos(psi)*sin(phi)*sin(theta);
    Mmg[2][2] = cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta);
    Mmg[2][3] = cos(theta)*sin(phi); 
    Mmg[3][1] = sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta);
    Mmg[3][2] = -(cos(psi)*sin(phi)) + cos(phi)*sin(psi)*sin(theta);
    Mmg[3][3] = cos(phi)*cos(theta);

    mx = Mmg[1][1]*EARTH_MAGNETIC_NORTH + Mmg[1][2]*EARTH_MAGNETIC_EAST + Mmg[1][3]*EARTH_MAGNETIC_DOWN;
    my = Mmg[2][1]*EARTH_MAGNETIC_NORTH + Mmg[2][2]*EARTH_MAGNETIC_EAST + Mmg[2][3]*EARTH_MAGNETIC_DOWN;

    // Compute heading from readings 
    if ((mx>0.0) && (my<=0.0)) mg_yaw = -atan(my/mx);
    else if ((mx<=0.0) && (my<0.0)) mg_yaw = atan(mx/my)+PI/2.0;
    else if ((mx<0.0) && (my>=0.0)) mg_yaw = -atan(my/mx)+PI;
    else if ((mx>=0.0) && (my>0.0)) mg_yaw = atan(mx/my)+3.0*PI/2.0;
    else mg_yaw = 0.0;

    // Add noise and write to the structure
    mg_yaw += compass_noise*std_normal_random_num();

    mg_yaw -= sensor(UO_MAGNETIC_VARIATION);

    update(M_COMPASS_YAW,mg_yaw);
}

static void update_heading(void) {
    double meas_yaw, meas_pitch, meas_roll;
    double meas_magnetic_variation, corrected_yaw;

    meas_yaw = sensor(M_COMPASS_YAW);
    meas_magnetic_variation = sensor(UO_MAGNETIC_VARIATION);
    corrected_yaw = PI + wrap_angle(meas_yaw + meas_magnetic_variation - PI);
    update(M_YAW, corrected_yaw);

    meas_pitch = sensor(M_INCLINO_PITCH);
    update(M_PITCH, meas_pitch);
    
    meas_roll = sensor(M_INCLINO_ROLL);
    update(M_ROLL, meas_roll);
}

static void update_inclinometers( double ax, double ay ) {
    double inclino_noise=0.001745; /* 0.1 degree */
    double raw_pitch_roll[2],filtered_pitch_roll[2];

    raw_pitch_roll[0]=asin(ax/GRAVITY);
    raw_pitch_roll[1]=-asin(ay/(GRAVITY*cos(raw_pitch_roll[0])));
    inclinometer_low_pass_filter(raw_pitch_roll,filtered_pitch_roll);

    filtered_pitch_roll[0]+=inclino_noise*std_normal_random_num();
    filtered_pitch_roll[1]+=inclino_noise*std_normal_random_num();

    update(M_INCLINO_PITCH,filtered_pitch_roll[0]);
    update(M_INCLINO_ROLL,filtered_pitch_roll[1]);
}

void update_ahrs_sensors(void) {
    double aax, aay;

    update_angular_rate();
    update_acceleration(&aax, &aay);
    update_compass_yaw(aax, aay);
    update_heading();
}