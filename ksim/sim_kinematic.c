/*----------------------------------------------------------------------*
   Odyssey Kinematic Simulator — Steady-State Motion Model
   
   Eliminates hydrodynamics and integrates kinematic state directly
   from layered-control outputs. Algebraic steady-state model for:
   - Pitch from elevator deflection and buoyancy moment
   - Vertical velocity from pitch and net buoyancy
   - Yaw rate from rudder deflection
   - Position integration via heading and speed
   
   Interfaces identically to sim.c: init_simulation() and motion()
   so no changes needed to sim_main.c or the control pipeline.
 *----------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "vehicle.h"
#include "diagnostic.h"
#include "paths.h"

/* Kinematic simulator state and coefficients */
struct ksim_state {
    double rho;              /* water density (kg/m^3) */
    double g;                /* gravity (m/s^2) */
    double mass;             /* vehicle mass (kg) */
    double volume;           /* displaced volume (m^3) */
    double cg_cb_offset;     /* CG to CB distance (m), positive if CB above CG */
    
    /* Pitch control coefficients */
    double ref_area;         /* planform area S (m^2) */
    double ref_length;       /* reference length c (m) */
    double Cm_theta;         /* pitching moment coeff vs pitch (nondimensional) */
    double Cm_delta;         /* pitching moment coeff vs elevator (nondim/rad) */
    double CL_theta;         /* vertical force coeff vs pitch (nondim/rad) */
    double vertical_drag_coeff;  /* vertical drag k_v (N·s/m) */
    
    /* Yaw control coefficients */
    double K_rudder;         /* rudder → yaw-rate gain (rad/s per rad per m/s) */
    double D_rudder;         /* rudder yaw damping (s^-1) */
    double tau_rudder;       /* rudder dynamics time constant (s) */
    
    /* Speed control */
    double tau_speed;        /* speed response time constant (s) */
    double max_speed;        /* max achievable speed (m/s) */
    
    /* Pitch dynamics */
    double tau_pitch;        /* pitch response time constant (s) */
    double max_pitch;        /* max pitch (rad) */
    
    /* Heading control */
    double max_yaw_rate;     /* max yaw rate (rad/s) */
    double K_hdg_turn;       /* heading setpoint controller gain */
    
    /* Depth to pitch mapping (from dynamic_control) */
    double K_depth_to_pitch; /* depth error → pitch gain (rad/m) */
    
    /* Drop weight masses */
    double drop_weight_1_mass;  /* drop weight 1 mass (kg) */
    double drop_weight_2_mass;  /* drop weight 2 mass (kg) */
    
} ksim;

/* Running state */
static double U_cmd = 0.0;      /* commanded forward speed (m/s) */
static double theta_ss = 0.0;   /* steady-state pitch */
static double psi_des = 0.0;    /* desired heading for heading-mode */
static double r = 0.0;          /* yaw rate (rad/s) */

/*----------------------------------------------------------------------*
  Load kinematic coefficients from ksim.cfg
 *----------------------------------------------------------------------*/

static int load_ksim_config( const char *fname )
{
    FILE *fp;
    char line[256], key[64];
    double val;
    
    fp = fopen(fname, "r");
    if (!fp) {
        sprintf( msg_buf, "ksim: cannot open %s (using hardcoded defaults)\n", fname );
        write_message( TERM | SLOG );
        return -1;
    }
    
    sprintf( msg_buf, "ksim: loading config from %s\n", fname );
    write_message( TERM | SLOG );
    
    while (fgets(line, sizeof(line), fp)) {
        if (line[0] == '#' || line[0] == '\n') continue;
        
        if (sscanf(line, "%s %lf", key, &val) == 2) {
            if      (!strcmp(key, "rho"))                ksim.rho = val;
            else if (!strcmp(key, "mass"))               ksim.mass = val;
            else if (!strcmp(key, "volume"))             ksim.volume = val;
            else if (!strcmp(key, "cg_cb_offset"))       ksim.cg_cb_offset = val;
            else if (!strcmp(key, "ref_area"))           ksim.ref_area = val;
            else if (!strcmp(key, "ref_length"))         ksim.ref_length = val;
            else if (!strcmp(key, "Cm_theta"))           ksim.Cm_theta = val;
            else if (!strcmp(key, "Cm_delta"))           ksim.Cm_delta = val;
            else if (!strcmp(key, "CL_theta"))           ksim.CL_theta = val;
            else if (!strcmp(key, "vertical_drag_coeff")) ksim.vertical_drag_coeff = val;
            else if (!strcmp(key, "K_rudder"))           ksim.K_rudder = val;
            else if (!strcmp(key, "D_rudder"))           ksim.D_rudder = val;
            else if (!strcmp(key, "tau_rudder"))         ksim.tau_rudder = val;
            else if (!strcmp(key, "tau_speed"))          ksim.tau_speed = val;
            else if (!strcmp(key, "max_speed"))          ksim.max_speed = val;
            else if (!strcmp(key, "tau_pitch"))          ksim.tau_pitch = val;
            else if (!strcmp(key, "max_pitch"))          ksim.max_pitch = val;
            else if (!strcmp(key, "max_yaw_rate"))       ksim.max_yaw_rate = val;
            else if (!strcmp(key, "K_hdg_turn"))         ksim.K_hdg_turn = val;
            else if (!strcmp(key, "K_depth_to_pitch"))   ksim.K_depth_to_pitch = val;
            else if (!strcmp(key, "drop_weight_1_mass")) ksim.drop_weight_1_mass = val;
            else if (!strcmp(key, "drop_weight_2_mass")) ksim.drop_weight_2_mass = val;
        }
    }
    
    fclose(fp);
    return 0;
}

/*----------------------------------------------------------------------*
  Initialize kinematic simulator
 *----------------------------------------------------------------------*/

void init_simulation( void )
{
    /* Hardcoded defaults (typical AUV parameters) */
    ksim.rho = 1025.0;
    ksim.g = 9.80665;
    ksim.mass = 100.0;
    ksim.volume = 0.098;
    ksim.cg_cb_offset = 0.001;     /* ~1mm CB above CG for slight restoring */
    
    ksim.ref_area = 0.05;
    ksim.ref_length = 0.5;
    ksim.Cm_theta = -0.5;           /* negative: pitch-restoring */
    ksim.Cm_delta = 0.3;
    ksim.CL_theta = 2.0;            /* lift per unit pitch */
    ksim.vertical_drag_coeff = 50.0;
    
    ksim.K_rudder = 0.2;
    ksim.D_rudder = 0.1;
    ksim.tau_rudder = 0.5;
    
    ksim.tau_speed = 2.0;
    ksim.max_speed = 2.5;
    ksim.tau_pitch = 1.0;
    ksim.max_pitch = 0.436;         /* ~25 degrees */
    
    ksim.max_yaw_rate = 0.5;
    ksim.K_hdg_turn = 0.5;
    ksim.K_depth_to_pitch = 0.02;   /* 0.02 rad/m depth error */
    
    ksim.drop_weight_1_mass = 0.0;
    ksim.drop_weight_2_mass = 0.0;
    
    /* Try to load from ksim/ksim.cfg under the simulator root. */
    {
        char cfg_path[ODY_PATH_MAX];
        ody_join_path( cfg_path, sizeof(cfg_path), ODY_ROOT,
                       "ksim/ksim.cfg" );
        load_ksim_config( cfg_path );
    }
    
    /* Initialize running state */
    U_cmd = 0.5;
    theta_ss = 0.0;
    psi_des = 0.0;
    r = 0.0;
    
    sprintf( msg_buf, "ksim initialized: m=%.2f, vol=%.3f, rho=%.1f\n",
             ksim.mass, ksim.volume, ksim.rho );
    write_message( TERM | SLOG );
}

/*----------------------------------------------------------------------*
  Steady-state pitch computation from elevator and buoyancy
 *----------------------------------------------------------------------*/

static double compute_pitch( double U, double elevator_angle, double net_buoyancy )
{
    double q, M_buoy_coeff, denom, theta;
    
    if (U < 0.01) U = 0.01;  /* avoid divide-by-zero */
    
    q = 0.5 * ksim.rho * U * U;
    M_buoy_coeff = net_buoyancy * ksim.cg_cb_offset;
    denom = q * ksim.ref_area * ksim.ref_length * ksim.Cm_theta + M_buoy_coeff;
    
    if (fabs(denom) < 1e-6) {
        theta = elevator_angle * 0.1;  /* fallback: simple proportional */
    } else {
        theta = -(q * ksim.ref_area * ksim.ref_length * ksim.Cm_delta * elevator_angle) / denom;
    }
    
    /* Limit pitch */
    if (theta > ksim.max_pitch) theta = ksim.max_pitch;
    if (theta < -ksim.max_pitch) theta = -ksim.max_pitch;
    
    return theta;
}

/*----------------------------------------------------------------------*
  Compute vertical velocity from pitch and buoyancy
 *----------------------------------------------------------------------*/

static double compute_vertical_velocity( double U, double pitch, double net_buoyancy )
{
    double q, L_theta, vz;
    
    if (U < 0.01) U = 0.01;
    
    q = 0.5 * ksim.rho * U * U;
    L_theta = q * ksim.ref_area * ksim.CL_theta * pitch;
    
    vz = (net_buoyancy + L_theta) / ksim.vertical_drag_coeff;
    
    return vz;
}

/*----------------------------------------------------------------------*
  Compute yaw rate from rudder command or heading setpoint
 *----------------------------------------------------------------------*/

static double compute_yaw_rate( double U, int heading_mode, double heading_cmd, double psi_current )
{
    double delta_rudder, r_ss;
    
    if (U < 0.01) U = 0.01;
    
    if (heading_mode == 0) {
        /* Rudder command: map to yaw rate */
        delta_rudder = heading_cmd;
        r_ss = ksim.K_rudder * delta_rudder * U;
    } else {
        /* Heading setpoint: use heading controller to compute desired yaw rate */
        double hdg_error = heading_cmd - psi_current;
        /* Wrap to [-pi, pi] */
        while (hdg_error > M_PI) hdg_error -= 2*M_PI;
        while (hdg_error < -M_PI) hdg_error += 2*M_PI;
        
        r_ss = ksim.K_hdg_turn * hdg_error;
    }
    
    /* Limit yaw rate */
    if (r_ss > ksim.max_yaw_rate) r_ss = ksim.max_yaw_rate;
    if (r_ss < -ksim.max_yaw_rate) r_ss = -ksim.max_yaw_rate;
    
    return r_ss;
}

/*----------------------------------------------------------------------*
  Main kinematic motion integrator
  
  Called by sim_main every 10 control cycles.
  e[] = [1:U, 2:V, 3:W, 4:P, 5:Q, 6:R]  (not used; we compute directly)
  p[] = [1:X, 2:Y, 3:Z, 6:PSI]
  dt = time step (s)
 *----------------------------------------------------------------------*/

void motion( double e[], double p[], double Amps, double Del13,
             double Del24, double dt )
{
    double U, vz, theta, psi, r_cmd, cos_theta;
    double cos_psi, sin_psi, X_dot, Y_dot, Z_dot;
    double net_buoyancy, current_north, current_east;
    int depth_mode, heading_mode, speed_mode;
    
    /* Read current command modes from resolved command structure */
    heading_mode = vvp->c[0].heading_mode;
    depth_mode = vvp->c[0].depth_mode;
    speed_mode = vvp->c[0].speed_mode;
    
    /* Get current speed from sensor (or use commanded) */
    U = sensor(REAL_U);
    if (U < 0.01) U = 0.01;
    
    /* Compute net buoyancy force */
    double mass_active = ksim.mass;
    if (sensor(C_WEIGHT1_DROP) > 0.5) mass_active -= ksim.drop_weight_1_mass;
    if (sensor(C_WEIGHT2_DROP) > 0.5) mass_active -= ksim.drop_weight_2_mass;
    
    double buoyancy_force = ksim.rho * ksim.g * ksim.volume - mass_active * ksim.g;
    net_buoyancy = buoyancy_force;
    
    /* Compute commanded speed and update it with first-order dynamics */
    if (speed_mode == 1 || speed_mode == 2 || speed_mode == 3) {
        U_cmd = vvp->c[0].speed[0];
        if (U_cmd > ksim.max_speed) U_cmd = ksim.max_speed;
        if (U_cmd < 0.0) U_cmd = 0.0;
    }
    
    /* Simple first-order speed response */
    double U_new = U + (U_cmd - U) * (dt / ksim.tau_speed);
    update(REAL_U, U_new);
    
    /* Compute pitch based on depth_mode */
    if (depth_mode == 2) {
        /* Commanded pitch directly */
        theta = vvp->c[0].depth[0];
    } else if (depth_mode == 1) {
        /* Depth command: convert to pitch via error mapping */
        double depth_error = vvp->c[0].depth[0] - sensor(M_DEPTH);
        theta = -ksim.K_depth_to_pitch * depth_error;
    } else {
        /* Elevator command (mode 0) */
        theta = compute_pitch(U_new, Del24, net_buoyancy);
    }
    
    /* Apply first-order pitch dynamics */
    theta_ss = theta_ss + (theta - theta_ss) * (dt / ksim.tau_pitch);
    if (theta_ss > ksim.max_pitch) theta_ss = ksim.max_pitch;
    if (theta_ss < -ksim.max_pitch) theta_ss = -ksim.max_pitch;
    
    update(REAL_THETA, theta_ss);
    
    /* Compute vertical velocity from pitch and buoyancy */
    vz = compute_vertical_velocity(U_new, theta_ss, net_buoyancy);
    update(REAL_W, vz);

    /* Compute vertical acceleration for logging */
    update(REAL_WD, (vz - sensor(REAL_W)) / dt);

    /* Yaw rate computation */
    psi = sensor(REAL_PSI);
    r_cmd = compute_yaw_rate(U_new, heading_mode, vvp->c[0].heading[0], psi);
    
    /* Apply first-order rudder dynamics */
    r = r + (r_cmd - r) * (dt / ksim.tau_rudder);
    update(REAL_R, r);
    
    /* Update attitude and position */
    cos_theta = cos(theta_ss);
    cos_psi = cos(psi);
    sin_psi = sin(psi);
    
    /* Forward velocity components */
    update(REAL_V, 0.0);  /* zero lateral velocity for nominal motion */
    
    /* Position derivatives */
    X_dot = U_new * cos_theta * cos_psi;
    Y_dot = U_new * cos_theta * sin_psi;
    Z_dot = -vz;  /* depth increases downward */
    
    /* Add current if present */
    current_north = sensor(REAL_CURRENT_NORTH);
    current_east = sensor(REAL_CURRENT_EAST);
    X_dot += current_north;
    Y_dot += current_east;
    
    /* Integrate position */
    p[1] += X_dot * dt;
    p[2] += Y_dot * dt;
    p[3] += Z_dot * dt;
    
    /* Integrate heading */
    psi += r * dt;
    while (psi > M_PI) psi -= 2*M_PI;
    while (psi < -M_PI) psi += 2*M_PI;
    p[6] = psi;
    
    /* Update REAL_* fields for sensors */
    update(REAL_X, p[1]);
    update(REAL_Y, p[2]);
    update(REAL_Z, p[3]);
    update(REAL_PSI, psi);
    
    /* Compute roll (assume zero) */
    update(REAL_PHI, 0.0);
    update(REAL_P, 0.0);
    update(REAL_Q, 0.0);
    
    /* Update velocity derivatives */
    update(REAL_UD, 0.0);  /* assume quasi-steady */
    update(REAL_VD, 0.0);
    
    /* Update pitch rate (approximate from pitch change) */
    static double theta_prev = 0.0;
    update(REAL_QD, (theta_ss - theta_prev) / dt);
    theta_prev = theta_ss;
    
    /* Update excess buoyancy for logging */
    update(REAL_EXCESS_BUOYANCY, net_buoyancy);
}
