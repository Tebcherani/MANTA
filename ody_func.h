/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: ody_func.h,v 2.22 1998/02/03 04:21:57 bamoran Exp $
 *----------------------------------------------------------------------*/

#ifndef ODY_FUNC_H
#define ODY_FUNC_H

#include <stdio.h>
                                /* main.c */
void sighand( int );
void exit_gracefully( int );
int io();
int init_io();
void direct_control( void );
void simple_dead_reckoning( void );
                                /* update.c */
void update( int, double );
void update_add( int, double );
int init_data_structure( );

                                /* dynamic_control.c */
void dynamic_control( void );
double limit( double, double, double );
double sgn( double value );

                                /* layered_control.c */
void layered_control( void );
void resolve( int priority );
int shut_down( int );
int load_modem( char *);
int display_com( int, char *);
int manage_time( struct behavior *b, double duration );
double modPi( double );

				/* load_mission.c */
int load_state( char * );

				/* thruster.c */
int init_thruster( int );
int control_thruster();

				/* fins.c */
int init_fin( int );
int init_rudder( void );
int init_elevator( void );
int fin_command( int, double, char * );
int rudder_command( double, char * );
int elevator_command( double, char * );
int control_fin( int,  double );
int query_fin( int );
int query_rudder();
int query_elevator();
int get_fin_location(int, int, int *);
int reset_fin_glitch( int );
int check_for_fin_glitch( int );
int check_for_rudder_glitch();
int check_for_elev_glitch();
int query_fin_HE( int, int* );
int get_fin_reply( int, int );

				/* parosci.c */
int init_parosci();
double pres_depth( double, double );
int read_parosci();
void dump_parosci_buffer();

				/* log_data.c */
int log_data( FILE *datafile );
FILE *init_data_logging( char *DataFileName );
void init_tab( void );
void update_tab( float depth );
int write_tab( void );

			/* load_fault.c */
int         load_faults        ( const char *fname );
void        apply_faults       ( void );
const char *get_fault_file_name( void );

int send( int port, char *string );
int millisec( void );
int csend( int, char *, int );
int confirm( int, char *, int );
int confirm2(int, char *, char *, int );
int get_reply( int, char *, int );
int clear_port( int );
int open_serial_port( char *, int baud, int parity );

				/* sim.c — full prototypes required by modern GCC */
void motion( double *e, double *p, double Amps, double Del13,
	     double Del24, double dt );
void hydro( double *, double *, double, double, double, double );
void fins( double *, double, double );
void init_simulation();
double thruster_torque( double real_amps, double w_speed );
double thruster_force( double real_amps, double w_speed );
double act_model( double des_angle, double angle, double dt );
void   mmul( int na, int nb, int nc, int l, int m, int n,
             double a[][7], double b[][7], double c[][7] );
int    matinv( double a[][7], double ainv[][7], int n, double adum[][7] );
double hysteresis( double, double, double, double, double );
int get_hydro( );
#ifdef _OSK			/* OS-9 has no drand48() */
double drand48();
#endif

				/* sensor_load.c */
void simSensors( void );
void init_rotation( void );
void load_accel( void );
void load_magnetic( void );
void load_angular_rate( void );
void load_depth( void );
void load_water_speed( void );
void load_sonar( void );
void load_water_param( void );
void calc_rotation_matrix( void );
double *e2b( double *xe );
double *b2e( double *xb );
void simLBL4EKF();
void sensor_processing( void );

				/* function prototypes from nrad.c */
int init_nrad_sensor();    
int nrad_detect_mode();
int nrad_track_mode();
int read_nrad( int *x, int *y, int *v, int *d );

				/* kvh.c */
int init_kvh_dgc( );
int init_kvh_dgi( );
int convert_kvh_string( );
int shut_down_kvh( );
int read_kvh_dgc( void );
int read_kvh_dgi( void );

				/* network.c */
int communicate();

				/* sontek.c */
#ifdef SONTEK_ADV
int initialize_adv( void );
int read_adv_ascii( void );
int terminate_adv( void );
#endif /* SONTEK */

#ifdef PSA_916			/* Datasonics PSA-916 Altimeter */
int init_psa916( void );
int start_psa916( void );
int read_psa916( void );
#endif /* PSA_916 */

				/* docking.c */
int appendDockFile( const char *string );

#endif /* ODY_FUNC_H */
