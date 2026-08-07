#include <stdlib.h>

#include "ody_defs.h"
#include "odyssey_data_struct.h"
#include "odyssey_sensors.h"
#include "vehicle.h"

/*--------------------------------------------------------------------------*
  Calculate sensed LBL readings...                         

  jl: 10/5/94 making changes for spherical lbl, reporting round-trip
  travel times instead of arrival times 
 
  jl: 10/25/94 changing so that times 1,2,3,4 are used 
  (from now on, M_LBL_0T corresponds to the bottom return)

  jv: 7/13/95 simulating the times of flight output as observed in JdF'95
  data. 
  The tof are set to 0 except:
      * at the sampling time corresponding to the vehicle ping and the
        sampling time immediately following, where the tof are set to -1.0
      * at the sampling time corresponding to the LBL timeout and the
        sampling time immediately following where the tof are set to the
        measured values.
  M_LBL_PING_TIME is set to 0 except:
      * at the sampling time corresponding to the vehicle ping and the
        sampling time immediately following, where M_LBL_PING_TIME is set
	to the sampling time corresponding to the vehicle ping.
  if a tof drops out 0.0 is written in the structure.
 *--------------------------------------------------------------------------*/

void update_lbl_sensor() {
    static int uninitialized = 1;
    static int num_beacons, update_times;
    static int gotReply[5], cycle_number = 0;
    static double memo1[5];
    static double north[5], east[5], depth[5], tat[5];
    static double last_cycle_time, cycle_time, lbl_cycle, next_ping = 0;
    static double sound_speed, timeout, percent_bias, percent_dropout;
    static double bias_amplitude, noise_amplitude;
    static double north_ping, east_ping, depth_ping;

    int i, j, offset;
    double dist1, dist2, time_of_flight;
    double since_cycle, present_time;
    double noise, bias;

    if (uninitialized) {
        lbl_cycle = sensor(U_LBL_CYCLE);
        sound_speed = sensor(U_SPEED_OF_SOUND);
        num_beacons = (int) sensor(U_LBL_BEACON_NUMBER);
        timeout = sensor(U_LBL_TIMEOUT);
        percent_bias = sensor(REAL_LBL_FRAC_BIAS);
        percent_dropout = sensor(REAL_LBL_FRAC_DROPOUT);
        bias_amplitude = sensor(REAL_LBL_BIAS_AMPLITUDE);   // mSec
        noise_amplitude = sensor(REAL_LBL_TIMING_NOISE);    // mSec
        last_cycle_time = 0.0;

        for (i = 1, offset = 0; i <= num_beacons; i++) {
            gotReply[i] = 0;
			
            // Store beacon coordinates
            north[i] = sensor(U_LBL_1NORTH + offset);
            east[i]  = sensor(U_LBL_1EAST  + offset);
            depth[i] = sensor(U_LBL_1DEPTH + offset);
	        tat[i]   = sensor( U_LBL_1TAT   + offset );
            offset += U_LBL_2NORTH - U_LBL_1NORTH;       
        }
        uninitialized = 0;
    }
	
    // PING! (only if c_lbl_control is on, but update next_ping regardless)
    present_time = sensor(M_PRESENT_TIME);
    if (present_time >= next_ping) {
	    next_ping += sensor(U_LBL_CYCLE);
        if (sensor(C_LBL_CONTROL) > 0) {
            update(M_LBL_PING_TIME, present_time);
            for (i = 0; i <= num_beacons; i++) update(M_LBL_0T+i, -1.0);

            cycle_number++;
            update_times = 1;
            north_ping = sensor(REAL_X);
            east_ping = sensor(REAL_Y);
            depth_ping = sensor(REAL_Z);
            memset(gotReply, 0, sizeof(gotReply));
        }
    }
	
    // Compute time of ping and time elapsed since ping
    cycle_time = sensor(M_LBL_PING_TIME);
    since_cycle = present_time - cycle_time;

	// If between ping time and before LBL timeout
    if ((present_time > cycle_time) && (since_cycle < timeout) && (cycle_number > 0)) {
        for(i = 1; i <= num_beacons; i++) {
            if (!gotReply[i]) {
				// Distance from position at ping to beacon
                dist1  = (north_ping-north[i])*(north_ping-north[i]);
                dist1 += (east_ping-east[i])*(east_ping-east[i]);
                dist1 += (depth_ping-depth[i])*(depth_ping-depth[i]);
                dist1  = sqrt(dist1);

				// Add distance from beacon to current position
                dist2  = (sensor(REAL_X)-north[i])*(sensor(REAL_X)-north[i]);
                dist2 += (sensor(REAL_Y)-east[i])*(sensor(REAL_Y)-east[i]);
                dist2 += (sensor(REAL_Z)-depth[i])*(sensor(REAL_Z)-depth[i]);
                dist2  = sqrt(dist2);
				
                // Time required to travel distance1 + distance2 + tat
                time_of_flight = (dist1+dist2)/sound_speed + tat[i];

				// Add noise
                noise = std_normal_random_num()*noise_amplitude/1000.0;
                time_of_flight += noise;

				// Add bias
                if (drand48() < percent_bias) {
                    bias = drand48()*bias_amplitude/1000.0;
                    time_of_flight += bias;
                }

                if (time_of_flight < since_cycle) {
                    gotReply[i] = 1;
                    if (drand48() < percent_dropout) memo1[i] = 0.0; // Beacon i drops out
                    else memo1[i] = time_of_flight; // Beacon i gets calculated value
                }
            }
        }
    }
	
    // If LBL has timed out: update structure
    if (update_times && (since_cycle > timeout)) {
        for (i = 1; i <= num_beacons; i++) update(M_LBL_0T+i, memo1[i]);
        update(LBL_NEWDATA, 1);
        update_times = 0;
    }
}