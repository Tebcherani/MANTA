#include <math.h>

#include "odyssey_data_struct.h"
#include "odyssey_nav.h"
#include "vehicle.h"

static double slant[NXP];	/* slant ranges derived from tt's */
static void ABE_LBL(void);

static void ABE_LBL(void) {
    fix_t fix;  // Result of computation
    static lbl_t lblenv;    // LBL parameters: xpndrs, sound speed, ring buffers, max ranges, pos est, convergence parms
    static int uninitialized = 1;
    static int numXP;   // Number active transponders
    double dx, dy;
    int ii, jj, offset, numGood;

    if (uninitialized) {
		// Store lbl environment
        lblenv.range_tol = read_sensor(LBL_RANGE_TOL);
        lblenv.fix_tol = read_sensor(LBL_FIX_ERR);
        lblenv.timeout = 99999;
        lblenv.min_angle = read_sensor(LBL_MIN_ANGLE);
        lblenv.alpha = 0.5;
        lblenv.num_steps_pinv = 10;
        lblenv.max_iter = 40;

		// Store beacon count as static int
	    numXP = (int) read_sensor(U_LBL_BEACON_NUMBER);
		
        // Xponder data for fix calcs
	    for (ii = 0, offset = 0; ii < numXP; ii++) {
            lblenv.xp[ii].x = read_sensor(U_LBL_1NORTH + offset);
            lblenv.xp[ii].y = read_sensor(U_LBL_1EAST  + offset);
            lblenv.xp[ii].z = read_sensor(U_LBL_1DEPTH + offset);
            lblenv.tat[ii]  = read_sensor(U_LBL_1TAT   + offset);
            lblenv.max_range[ii] = 4000;
            offset += U_LBL_2NORTH - U_LBL_1NORTH;
	    
            for (jj = 0; jj < LBL_NUM_RANGES; jj++) {
		        lblenv.range[ii][jj] = -100;
		        lblenv.time[ii][jj]  = -100;
	        }
	    
            lblenv.good_time[ii] = -100;
	        lblenv.good[ii] = RANGE_OLD;
	    }
				
        // Initialize Dana's struct to zero
	    memset(&fix, 0, sizeof(fix_t));
	    uninitialized = 0;
    }
	
    // Calculate DR updates, will be used in presence or absence of LBL hit
    dx = read_sensor(M_DR_DN);
    dy = read_sensor(M_DR_DE);

    if (read_sensor(LBL_NEWDATA) > 0) {
        for (ii = 0; ii < numXP; ii++) {
            if (read_sensor(M_LBL_1T + ii) > 0) {
                slant[ii] = 0.5*(read_sensor(M_LBL_1T + ii) - lblenv.tat[ii]);
                slant[ii] *= read_sensor(U_SPEED_OF_SOUND);
            } else slant[ii] = 0.;
        }

        lblenv.depth  = read_sensor(M_DEPTH);
        lblenv.xhat.x = read_sensor(M_POS_N);
        lblenv.xhat.y = read_sensor(M_POS_E);

	    compute_lbl_fix(slant, &lblenv, read_sensor(M_PRESENT_TIME), &fix, 1);

		// Log the fix and its status
	    update_sensor(LBL_FIX_ERR, lblenv.error);
	    update_sensor(LBL_FIXSTAT, fix.fix_status);

		// Indicate we've used the data
	    update_sensor(LBL_NEWDATA, 0);

		// Count and log good hits
	    for (ii = 0, numGood = 0; ii < numXP; ii++) {
            if (lblenv.good[ii] == RANGE_GOOD) {
                numGood++;
                update_sensor(M_LBL_VALID1 + ii, 1);
            } else update_sensor(M_LBL_VALID1 + ii, 0);
	    }

	    update_sensor(M_LBL_NUM_USED, numGood);

        if (fix.fix_status == NAV_GOOD) {
            update_sensor(M_LBL_NORTHFIX, fix.position.x);
            update_sensor(M_LBL_EASTFIX,  fix.position.y);

            dx += read_sensor(LBL_FIXGAIN)*(read_sensor(M_LBL_NORTHFIX) - read_sensor(M_POS_N));
            dy += read_sensor(LBL_FIXGAIN)*(read_sensor(M_LBL_EASTFIX) - read_sensor(M_POS_E));
        }
    }
    add_to_sensor(M_POS_N, dx);
    add_to_sensor(M_POS_E, dy);
}

double oline(vector_t *x0, vector_t *x1, vector_t *x) {
    double alpha ;
    alpha = atan2(x1->x-x0->x, x1->y-x0->y);
    return(-cos(alpha)*(x->x-x0->x) + sin(alpha)*(x->y-x0->y));
}

/*----------------------------------------------------------------------*
  New LBL navigation using Dana Yoerger's ABE algorithm installed on CCB97
  cruise aboard R/V Cape Hatteras 15-May-1997.
 *----------------------------------------------------------------------*/
void update_lbl_pos(void) {
    if (read_sensor(U_LBL_BEACON_NUMBER) < 2) {
	    add_to_sensor(M_POS_N, read_sensor(M_DR_DN));
	    add_to_sensor(M_POS_E, read_sensor(M_DR_DE));
	    return;
    }

    ABE_LBL();
}