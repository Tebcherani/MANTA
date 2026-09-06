#ifndef ODYSSEY_NAV_H
#define ODYSSEY_NAV_H

#define LBL_NUM_RANGES 5
#define NXP 4

typedef enum {NAV_OFF, NAV_GOOD, NAV_STALE, NAV_TIMED_OUT} fix_status_t;
typedef enum {CW, CCW, ON_BASELINE} lbl_side_t;
typedef enum {RANGE_OLD, RANGE_TOO_LONG, RANGE_FAILED_MEDIAN, RANGE_HIGH_ERROR, RANGE_GOOD} range_status_t;

typedef struct{
    double x, y, z;
} vector_t;

typedef struct {
    double range, bearing, range_rate;
    vector_t position;
    double time, last_time;
    int new, num_fixes;
    fix_status_t fix_status;
} fix_t;

typedef struct {
    int i0, i1;
} lbl_baseline_pair_t;

typedef struct {
    vector_t xp[NXP] ;		/* xpndr positions */
    vector_t xhat ;		/* estimated vehicle position*/
    double tat[NXP] ;		/* turn-around time (sec) */  
    range_status_t good[NXP];	/* 1 if judged good, 0 if bad, 2 if high error */
    double range[NXP][LBL_NUM_RANGES] ;	/* array of ranges */
    double max_range[NXP] ;	/* max range from each xpndr */
    double time[NXP][LBL_NUM_RANGES] ; /* time each range was rcv'd */
    vector_t position[LBL_NUM_RANGES] ;	/* position from each set of ranges */
    int status[LBL_NUM_RANGES] ; /* status of each set of ranges */
    double good_range[NXP] ;	/* last range declared good for each xpndr*/
    double good_time[NXP] ;	/* times for the good ranges */       
    double depth ;		/* vehicle depth */    
    double sound_speed ;	/* sound speed at bottom */
    double range_tol ;		/* range tolerance used for median filter */
    double fix_tol ;		/* fix tolerance used to judge ls soln */
    double timeout ;		/* timeout (secs) */
    lbl_baseline_pair_t baseline; /* baseline pair used in 2 xpndr nav */
    lbl_side_t side ;		/* baseline side used in 2 xpndr nav */
    double min_angle ;		/* min crossing angle for a 2 xpndr fix */
    double error ;		/* error from LS position fix */
    double alpha ;		/* convergence parameter  (0.5)*/
    int max_iter ;		/* max iterations for ls solution */ 
    int num_steps_pinv ;	/* # steps between psuedoinverse calc */ 
    int index ;			/* current index for arrays */     
    int new_fix ;		/* 1 if a new fix has been rcvd */
    int lost ;			/* 1 if we're lost */           
} lbl_t;

void update_dead_reckoned_pos(void);
void update_lbl_pos(void);

int update_odyssey_nav(void);

#endif