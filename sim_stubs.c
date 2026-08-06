/*----------------------------------------------------------------------*
 * sim_stubs.c
 *
 * Defines hardware-subsystem globals declared with plain 'extern' in
 * ody_var.h whose implementing files were excluded from the simulator
 * build.  Variables declared with the EXTERN macro are already defined
 * by vehicle.c (which sets EXTERN to nothing) and must NOT appear here.
 *
 * All hardware flags are set to 0 (not selected / no error).
 *----------------------------------------------------------------------*/

#include <stdio.h>
#include "diagnostic.h"
#include "vehicle.h"
#include "time.h"

/*--- KVH DGI inclinometer (kvh.c deleted) ---*/
int UseKVH_DGI    = 0, MonitorKVH_DGI = 0;

/*----------------------------------------------------------------------*
 * heading_reference: simplified version for simulator use.
 *
 * The full KVH EKF filter (sensor_kvh.c) depends on a matrix library
 * not available in this build.  The filter is disabled in practice
 * (do_filtering=0), so we implement only the direct-copy path:
 * simulated inclinometer -> roll/pitch, simulated compass -> yaw.
 *----------------------------------------------------------------------*/
void heading_reference( void )
{
    double flux_yaw;

    flux_yaw = 3.14159265358979 +
               modPi( sensor(M_COMPASS_YAW)
                    + sensor(UO_MAGNETIC_VARIATION)
                    - 3.14159265358979 );

    update( M_ROLL,  sensor(M_INCLINO_ROLL)  );
    update( M_PITCH, sensor(M_INCLINO_PITCH) );
    update( M_YAW,   flux_yaw                );
}

/*----------------------------------------------------------------------*
 * remote_delay: stub behavior registered in vehicle.c but source file
 * not included in the simulator build.
 *----------------------------------------------------------------------*/
int remote_delay( struct behavior *b, int priority, int level )
{
    (void)b; (void)priority; (void)level;
    return 0;
}

/*======================================================================*
 * Serial-bus stubs (would live in serial.c / ody_serio.c)
 *
 * These functions are called only from lbl.c's hardware init path,
 * which is guarded by "if (!UseLBL) return;" at entry.  They are
 * never reached in simulator mode but must exist for the linker.
 *======================================================================*/

 /* timing stubs */
int hirestime( double *hires )
{
    struct timespec ts;
    clock_gettime( CLOCK_MONOTONIC, &ts );
    *hires = (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
    return 0;
}

int millisec( void )
{
    struct timespec ts;
    clock_gettime( CLOCK_MONOTONIC, &ts );
    return (long)(ts.tv_sec * 1000L + ts.tv_nsec / 1000000L);
}