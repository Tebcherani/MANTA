/*---------------------------------------------------------------------*
  Minimal ksim runtime stubs

  Provides the globals and random noise generator used by the shared
  simulator frontend, without pulling in the full hydrodynamic code.
 *---------------------------------------------------------------------*/

#include <stdlib.h>
#include <math.h>
#include <time.h>

int UseSim = 1;
int MonitorSim = 0;

double gasdev( void )
{
    static int inited = 0;
    static int has_spare = 0;
    static double spare;
    double u, v, s;

    if (!inited) {
        srand((unsigned) time(NULL));
        inited = 1;
    }

    if (has_spare) {
        has_spare = 0;
        return spare;
    }

    do {
        u = 2.0 * rand() / RAND_MAX - 1.0;
        v = 2.0 * rand() / RAND_MAX - 1.0;
        s = u*u + v*v;
    } while (s == 0.0 || s >= 1.0);

    s = sqrt(-2.0 * log(s) / s);
    spare = v * s;
    has_spare = 1;
    return u * s;
}
