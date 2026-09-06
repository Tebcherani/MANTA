#include <time.h>

#include "time_utils.h"

int hirestime(double *hires) {
    struct timespec ts;

    if (hires == NULL) return -1;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return -1;

    *hires = (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;

    return 0;
}

int millisec(void) {
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return -1;

    return (int)(ts.tv_sec * 1000L + ts.tv_nsec / 1000000L);
}