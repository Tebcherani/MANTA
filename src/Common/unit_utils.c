#include "common_defs.h"
#include "unit_utils.h"

double convert_deg_to_rad(double degree) {
    return degree * PI /180.0;
}

double convert_rad_to_deg(double rad) {
    return rad * 180.0 / PI;
}

int convert_units( const char *expected, const char *given, const double inval, double *outval ) {
    int error = 0;
                                /* if they differ, then check for cases */
    if ( strcmp(expected, given) != 0 )
    {
        if ( !strncmp( expected, "(rad", 4 ) &&
             !strncmp( given, "(deg", 4 ) )
            *outval = convert_deg_to_rad(inval);
        else if ( !strncmp( expected, "(deg", 4 ) &&
                  !strncmp( given, "(rad", 4 ) )
            *outval = convert_rad_to_deg(inval);
	else if ( !strncmp(expected, "(m", 2) &&
		  !strncmp(given, "(km", 3) )
	    *outval = 1000*inval;
	else if ( !strncmp(expected, "(km", 3) &&
		  !strncmp(given, "(m", 2) )
	    *outval = 0.001*inval;
        else
            error = -1;		/* differ, but unknown conversion */
    }
    else                        /* otherwise just transfer directly */
        *outval = inval;

    return error;
}