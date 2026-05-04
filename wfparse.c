/*----------------------------------------------------------------------*
                       Copyright (C) 1994-1996
        Massachusetts Institute of Technology, Cambridge, MA.
         Sea Grant Autonomous Underwater Vehicles Laboratory
                         All rights reserved.
 *----------------------------------------------------------------------*/
static char *rcsid = "$Revision: 2.21 $\t$Date: 1998/01/20 00:31:51 $";

#include <getopt.h>
#include <stdio.h>
#include <time.h>

main( int argc, char **argv )
{
    FILE *fin;
    struct tm *gmt;
    time_t clock;
    char *vname=0, format[256], line[1024];
    double pres_t, prev_t=-1, lat, lon, lat2, lon2;
    int c, i, nconv, first=1, odd, timeflag=0;

    while ( (c = getopt( argc, argv, "tv:" )) != EOF )
    {
        switch( c ) {
	case 't':
	    timeflag = 1;
	    break;
	case 'v':
	    vname = optarg;
	    break;
	case '?':
	    fprintf( stderr, "usage: %s [-t][-v vehiclename] files\n",
		     argv[0] );
	    exit( 1 );
	    break;
	}
    }
    if ( !vname )		/* default vehicle name */
        vname = "AUV";
    sprintf( format, "300,%s,%%lf,%%lf,%%lf,%%*lf,%%lf,%%lf", vname );
    for ( i = optind; i < argc; i++ )
    {
				/* open argv[i] with error check */
        if ( (fin = fopen( argv[i], "r" )) != NULL )
	{
	    do {
				/* read next available line */
	        fgets( line, sizeof(line), fin );
		nconv = sscanf( line, format, 
				&pres_t, &lat, &lon, &lat2, &lon2 );
				/* if pattern matches */
		if ( nconv == 5 )
		{
				/* output NaNs when delta time greater
				 than one minute */
		    if ( pres_t - prev_t > 60.0 )
		        printf( "NaN      NaN       NaN      "
				"NaN       NaN\n" );

				/* print data for unique records */
		    if ( pres_t != prev_t )
		    {
				/* should time be converted or not? */
			if ( timeflag )
			{
			    clock = pres_t;
			    gmt = gmtime( &clock );
			    printf( "%09.6f ", gmt->tm_hour +
				    (gmt->tm_min +
				     gmt->tm_sec/60.0)/60.0 );
#if FRED
			    printf( "%02d:%02d:%02d ", gmt->tm_hour,
				    gmt->tm_min, gmt->tm_sec );
#endif
			}
			else	/* print raw cpu time */
			    printf( "%.1f ", pres_t );

			printf( "%9.6f %9.6f %9.6f %9.6f\n",
				lat, lon, lat2, lon2 );
		    }
		    prev_t = pres_t;
		}
	    } while ( !feof( fin ) );
	    fclose( fin );
	}
	else
	    perror( argv[i] );
    }
    exit( 0 );
}
