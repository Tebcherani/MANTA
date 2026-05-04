/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: getopt.c,v 2.21 1998/01/20 00:31:51 bamoran Exp $
 *----------------------------------------------------------------------*/

#ifdef _OSK
#include <stdio.h>
#include <string.h>

/*
 * The getopt function available for UNIX is useful to process the
 * options of a command line.  We only need to provide it for OS-9
 * systems, however, because the Mac has no command line.  It is not
 * ever referenced in Mac version.
 */

char *optarg;
int optopt, optind = 1, opterr = 1;

int getopt( int argc, char *const argv[], const char *optstring )
{
    static int offset = 0;
    int arglen;
    char *opt, tmp;

    if ( optind < argc )
    {
                                /* make sure it is an option */
        if ( *argv[optind] != '-' || !*(argv[optind] + 1) )
            return EOF;
                                /* check for special case */
        if ( strcmp( argv[optind], "--" ) == 0 )
        {
            optind++;
            return EOF;
        }

        optopt = argv[optind][1 + offset];
        opt = strchr( optstring, optopt );

                                /* handle illegal option */
        if ( optopt == ':' || !opt )
        {
            if ( opterr )
                fprintf( stderr,  "illegal option -- %c\n", optopt );

                                /* does argv[optind] end? */
            if ( argv[optind][2 + offset] )
                                /* no, continue on next call */
                offset++;
            else
            {			/* yes, increment to next argument */
                offset = 0;
                optind++;
            }
            return '?';
        }

                                /* no optarg */
        if ( *(opt + 1) != ':' )
        {
                                /* does argv[optind] end? */
            if ( argv[optind][2 + offset] )
                                /* no, continue on next call */
                offset++;
            else
            {			/* yes, increment to next argument */
                offset = 0;
                optind++;
            }
            optarg = NULL;
            return optopt;
        }
        else			/* requires an optarg */
        {
                                /* set opt to point to the rest of */
                                /* argv[optind] following the flag */
            opt = argv[optind] + 2 + offset;

                                /* deal with the = that OS-9 likes */
            if ( *opt == '=' )
                opt++;

                                /* argv[optind] doesn't end */
            if ( *opt )
            {
                optarg = opt;	/* optarg is rest of argv[optind] */
                optind++;        /* increment for next call */
            }
                                /* else optind did end, is there */
                                /* another command line arg? */
            else if ( argc <= ++optind )
            {
                if ( opterr )
                    fprintf( stderr, "Option requires an argument -- "
                            "%c\n", optopt );
                return '?';
            }
            else
                optarg = argv[optind++];
            offset = 0;
            return optopt;
        }
    }
    return EOF;
}
#endif
