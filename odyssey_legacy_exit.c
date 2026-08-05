#include <stdio.h>
#include <stdlib.h>

#include "command_line.h"
#include "diagnostic.h"
#include "ody_func.h"
#include "sim_behavior_logger.h"
#include "vehicle.h"

extern FILE *datafile;
extern command_line_options_t options;

void handle_modem(void);
int  stack_busy(struct command *);

void exit_gracefully( int exit_status )
{
    char tmp[256], *tmp1, *tmp2;
    long size = 0;

    handle_modem();      /* send any remaining modem messages */
    
    if ( datafile )
    {
	fflush( datafile );
	size = ftell( datafile );
	fclose( datafile );
    }

    if (options.behavior_dump_file[0]) {
        dump_behavior_log_json(options.behavior_dump_file);
    }

    sprintf( msg_buf, "Mission finished -- data file: %s\n", 
	     vvp->data_file_name); 
    write_message( TERM | SLOG );

#ifdef ODY_MODEM
    load_modem( msg_buf );   /* jleonard 6/28/96 */
    write_modem(); 
#endif
				/* 01/10/94 adding an "auto_parse" option */
    tmp1 = getenv("ODY_AUTO_PARSE");
    tmp2 = getenv("ODY_PARSE_OPTION");

    if ( tmp1 )
    {
        if ( tmp2 )
	    sprintf( tmp, "parse -o %s %s", tmp2, vvp->data_file_name );
        else sprintf( tmp, "parse %s", vvp->data_file_name );

        system( tmp );
    }

#ifdef ODY_MODEM
    if ( modemlog != NULL )
	fclose( modemlog );
#endif
				/* generate entry in tabular file */
    write_tab();

    exit( 0 );
}