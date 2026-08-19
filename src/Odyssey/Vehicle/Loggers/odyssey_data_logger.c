#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "load_fault.h"
#include "odyssey_data_logger.h"
#include "odyssey_data_struct.h"
#include "odyssey_defs.h"
#include "odyssey_sim.h"
#include "paths.h"
#include "sim_log.h"
#include "time_utils.h"
#include "vehicle.h"

static void write_fault_header(FILE *, const odyssey_config_t *);
static void write_mission_header(FILE **, const char *);

static void write_fault_header( FILE *datafile, const odyssey_config_t *config )
{
    FILE       *ffd;
    char        fd_path[256];
    char        line[256];
    int         n_lines = 0;
    const char *fname   = get_fault_file_name();

    fprintf( datafile,
        "# ===== FAULT_FILE_BEGIN ================================\xa" );

    if ( fname == NULL || fname[0] == '\0' ) {
        fprintf( datafile, "# (no fault file loaded)\xa" );
    } else {
        fprintf( datafile, "# fault file: %s\xa", fname );
        strncpy( fd_path, get_fault_file_path(), sizeof(fd_path) - 1 );
        fd_path[sizeof(fd_path) - 1] = '\0';
        if ( fd_path[0] == '\0' )
            ody_join_path( fd_path, sizeof(fd_path), config->fault_dir, fname );

        if ( (ffd = fopen(fd_path, "r")) == NULL ) {
            fprintf( datafile,
                "# WARNING: could not open %s for embedding\xa", fd_path );
        } else {
            while ( fgets(line, sizeof(line), ffd) ) {
                line[ strcspn(line, "\r\n") ] = '\0';
                fprintf( datafile, "# fd> %s\xa", line );
                n_lines++;
            }
            fclose( ffd );
            fprintf( datafile, "# (%d lines)\xa", n_lines );
        }
    }

    fprintf( datafile,
        "# ===== FAULT_FILE_END ==================================\xa" );
}

static void write_mission_header(FILE **datafile, const char *mission_path) {
    FILE *fmi;
    char line[MAX_LINE];
    size_t path_size;
    int n_lines = 0;

    fprintf(datafile, "# ===== MISSION_FILE_BEGIN ==============================\xa");

    if (mission_path == NULL || mission_path[0] == '\0') fprintf(datafile, "# (no mission file loaded)\xa");
    else {
            fmi = fopen(mission_path, "r");
            if (fmi == NULL) fprintf(datafile, "# WARNING: could not open %s for embedding\xa", mission_path);
            else {
                while (fgets(line, sizeof(line), fmi) != NULL) {
                    line[strcspn(line, "\r\n")] = '\0';
                    fprintf(datafile, "# mi> %s\xa", line);
                    n_lines++;
                }

                fclose(fmi);
                fprintf(datafile, "# (%d lines)\xa", n_lines);
            }
    }

    fprintf(datafile, "# ===== MISSION_FILE_END ================================\xa");
}

init_odyssey_data_logger(FILE **datafile, const char *vehicle_id, const char *data_dir, const char *mission_path, const char *fault_dir) {
    int i = 0;
    int j, beh_id, offset, run_number;
    int path_length;

    char *data_file_name;
    char time_string[30];
    char fname_string[32];

    time_t t;
    struct tm *tp;

    t = time(NULL);
    tp = gmtime(&t);

    if (tp == NULL) {
        write_to_sim_log("[odyssey_data_logger] Failed to get current time.\n");
        return 0;
    }

    if (strftime(fname_string, sizeof(fname_string), "%y%j", tp) == 0) {
        write_to_sim_log("[odyssey_data_logger] Failed to create data file timestamp.\n");
        return 0;
    }

    run_number = 0;
    data_file_name = NULL;

    do {
        ++run_number;

        free(data_file_name);

        path_length = snprintf(NULL, 0, "%s/%s_%c%s%02d.obd", data_dir, vehicle_id, PREFIX_CHAR, fname_string, run_number);        
        if (path_length < 0)
            return 0;

        data_file_name = malloc((size_t)path_length + 1);
        if (data_file_name == NULL)
            return 0;

        snprintf(data_file_name, (size_t)path_length + 1, "%s/%s_%c%s%02d.obd", data_dir, vehicle_id, PREFIX_CHAR, fname_string, run_number);    
    } while (access(data_file_name, F_OK) != -1);

    *datafile = fopen(data_file_name, "wb");
    if (*datafile == NULL) {
        write_to_sim_log("[odyssey_data_logger] Failed to open data file.\n");
        free(data_file_name);
        return 0;
    }

    strcpy(vvp->data_file_name, data_file_name);

    if (strftime(time_string, sizeof(time_string), "%Y/%m/%d %H:%M:%S GMT", tp) == 0) {
        fclose(*datafile);
        *datafile = NULL;
        free(data_file_name);
        write_to_sim_log("[odyssey_data_logger] Failed to create log timestamp.\n");
        return 0;
    }

    fprintf(*datafile, "%s     # OS-9 file creation time\xa", time_string);
    fprintf(*datafile, "%s     # Structure creation time\xa", vvp->creation_date);

    write_fault_header(*datafile, fault_dir);
    write_mission_header(*datafile, mission_path);

    for (i = 0; i <= vvp->last_s; i++) {
        fprintf(*datafile, "%d %-29s", i, sensor_name(i));

        if (sensor_time(i) >= 0L) fprintf(*datafile, " %.8g\xa", read_sensor(i));
        else fputc('\xa', *datafile);
    }

    for (i = 0; i < MAX_COMMAND; i++) {
        beh_id = vvp->c[i].behavior;

        if (beh_id != -1) {
            offset = vvp->last_s + 1 + (B_ARG_SIZE * (i - 1));
            fprintf(*datafile, "%d %-30s \t%-d\n", offset, vvp->c[i].b.name, i);

            for (j = 0; j < B_ARG_SIZE; j++) {
                if (vvp->c[i].b.arg[j].name[0] != '\0' && strlen(vvp->c[i].b.arg[j].name)) {
                    fprintf(*datafile, "%d %s_%-30s \t%-g\xa", offset + 1 + j, vvp->c[i].b.name, vvp->c[i].b.arg[j].name, vvp->c[i].b.arg[j].x);
                }
            }
        }
    }

    if (fflush(*datafile) != 0) {
        fclose(*datafile);
        *datafile = NULL;
        free(data_file_name);
        write_to_sim_log("[odyssey_data_logger] Failed to flush data file.\n");
        return 0;
    }

    free(data_file_name);
    
    write_to_sim_log("[odyssey_data_logger] Ody data logger initialized.\n");

    return 1;
}

/*------------------------------------------------------------------*
 | log_data
 *------------------------------------------------------------------*/ 

int log_data(FILE *datafile)
{
    short int flag=10001, i;
    static int first = 1;
    static float last_elapsed_time=0.0;   /* for debugging data logging */
    float temp_float;
    int my_start_time;

#ifdef NO_DATA_LOGGING
    return 0;
#endif

    my_start_time = millisec();
				/* write timestamp on first call, special
				   9999 format to parse this line in the
				   data file */
    if ( first )
    {
	fprintf( datafile, "9999 %.2f (%s)\xa", vvp->mission_start_time,
		 vvp->mission_date );
		 fflush( datafile );    /* ensure 9999 marker reaches disk */
    }
				/* first write the flag (use 10001) */
    fwrite( (void *) &flag, 2, 1, datafile);

    fwrite( (void *) &last_elapsed_time, 4, 1, datafile);  

/* need to modify to write only those values that have changed since
   the last read (use >= to write every value the first time through)
   Doesn't seem to work, use first */

    for ( i = 0; i <= vvp->last_s; i++ )
    {
	if ( sensor_time(i) >= sensor_time(M_PRESENT_TIME) )
	{
	    fwrite( (void *) &i, 2, 1, datafile );

	    temp_float = (float) read_sensor(i); 
	    fwrite((void *) &temp_float, 4, 1, datafile); 
	}
    }
    if ( first )

	first = 0;

    last_elapsed_time = (float) (millisec() - my_start_time);

    return 0;
}