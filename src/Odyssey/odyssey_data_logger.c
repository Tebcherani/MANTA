#include <stdio.h>
#include <time.h>
#include <unistd.h>

#include "odyssey_data_logger.h"
#include "odyssey_sim.h"
#include "sim_log.h"
#include "vehicle.h"

static void write_fault_header( FILE *datafile, const odyssey_config_t *config );
static void write_mission_header(FILE *datafile, const odyssey_config_t *config, const char *mission_file);

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

static void write_mission_header(FILE *datafile, const odyssey_config_t *config, const char *mission_file)
{
    FILE *fmi;
    char mi_path[PATH_SIZE];
    char line[MAX_LINE];
    int n_lines = 0;
    int result;

    fprintf(datafile,
        "# ===== MISSION_FILE_BEGIN ==============================\xa");

    if (mission_file == NULL || mission_file[0] == '\0') {
        fprintf(datafile, "# (no mission file loaded)\xa");
    } else {
        result = snprintf(mi_path, sizeof(mi_path), "%s/%s",
                          config->mission_dir, mission_file);

        if (result < 0 || (size_t)result >= sizeof(mi_path)) {
            fprintf(datafile, "# WARNING: mission path is invalid or too long\xa");
        } else {
            fmi = fopen(mi_path, "r");

            if (fmi == NULL) {
                fprintf(datafile,
                        "# WARNING: could not open %s for embedding\xa",
                        mi_path);
            } else {
                while (fgets(line, sizeof(line), fmi) != NULL) {
                    line[strcspn(line, "\r\n")] = '\0';
                    fprintf(datafile, "# mi> %s\xa", line);
                    n_lines++;
                }

                fclose(fmi);
                fprintf(datafile, "# (%d lines)\xa", n_lines);
            }
        }
    }

    fprintf(datafile,
        "# ===== MISSION_FILE_END ================================\xa");
}

int init_odyssey_data_logger(char *data_file_name,
                             FILE **data_file,
                             const odyssey_config_t *config,
                             const char *mission_file)
{
    int i = 0;
    int j;
    int beh_id;
    int offset;
    char time_string[30];
    char fname_string[32];
    time_t t;
    struct tm *tp;
    int run_number;

    if (data_file == NULL ||
        data_file_name == NULL ||
        config == NULL ||
        mission_file == NULL) {
        write_to_sim_log("[odyssey_data_logger] Invalid data logger arguments.\n");
        return 0;
    }

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

    do {
        ++run_number;

        snprintf(data_file_name, PATH_SIZE, "%s/%c%s%02d.obd",
                config->data_dir, PREFIX_CHAR, fname_string, run_number);

    } while (access(data_file_name, F_OK) != -1);

    *data_file = fopen(data_file_name, "wb");

    if (*data_file == NULL) {
        write_to_sim_log("[odyssey_data_logger] Failed to open data file.\n");
        return 0;
    }

    strcpy(vvp->data_file_name, data_file_name);

    fprintf(*data_file,
            "MIT_SG_AUV_LAB_v%d %s\xa",
            2,
            VEHICLE_NAME);

    if (strftime(time_string,
                 sizeof(time_string),
                 "%Y/%m/%d %H:%M:%S GMT",
                 tp) == 0) {
        fclose(*data_file);
        *data_file = NULL;
        write_to_sim_log("[odyssey_data_logger] Failed to create log timestamp.\n");
        return 0;
    }

    fprintf(*data_file,
            "%s     # OS-9 file creation time\xa",
            time_string);

    fprintf(*data_file,
            "%s     # Structure creation time\xa",
            vvp->creation_date);

    write_fault_header(*data_file, config);
    write_mission_header(*data_file, config, mission_file);

    for (i = 0; i <= vvp->last_s; i++) {
        fprintf(*data_file, "%d %-29s", i, sensor_name(i));

        if (sensor_time(i) >= 0L)
            fprintf(*data_file, " %.8g\xa", read_sensor(i));
        else
            fputc('\xa', *data_file);
    }

    for (i = 0; i < MAX_COMMAND; i++) {
        beh_id = vvp->c[i].behavior;

        if (beh_id != -1) {
            offset = vvp->last_s + 1 + (B_ARG_SIZE * (i - 1));

            fprintf(*data_file,
                    "%d %-30s \t%-d\n",
                    offset,
                    vvp->c[i].b.name,
                    i);

            for (j = 0; j < B_ARG_SIZE; j++) {
                if (vvp->c[i].b.arg[j].name[0] != '\0' &&
                    strlen(vvp->c[i].b.arg[j].name)) {
                    fprintf(*data_file,
                            "%d %s_%-30s \t%-g\xa",
                            offset + 1 + j,
                            vvp->c[i].b.name,
                            vvp->c[i].b.arg[j].name,
                            vvp->c[i].b.arg[j].x);
                }
            }
        }
    }

    if (fflush(*data_file) != 0) {
        fclose(*data_file);
        *data_file = NULL;
        write_to_sim_log("[odyssey_data_logger] Failed to flush data file.\n");
        return 0;
    }

    write_to_sim_log("[odyssey_data_logger] Ody data logger initialized.\n");
    return 1;
}