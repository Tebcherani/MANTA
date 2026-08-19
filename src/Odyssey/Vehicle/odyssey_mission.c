#include <errno.h>
#include <string.h>

#include "odyssey_data_struct.h"
#include "odyssey_defs.h"
#include "odyssey_mission.h"
#include "sim_log.h"
#include "time_utils.h"
#include "unit_utils.h"
#include "vehicle.h"

static int match_behavior_arg(const char *name, int b_index);
static int read_state(FILE *fptr);

static int match_behavior_arg( const char *name, int b_index )
{
    int i;
    char *loc;
	char thisName[sizeof(vvp->s[0].name)];
    char vvvName[sizeof(vvp->s[0].name)];

    strcpy( thisName, name );
    if ( (loc = strchr(thisName, '(')) != NULL )
	*loc = '\0';

    for ( i = 0; i < B_ARG_SIZE; i++ )
    {
	strcpy( vvvName, vvp->b[b_index].arg[i].name );
	if ( (loc = strchr(vvvName, '(')) != NULL )
	    *loc = '\0';

        if ( strcmp(vvvName, thisName) == 0 )
            return i;
    }
    return -1;
}

static int read_state(FILE *fptr) {
    char name[MAX_STATE_NAME];
    char linebuf[MAX_LINE];
    char *expect;
    char *actual;
    int priority = 0;
    int index;
    int b_index = -1;
    double value;

    if (fptr == NULL) {
        write_to_sim_log("[odyssey_mission] Mission state file pointer is NULL.\n");
        return -1;
    }

    while (fgets(linebuf, sizeof(linebuf), fptr) != NULL) {
        if (sscanf(linebuf, " state: %s", name) == 1) {
            if (vvp->nstates >= MAX_STATES) {
                write_to_sim_log("[odyssey_mission] Maximum number of mission states exceeded.\n");
                return -1;
            }

            snprintf(vvp->sta[vvp->nstates].name, MAX_STATE_NAME, "%s", name);
            
            vvp->nstates++;

            if (vvp->nstates == 1)
                vvp->cstate = 0;
            else
                vvp->cstate++;

            priority = 0;
            b_index = -1;
        }

        else if (sscanf(linebuf, "  sensor: %s %lg", name, &value) == 2) {
            index = match_sensor(name);

            if (index < 0) {
                write_to_sim_log("[odyssey_mission] Unknown sensor in mission file: %s\n", name);
                return -1;
            }

            expect = strchr(vvp->s[index].name, '(');
            actual = strchr(name, '(');

            if (expect && actual &&
                !convert_units(expect, actual, value, &value)) {
                write_to_sim_log("[odyssey_mission] Failed to convert units for sensor: %s\n", name);
                return -1;
            }

            update_sensor(index, value);
        }

        else if (sscanf(linebuf, " behavior: %s", name) == 1) {
            priority++;

            if (priority >= MAX_COMMAND) {
                write_to_sim_log("[odyssey_mission] Maximum behavior priority exceeded.\n");
                return -1;
            }

            b_index = match_behavior(name);

            if (b_index < 0) {
                write_to_sim_log("[odyssey_mission] Unknown behavior in mission file: %s\n", name);
                return -1;
            }

            vvp->sta[vvp->cstate].c[priority].behavior = b_index;

            memcpy(
                &vvp->sta[vvp->cstate].c[priority].b,
                &bTemplate[b_index],
                sizeof(struct behavior)
            );

            if (assign_behavior_fcn_ptr(
                    name,
                    &vvp->sta[vvp->cstate].c[priority].b) != 0) {
                write_to_sim_log("[odyssey_mission] Failed to assign behavior function: %s\n", name);
                return -1;
            }
        }

        else if (sscanf(linebuf, "  b_arg: %s %lg", name, &value) == 2) {
            if (b_index < 0) {
                write_to_sim_log("[odyssey_mission] Behavior argument found before a behavior definition.\n");
                return -1;
            }

            index = match_behavior_arg(name, b_index);

            if (index < 0) {
                write_to_sim_log("[odyssey_mission] Unknown behavior argument: %s\n", name);
                return -1;
            }

            expect = strchr(vvp->b[b_index].arg[index].name, '(');
            actual = strchr(name, '(');

            if (expect && actual &&
                !convert_units(expect, actual, value, &value)) {
                write_to_sim_log("[odyssey_mission] Failed to convert behavior argument units: %s\n", name);
                return -1;
            }

            vvp->b[b_index].arg[index].x = value;
            vvp->sta[vvp->cstate].c[priority].b.arg[index].x = value;

            strncpy(
                vvp->sta[vvp->cstate].c[priority].b.arg[index].name,
                vvp->b[b_index].arg[index].name,
                sizeof(vvp->sta[vvp->cstate].c[priority].b.arg[index].name) - 1
            );

            vvp->sta[vvp->cstate].c[priority].b.arg[index].name[sizeof(vvp->sta[vvp->cstate].c[priority].b.arg[index].name) - 1] = '\0';
        }
    }

    update_sensor(M_DR_NORTH, read_sensor(M_POS_N));
    update_sensor(M_DR_EAST, read_sensor(M_POS_E));

    return 0;
}

int match_behavior( const char *name )
{
    int i = 0;

    for ( i = 0; i < MAX_BEHAVIOR; i++ )
        if ( strcmp(vvp->b[i].name, name) == 0 ) 
            return i;
    
    return -1;
}

int stack_busy(struct command *cmd) {
    return (cmd->speed_mode != -1) && (cmd->heading_mode != -1) &&
	(cmd->depth_mode != -1);
}

int init_mission_time(time_t calendar_start_time) {
    struct tm *time_info;

    time_info = gmtime(&calendar_start_time);
    if (time_info == NULL) return 0;
    if (strftime(vvp->mission_date, sizeof(vvp->mission_date), "%b %d, %Y %H:%M:%S", time_info) == 0) return 0;
    if (hirestime(&vvp->mission_start_time) != 0) return 0;

    write_to_sim_log("[odyssey_mission] Mission time initialized.\n");
    return 1;
}

int init_odyssey_mission(const char *mission_path) {
    FILE *file;

    file = fopen(mission_path, "r");
    if (file == NULL) {
        write_to_sim_log("[load_mission] Failed to open mission file '%s': %s.\n", mission_path, strerror(errno));
        return 0;
    }

    if (read_state(file) != 0) {
        write_to_sim_log("[load_mission] Failed to read mission state.\n");
        fclose(file);
        return 0;
    }

    fclose(file);

    write_to_sim_log("[load_mission] Mission loaded.\n");
    return 1;
}

int update_odyssey_mission(void) {
    if (stack_busy(&vvp->c[0])) return 0;
    vvp->cstate++;
    return vvp->cstate >= vvp->nstates;
}