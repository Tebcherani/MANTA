/*----------------------------------------------------------------------*
   Odyssey AUV Simulator -- Fault Injection Engine
   load_fault.c

   Parses a fault definition file (*.fd) and evaluates
   fault rules every control cycle via apply_faults().

   See fault_injection_spec.md for full language description.

   J. G. Bellingham, 2024
 *----------------------------------------------------------------------*/

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "diagnostic.h"
#include "odyssey_config.h"
#include "odyssey_data.h"
#include "paths.h"
#include "vehicle.h"

/* Use a local buffer size to avoid conflict with ody_defs.h MAX_LINE */
#define FAULT_MAX_LINE   256

#define MAX_FAULT_RULES   64
#define MAX_COND_TERMS     8
#define MAX_ACT_TERMS      8

typedef enum { OP_SENSOR, OP_LITERAL } OperandKind;

typedef struct {
    OperandKind kind;
    int         index;
    double      literal;
} Operand;

typedef enum { REL_GT, REL_LT, REL_GE, REL_LE, REL_EQ, REL_NE } RelOp;

typedef struct {
    Operand lhs;
    RelOp   op;
    Operand rhs;
} Comparison;

typedef enum { CONN_AND, CONN_OR } Connector;

typedef struct {
    int        is_prob;
    double     prob_rate;
    int        n_terms;
    Comparison term[MAX_COND_TERMS];
    Connector  conn[MAX_COND_TERMS - 1];
} Condition;

typedef struct {
    int     target_index;
    Operand value;
} Assignment;

typedef struct {
    Condition  cond;
    int        n_actions;
    Assignment action[MAX_ACT_TERMS];
} FaultRule;

/* ------------------------------------------------------------------ */
/* Module-level state                                                   */
/* ------------------------------------------------------------------ */
static FaultRule rules[MAX_FAULT_RULES];
static int       n_rules = 0;
static char      linebuf[FAULT_MAX_LINE];
static char      fullpath[FAULT_MAX_LINE];
static char      loaded_fname[MAX_FILE_NAME] = "";
static char      loaded_path[FAULT_MAX_LINE] = "";

/* forward declarations */
static int    parse_condition( const char *s, Condition *cond );
static int    parse_actions( const char *s, int max, Assignment *acts, int *n );
static int    parse_operand( const char *tok, Operand *op );
static int    parse_relop( const char *tok, RelOp *op );
static double eval_operand( const Operand *op );
static int    eval_condition( const Condition *cond );
static int    eval_comparison( const Comparison *c );


/* ================================================================== */
/* PUBLIC API                                                          */
/* ================================================================== */

/*--------------------------------------------------------------------
  GET_FAULT_FILE_NAME
  Returns the bare filename of the last successfully opened fault file,
  or "" if none was loaded.  Used by log_data.c to embed the fault
  scenario in the ODB header.
 --------------------------------------------------------------------*/
const char *get_fault_file_name( void )
{
    return loaded_fname;
}

const char *get_fault_file_path( void )
{
    return loaded_path;
}

/*--------------------------------------------------------------------
  LOAD_FAULTS
  Call once after load_mission().  Returns 0 on success, -1 on error.
  A missing fault file is not an error -- fault injection is optional.
 --------------------------------------------------------------------*/
int load_faults( const char *fname )
{
    FILE *fp;
    int   lineno = 0;
    char *arrow, *cond_str, *act_str;

    n_rules = 0;
    loaded_fname[0] = '\0';
    loaded_path[0] = '\0';
    vvp->fault_file_name[0] = '\0';

    if ( fname == NULL || fname[0] == '\0' ) {
        sprintf( msg_buf, "load_faults: no fault file specified\n" );
        write_message( TERM | SLOG );
        return 0;
    }

    strncpy( loaded_fname, fname, sizeof(loaded_fname) - 1 );
    strncpy( vvp->fault_file_name, fname,
             sizeof(vvp->fault_file_name) - 1 );

    if ( ody_path_is_explicit(fname) || ody_file_exists(fname) )
	strcpy( fullpath, fname );
    else
	ody_join_path( fullpath, sizeof(fullpath), SFAULT_DIR, fname );
    strncpy( loaded_path, fullpath, sizeof(loaded_path) - 1 );

    if ( (fp = fopen(fullpath, "r")) == NULL ) {
        sprintf( msg_buf, "load_faults: no fault file %s\n", fullpath );
        write_message( TERM | SLOG );
        return 0;   /* optional -- not an error */
    }
    sprintf( msg_buf, "fault file: %s ...\n", fullpath );
    write_message( TERM | SLOG );

    while ( fgets(linebuf, sizeof(linebuf), fp) ) {
        lineno++;
        linebuf[ strcspn(linebuf, "\r\n") ] = '\0';
        if ( linebuf[0] == '#' || linebuf[0] == '\0' ) continue;

        if ( strncmp(linebuf, "fault:", 6) != 0 ) {
            sprintf( msg_buf,
                "load_faults line %d: expected 'fault:'\n", lineno );
            write_message( TERM | SLOG );
            fclose(fp); return -1;
        }
        if ( n_rules >= MAX_FAULT_RULES ) {
            sprintf( msg_buf, "load_faults: too many rules (max %d)\n",
                     MAX_FAULT_RULES );
            write_message( TERM | SLOG );
            fclose(fp); return -1;
        }

        cond_str = linebuf + 6;
        while ( *cond_str == ' ' || *cond_str == '\t' ) cond_str++;

        arrow = strstr( cond_str, "->" );
        if ( !arrow ) {
            sprintf( msg_buf,
                "load_faults line %d: missing '->'\n", lineno );
            write_message( TERM | SLOG );
            fclose(fp); return -1;
        }

        /* trim trailing whitespace from condition string */
        {
            char *p = arrow - 1;
            while ( p >= cond_str && (*p == ' ' || *p == '\t') ) *p-- = '\0';
        }
        *arrow = '\0';
        act_str = arrow + 2;
        while ( *act_str == ' ' || *act_str == '\t' ) act_str++;

        {
            FaultRule *r = &rules[n_rules];
            memset( r, 0, sizeof(FaultRule) );

            if ( parse_condition(cond_str, &r->cond) != 0 ) {
                sprintf( msg_buf,
                    "load_faults line %d: bad condition: %s\n",
                    lineno, cond_str );
                write_message( TERM | SLOG );
                fclose(fp); return -1;
            }
            if ( parse_actions(act_str, MAX_ACT_TERMS,
                               r->action, &r->n_actions) != 0 ) {
                sprintf( msg_buf,
                    "load_faults line %d: bad action: %s\n",
                    lineno, act_str );
                write_message( TERM | SLOG );
                fclose(fp); return -1;
            }

            sprintf( msg_buf, "  fault rule %d loaded\n", n_rules + 1 );
            write_message( TERM | SLOG );
            n_rules++;
        }
    }

    sprintf( msg_buf, "load_faults: %d rule(s) from %s\n",
             n_rules, fname );
    write_message( TERM | SLOG );
    fclose(fp);
    return 0;
}

/*--------------------------------------------------------------------
  APPLY_FAULTS
  Called every control cycle after sensor_processing(), before
  layered_control().  Evaluates all conditions and fires assignments.
  Updates M_FAULT_ACTIVE so the logger records when faults fire.
 --------------------------------------------------------------------*/
void apply_faults( void )
{
    int i, j, fired = 0;

    for ( i = 0; i < n_rules; i++ ) {
        FaultRule *r = &rules[i];
        if ( eval_condition(&r->cond) ) {
            fired++;
            for ( j = 0; j < r->n_actions; j++ ) {
                double val = eval_operand( &r->action[j].value );
                update( r->action[j].target_index, val );
            }
        }
    }
    update( M_FAULT_ACTIVE, (double)fired );
}


/* ================================================================== */
/* PARSING HELPERS                                                     */
/* ================================================================== */

static int parse_operand( const char *tok, Operand *op )
{
    char *end;
    double v = strtod( tok, &end );
    if ( end != tok && *end == '\0' ) {
        op->kind    = OP_LITERAL;
        op->literal = v;
        return 0;
    }
    {
        int idx = match_sensor( tok );
        if ( idx < 0 ) {
            sprintf( msg_buf, "parse_operand: unknown sensor '%s'\n", tok );
            write_message( TERM | SLOG );
            return -1;
        }
        op->kind  = OP_SENSOR;
        op->index = idx;
    }
    return 0;
}

static int parse_relop( const char *tok, RelOp *op )
{
    if ( strcmp(tok, ">=") == 0 ) { *op = REL_GE; return 0; }
    if ( strcmp(tok, "<=") == 0 ) { *op = REL_LE; return 0; }
    if ( strcmp(tok, "==") == 0 ) { *op = REL_EQ; return 0; }
    if ( strcmp(tok, "!=") == 0 ) { *op = REL_NE; return 0; }
    if ( strcmp(tok, ">")  == 0 ) { *op = REL_GT; return 0; }
    if ( strcmp(tok, "<")  == 0 ) { *op = REL_LT; return 0; }
    return -1;
}

static int parse_condition( const char *s, Condition *cond )
{
    char buf[FAULT_MAX_LINE];
    strncpy( buf, s, FAULT_MAX_LINE - 1 );
    buf[FAULT_MAX_LINE - 1] = '\0';

    /* prob() special case */
    if ( strncmp(buf, "prob(", 5) == 0 ) {
        char *close = strchr( buf + 5, ')' );
        if ( !close ) return -1;
        *close = '\0';
        if ( sscanf(buf + 5, "%lf", &cond->prob_rate) != 1 ) return -1;
        cond->is_prob = 1;
        cond->n_terms = 0;
        return 0;
    }

    cond->is_prob = 0;
    cond->n_terms = 0;

    {
        char *tokens[FAULT_MAX_LINE / 2];
        int   n_tok = 0;
        int   i = 0;
        char *p = strtok( buf, " \t" );
        while ( p && n_tok < (int)(FAULT_MAX_LINE / 2) ) {
            tokens[n_tok++] = p;
            p = strtok( NULL, " \t" );
        }

        while ( i < n_tok ) {
            Comparison *cmp;
            if ( i + 2 >= n_tok ) return -1;
            if ( cond->n_terms >= MAX_COND_TERMS ) return -1;

            cmp = &cond->term[ cond->n_terms ];
            if ( parse_operand(tokens[i],     &cmp->lhs) != 0 ) return -1;
            if ( parse_relop  (tokens[i + 1], &cmp->op ) != 0 ) return -1;
            if ( parse_operand(tokens[i + 2], &cmp->rhs) != 0 ) return -1;
            i += 3;
            cond->n_terms++;

            if ( i < n_tok ) {
                int ci = cond->n_terms - 1;
                if ( strcmp(tokens[i], "&&") == 0 )
                    cond->conn[ci] = CONN_AND;
                else if ( strcmp(tokens[i], "||") == 0 )
                    cond->conn[ci] = CONN_OR;
                else
                    return -1;
                i++;
            }
        }
    }
    return ( cond->n_terms == 0 ) ? -1 : 0;
}

static int parse_actions( const char *s, int max_act,
                          Assignment *acts, int *out_n )
{
    char buf[FAULT_MAX_LINE];
    strncpy( buf, s, FAULT_MAX_LINE - 1 );
    buf[FAULT_MAX_LINE - 1] = '\0';

    *out_n = 0;
    {
        char *tok = strtok( buf, ";" );
        while ( tok ) {
            char lhs_name[64], rhs_name[64];
            Assignment *a;
            int idx;

            while ( *tok == ' ' || *tok == '\t' ) tok++;
            if ( *out_n >= max_act ) return -1;

            a = &acts[ *out_n ];
            if ( sscanf(tok, "%63s = %63s", lhs_name, rhs_name) != 2 )
                return -1;

            idx = match_sensor( lhs_name );
            if ( idx < 0 ) {
                sprintf( msg_buf,
                    "parse_actions: unknown target '%s'\n", lhs_name );
                write_message( TERM | SLOG );
                return -1;
            }
            a->target_index = idx;
            if ( parse_operand(rhs_name, &a->value) != 0 ) return -1;

            (*out_n)++;
            tok = strtok( NULL, ";" );
        }
    }
    return ( *out_n == 0 ) ? -1 : 0;
}


/* ================================================================== */
/* EVALUATION HELPERS                                                  */
/* ================================================================== */

static double eval_operand( const Operand *op )
{
    if ( op->kind == OP_SENSOR )
        return sensor( op->index );
    return op->literal;
}

static int eval_comparison( const Comparison *c )
{
    double l = eval_operand( &c->lhs );
    double r = eval_operand( &c->rhs );
    switch ( c->op ) {
        case REL_GT: return l >  r;
        case REL_LT: return l <  r;
        case REL_GE: return l >= r;
        case REL_LE: return l <= r;
        case REL_EQ: return l == r;
        case REL_NE: return l != r;
    }
    return 0;
}

static int eval_condition( const Condition *cond )
{
    int group, result, i;

    if ( cond->is_prob )
        return ( drand48() < cond->prob_rate * sensor(M_DT) );

    if ( cond->n_terms == 0 ) return 0;

    /* && before ||, left-to-right */
    group  = eval_comparison( &cond->term[0] );
    result = 0;

    for ( i = 0; i < cond->n_terms - 1; i++ ) {
        if ( cond->conn[i] == CONN_AND ) {
            group = group && eval_comparison( &cond->term[i + 1] );
        } else {
            result = result || group;
            group  = eval_comparison( &cond->term[i + 1] );
        }
    }
    return result || group;
}
