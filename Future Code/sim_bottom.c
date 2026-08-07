/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: sim_bottom.c,v 2.21 1998/01/20 00:31:51 bamoran Exp $
 *----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*
 |                                                                       |
 | Created by abennett 11/17/94                                          |
 | Last Modified by abennett 12/19/95                                    |
 |      4.21.95 Reworked to carry internally consistant array and        | 
 |              different input data file types.                         |
 |      9.11.95 Restructured code, made more modular & systematic.       |
 |              Deleted OA1 and OA2 returns as vehicle only has altimeter|
 |     10.11.95 Added sonar ray model                                    |
 |     10.30.95 Added Faceted bottom model                               |
 |     12.18.95 Added tracking of true bottom location and facet (as     |
 |              opposed to the facet of interest to the sonar).          |
 |                                                                       |
 | Generates altimeter returns for the vehicle simulation.  Bottom       |
 |may be a mathematical function, a file or an old vehicle track.        |
 |                                                                       |
 |For the purposes of sonar calculations, a modified right hand coord-   |
 |inate system is used:                                                  |
 |                                                                         |
 | Global coordinate system:     X is due north.                         |
 |   (origin is defined as       Y is due east.                          |
 |    vehicle start point)       Z is positive downwards.                |
 |                                                                       |
 | Local coordinate system:      Xlocal is out the bow.                  |
 |   (origin is defined as       Ylocal is out the starboard side.       |
 |    center of vehicle          Zlocal is out the bottom.               |
 |                                                                         |
 | The three OA sonars are mounted looking out the bottom, forward at a  |
 | 45 degree angle, and straight out the bow.  They can be thought of as |
 | three unit vectors:                                                   |
 |                   Spherical                         Cartesian           |
 |  Altimeter: Theta = 0; Phi = 0; R = 1              [0,0,1]            |
 |               OA1: Theta = 0; Phi = 45; R = 1        [.7071,0,.7071]    |
 |               OA2: Theta = 0; Phi = 90; R = 1        [1,0,0]            |
 *-----------------------------------------------------------------------*/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "common_defs.h"
#include "diagnostic.h"
#include "vehicle.h"
#include "ody_defs.h"
#include "ody_types.h"
#include "ody_var.h"
#include "ody_func.h"
#include "odyssey_data_struct.h"

#define MAXROWS      300000
#define MAXCOLS      25
#define FIRST_RETURN 1
#define FAST         1
#define DETAILED     0
/*------------------------ Output Modes ----------------------------*/
#define NECESSARY  0
#define FULL       1

#ifndef DEG_TO_RAD
 #define DEG_TO_RAD  0.017453
 #define RAD_TO_DEG  57.2958
#endif
#define MAX_STRING         100
/*
 * Sonar specific parameters
 */
#define MAX_RANGE (double) 40.0        /* Maximum Sonar Range */
#define MIN_RANGE (double) 0.0        /* Minimum Useful Range */
#define CONE_ANGLE (double) 10.0 /* half of total cone angle of the sonar */
#define Alt_mount (double) [0, 0, 1]      /* Straight down */
#define OA1_mount (double) [0.5, 0, 0.87] /* Forward at a 30 degree angle */
#define OA2_mount (double) [1, 0, 0]      /* Straight ahead */
/*
 * FileBottom Variables
 */
/* #define DIMENX 1024
#define DIMENY 1024 */

#define DIMENX 128
#define DIMENY 128

/*
 * Structures
 */
struct bottom_point {
    double X;          /* Global X coordinate */
    double Y;          /* Global Y coordinate */
    double Z;          /* Global Z coordinate */
    double reflect;    /* Reflection Coefficient */
    double absorb;     /* Absorbtion Coefficient */
} ;
struct ray_struct {    /* sonar ray in both cartesian and angle/magnitude */
    double X;          /* Location of origin of ray.  Whether that is bottom */
    double Y;          /*  or xducer depends on the situation. */
    double Z;
    double nx;         /* unit normal components. */
    double ny;
    double nz;
    double mag;
    double phi_t;      /* angle relative to the plane of the sonar x-ducer */
    double phi_b;      /* angle relative to the plane of the bottom facet */
    double intensity;  /* for signal strength calculations */
    int P1;            /* Facet which returned the ray */
    int P2;            /* (GlobalBottom[P1], etc.) */
    int P3;
} ;
struct point_struct {
    double X;
    double Y;
    double Z;
} ;
struct state_vect {
    double X;
    double Y;
    double Z;
    double roll;
    double pitch;
    double yaw;
} ;
struct order_struct {
    int adjs;            /* Number of nodes adjacent to this point. */
    int *node;           /* Pointer to integer array of node numbbers. */
    } ;                  /* Array varies from approx 1 to 11 nodes. */

struct triangle_struct {
    int P1;              /* Node numbers of vertices.  I.e. the vertices */
    int P2;              /* are GlobalBottom[P1], GlobalBottom[P2], etc. */
    int P3;
    double Xc;           /* Centroid of triangle defined by P1, P2, P3 */
    double Yc;
    double Zc;
    double A;            /* A in planar form = Nx in normal form */
    double B;            /* B in planar form = Ny in normal form */
    double C;            /* C in planar form = Nz in normal form */
    double D;
} ;
struct node_struct {
    double Xo;           /* Offset from Centroid in local X-direction */
    double Yo;           /* Offset from Centroid in local Y-direction */
    double Zo;           /* Offset from Centroid in local Z-direction */
    double compliance;   /* Compliance of node. */
    double dist;         /* Distance from node to bottom. */
} ;
struct trench_struct {
  double X1;             /* One end of the trench */
  double Y1;
  double Z1;
  double X2;             /* The other end of the trench */
  double Y2;
  double Z2;
  double width;          /* Width of the trench */
  double slope_frac;     /* Fraction of width that is sloped */
} ;

static double time, last_time = 0.0; /* Used for occasional printing */
static double delay_time = 30.0;
static int counter = 10;    /* Used for occasional printing */
static int sonar_mode = FAST;     /* Set to FAST or DETAILED */
static int log_mode = NECESSARY; /* How much info we log to a file */
static int bytecount, nrows, ncols, num_triangles = 0;
static float bottom_array[DIMENY] [DIMENX], bottom[DIMENY] [DIMENX];
static struct bottom_point *GlobalBottom;  /* Bottom point coordinates */
static struct triangle_struct *Triangle;   /* List of all valid triangles */
static int *adj;            /* adjacency table for triangulation and the */
                            /* ordering of nodes */
static struct state_vect RealPos;
static struct state_vect PerceivedPos;
static struct order_struct *Order;  /* points to an array of pointers & */
                                    /* sizes of adjacencies */
/*
 * General Variables
 */
#define TRENCH_BOTTOM 1
#define CHIRP_BOTTOM 2
#define SINE_BOTTOM 3
#define MATH_FILEBOTTOM 4
#define FILE_BOTTOM 7
#define HISTORICAL_BOTTOM 5
#define SPECULAR 1
#define DIFFUSE 2
#define COMBINED 3
#define ELLIPSE 2
#define TRAP 1
/* debug states */
#define LOADDATA     1  /* 1 = LoadData */
#define RAYCOMP      2  /* 2 = ray computation routines */
#define LOCATOR      3  /* 3 = alternate locator */
#define EXTRAFILE    4  /* 4 = extra file is generated */
#define NOBEAM       5  /* 5 = disable beam pattern */
#define MATHBOTTOM   6  /* 6 = MathBottom */
#define INIT         7  /* 7 = Initialization routines */
#define TRENCH_TRACK 8  /* 8 = Monitor vehicle/trench interaction */
#define ALL          9  /* 9 = all */
static int debug = 0;
static int trench_type = TRAP;
static double *PV_theta, *PQ_theta;
static int reflect_model = COMBINED;        /* Specular or lambertian */
static int i, j, k, result;
/*
 * Sonar Variables
 */
#ifndef TRUE
  #define TRUE 1
  #define FALSE 0
#endif
#define KYD_TO_KM  1.094      /* converts db/kiloyard to db/km */
#define MAKE_KM    0.78       /* subtract to shift kyd based references to km */
                              /* km 0.78 = 20log10(1m/1yd) */
#define FREQ       500.0      /* sonar frequency, in kHz */
#define DIAMETER   0.047      /* diameter of sonar transducer, in meters */
#define LAMBDA     0.005      /* wavelength (1500m/s @500kHz), in meters */
#define SOURCE_INTENSITY 185.0 /* In Decibles. From jleonard */
#define DETECT_THRESH 85.0    /* In Decibles */ /* jleonard 12/18chngfrom100*/

static double Alt_Return;     /* Range reported by Altimeter */
static double OA1_Return;     /* Range reported by OA1 sonar */
static double OA2_Return;     /* Range reported by OA2 sonar */
static double Alt_Depth = 0.0;
static double *Temp_Alt_Depth;
static double Noise_Ampl;
static double dummy;
static int Near_Node = -1, start = 1;
static struct point_struct Alt_Mount;  /* Altimeter loc in vehicle coords */
static struct point_struct GAlt_Mount; /* Altimeter loc in global coords */
static struct ray_struct Alt;        /* Unit alt-relative normal in */
                                     /*  vehicle coords) */
static struct ray_struct GAlt;       /* Altimeter normal in global coords */
static struct point_struct AltTip;   /* alt-relative vector in vehicle coords */
static struct point_struct GAltTip;  /* alt-relative vector in global coords */
static struct ray_struct *Facet_Ray; /* Normal ray to every facet */
static double resolution = 0.05;     /* sonar resolution in meters */
static struct ray_struct minray;     /* Minimum range from filebottom */
static struct ray_struct diffray;    /* Duffuse reflection ray */
/*
 * Seabed Variables
 */
static int Bottom_Type;
static double wavelength = 100.0, base_depth = 25.0, amplitude = 8.0; /* SineBottom */
static double base_wavelength = 100.0;  /* ChirpBottom */
static double scale_factor = 0.1;       /* ChirpBottom */
static double ray_d;                    /* Ray Length */
static struct bottom_point Intersect_Point; /* Ray-seabed point of intersect */
static int kmax;                        /* maximum number of bottom points */
static int LoadFlag = TRUE;             /* Has datafile been loaded yet? */
static int TriangFlag = FALSE;          /* Has data file been triangulated? */
static double RealWaterColumn = 0.0;    /* True bottom at vehicle location. */
/* 
 * TrenchBottom
 */
static int num_trench = 1;  /* number of trenches in math_bottom */
static double *intercept, *slope;
static double *dd, *dd1, *dd2, *trench_length, Zc = 7.0, def_slope;
static struct trench_struct *Trench;
static struct point_struct *VP, *VQ, *PQ, *VPxVQ;

/*
 * Function Prototypes
 */
static int math_bottom( void );
static int file_bottom( void );
static int historical_bottom( void );
static int make_point_global( struct point_struct *global_point, struct point_struct *local_point);
static int make_point_local( struct point_struct *local_point, struct point_struct *global_point);
void make_ray_global(struct ray_struct *global_ray, struct ray_struct *local_ray);
int load_data( void );
static double dmax(double, double);
static double dmin(double, double);
static void rmin(struct ray_struct *R1, struct ray_struct *R2);
static void nodemin(struct node_struct *N1, struct node_struct *N2);
static int get_plane( struct triangle_struct *Tri, int k );
static double get_range( struct triangle_struct *Tri, struct ray_struct *xducer, struct ray_struct *any_ray );
static int in_triangle( struct triangle_struct *Tri, struct ray_struct *any_ray );
static int ray_check( void );
static double beam_pattern( struct ray_struct *any_ray );
static double bessj1( double );
static double trans_loss( struct ray_struct *any_ray );
static double bottom_reflect( struct ray_struct *any_ray, int );
static double ray_mag( struct ray_struct *any_ray );
static double triangle_search(struct state_vect *RealPos);
static double bottom_collision(struct state_vect *RealPos);
void delaunay_triangulate(double *, int *, int *, int *);

/*****************************************************************************
 **************************** SIM_BOTTOM FUNCTION  ***************************
 *****************************************************************************/

void sim_bottom(void)
{
    static int once = 1;
    static double outlier_threshold;
    int contact = 1;         /* 1 = stretch, -1 = shrink, 0 = good value */

    outlier_threshold = 1.0 - read_sensor(REAL_SONAR_FRAC_DROPOUT);
/*
 *  NOTE: REAL_X is NORTH and REAL_Y is EAST!!
 */
    RealPos.X = read_sensor(REAL_X);
    RealPos.Y = read_sensor(REAL_Y);
    RealPos.Z = read_sensor(REAL_Z);
    RealPos.roll = read_sensor(REAL_PHI);
    RealPos.pitch = read_sensor(REAL_THETA);
    RealPos.yaw = read_sensor(REAL_PSI);
    PerceivedPos.Z = read_sensor(M_DEPTH);
    PerceivedPos.roll = read_sensor(M_ROLL);
    PerceivedPos.pitch = read_sensor(M_PITCH);
    PerceivedPos.yaw = read_sensor(M_YAW);
    Noise_Ampl = read_sensor(U_BOTTOM_NOISE);
    Bottom_Type = read_sensor(U_BOTTOM_TYPE);

    if ( once ) {
        once = 0;
    }
/*
 * Altimeter location in local and global coords.  Needed to determine ray
 * intersections and intensities.  Note X is axial, Y is lateral and Z is down.
 */
    Alt_Mount.X = 0.11;          /* Location of Altimeter in Odyssey IIb in */
    Alt_Mount.Y = 0.0;           /* vehicle-relative coords, in (m) relative */
    Alt_Mount.Z = 0.3;           /* to a CG/origin which is 0.11m forward of */
                                 /* the mid-body (from Franz Hover). */

    result = make_point_global( &GAlt_Mount, &Alt_Mount );
/*
 * SEQUENCE OF EVENTS FOR MATH_BOTTOM:
 *
 *   1) Generate the ray.  
 *   2) Stretch it a bit.
 *   3) See if the tip of the ray is within "Resolution" of the bottom (plus
 *       or minus, doesn't matter).  The details of this test depend on
 *       which bottom type is in use.
 *   4a) If too short, make it 10 * Resolution larger.
 *   4b) If too long, make it 1 * Resolution shorter.
 *   5) Add noise and dropouts, as appropriate.
 *   6) Return results.
 */
    Alt.X = Alt_Mount.X;         /* xducer location in local coords. */
    Alt.Y = Alt_Mount.Y;
    Alt.Z = Alt_Mount.Z;
    Alt.nx = 0.0;                /* Initial Altimeter unit vector */
    Alt.ny = 0.0;                /* in local coords. */
    Alt.nz = 1.0;
    Alt.mag = ray_mag(&Alt);
    Alt.intensity = SOURCE_INTENSITY;
    make_ray_global( &GAlt, &Alt );

    AltTip.X = Alt_Mount.X;
    AltTip.Y = Alt_Mount.Y;
    AltTip.Z = Alt_Mount.Z;
        Alt_Depth = 0.0;

    while ( contact != 0 )
    {
        if(Bottom_Type != FILE_BOTTOM) {
            if ( contact == 1)         /* We haven't reached the bottom yet. */
                AltTip.Z = AltTip.Z + (100.0 * resolution);
            else if ( contact == -1 )  /* We overshot and need to back up. */
                AltTip.Z = AltTip.Z - resolution;
            else {                     /* Something went wrong. */
                exit_gracefully( -1 );
            }
        }
        result = make_point_global( &GAltTip, &AltTip );
        switch (Bottom_Type) {
        case TRENCH_BOTTOM:
        case CHIRP_BOTTOM:
        case SINE_BOTTOM:
        case MATH_FILEBOTTOM:
          contact = math_bottom();
          break;
            
        case FILE_BOTTOM:
          contact = file_bottom();
          break;
            
        case HISTORICAL_BOTTOM:
          contact = historical_bottom();
          break;
            
        default:
          contact = 0;
          Alt_Return = -1.0;
          OA1_Return = -1.0;
          OA2_Return = -1.0;
          break;
            
        } /* END OF BOTTOM TYPE SWITCH */
        
    } /* END OF WHILE */
/*
 * Check for range limits.
 */
    if (Alt_Return >= MAX_RANGE) Alt_Return = MAX_RANGE;
    if (OA1_Return >= MAX_RANGE) OA1_Return = MAX_RANGE;
    if (OA2_Return >= MAX_RANGE) OA2_Return = MAX_RANGE;

    /********************************** NOISE ******************************** 
     * The noise is split into two items - dropout and range noise.  The
     * fraction of dropouts is set by the value of outlier_threshold, which 
     * is in turn set by the value of REAL_SONAR_FRAC_DROPOUT, while the range
     * error is set by Noise_Ampl, which is obtained from the mission script
     * by the variable U_BOTTOM_NOISE.
     *************************************************************************/
    if ( Noise_Ampl != 0.0 ) {
        if(drand48() > outlier_threshold)
            Alt_Return = MAX_RANGE + (Noise_Ampl * MAX_RANGE) *
                2.0 * (drand48()-0.5);
        else
            Alt_Return = Alt_Return + (Noise_Ampl * Alt_Return) *
                2.0 * (drand48()-0.5);
        
        if(drand48() > outlier_threshold)
            OA1_Return = MAX_RANGE + (Noise_Ampl * MAX_RANGE) *
                2.0 * (drand48()-0.5);
        else
            OA1_Return = OA1_Return + (Noise_Ampl * OA1_Return) *
                2.0 * (drand48()-0.5);
        
        if(drand48() > outlier_threshold)
            OA2_Return = MAX_RANGE + (Noise_Ampl * MAX_RANGE) *
                2.0 * (drand48()-0.5);
        else
            OA2_Return = OA2_Return + (Noise_Ampl * OA2_Return)  *
                2.0 * (drand48()-0.5);
    }

    update_sensor(M_ALTITUDE, Alt_Return);

} /* END OF SIM_BOTTOM */

/**************************************************************************
**************** A mathematically generated bottom function ***************
*** THE LINES BELOW ARE A SIMPLE SAMPLE BOTTOM FILE FOR THE MATH_TRENCH ***
***************************************************************************
% First lines are mean bottom depth, mean bottom slope.  Next line is number
% of trenches in file.
%
% trench is P1.x, P1.y, P1.z, P2.x, P2.y, P2.z, trench width, fraction of 
% trench width that is sloped, wall type (1 = trapezoidal, 2 = elliptical).
% Note that the z coordinate is relative to the trench lip and not the total
% depth!
%
depth 7.25
slope 0.0
trenches 2
100.0 800.0 3.0 500.0 500.0 3.0 15.0 0.25 1
100.0 600.0 3.0 500.0 300.0 3.0 15.0 0.25 1
***************************************************************************/

static int math_bottom(void)
{
    double test, Z1, Z2, sqtmp;
    static double Zoffset;
    static double *Yoffset;
    static int OnceFlag = 1;
    static struct ray_struct TempVect;
    static int coll_flag = 1;
        char *MathFile, sdummy[80], sdummy2[80];
        FILE *trenchfile;

    switch (Bottom_Type)
    {
    case TRENCH_BOTTOM:               /* A Mathematically Defined Trench */
          if ( OnceFlag ) {
            OnceFlag = 0;
                MathFile = getenv("MATHFILE");
                if(MathFile == NULL) {
                  Trench = (struct trench_struct *) calloc( 1, 
                                                                                  sizeof(struct trench_struct) );
                  dd = (double *) calloc( 1, sizeof(double));
                  dd1 = (double *) calloc( 1, sizeof(double));
                  dd2 = (double *) calloc( 1, sizeof(double));
                  Temp_Alt_Depth = (double *) calloc( 1, sizeof(double));
/*
 * VERY IMPORTANT!  NOTE THAT X = north, Y = east!! 
 */
                  Trench[0].X1 = 100.0;     /* replaces A1.X,Y,Z of older versions */
                  Trench[0].Y1 = 800.0;
                  Trench[0].Z1 = 3.0;       /* replaces T_depth of older versions */
                  Trench[0].X2 = 500.0;     /* replaces A2.X,Y,Z of older versions */
                  Trench[0].Y2 = 500.0;
                  Trench[0].Z2 = 3.0;       /* replaces T_depth of older versions */
                  Trench[0].width = 15.0;   /* replaces T_width of older versions */
                  Trench[0].slope_frac = 0.25; /* replaces T_wall of older versions */
                  slope = (double *) calloc( 1, sizeof(double) );
                  intercept = (double *) calloc( 1, sizeof(double) );
                  VP = (struct point_struct *) calloc( 1, sizeof(struct point_struct));
                  VQ = (struct point_struct *) calloc( 1, sizeof(struct point_struct));
                  PQ = (struct point_struct *) calloc( 1, sizeof(struct point_struct));
                  VPxVQ = (struct point_struct *) calloc( 1,
                                                                                                  sizeof(struct point_struct));
                  PV_theta = (double *) calloc( 1, sizeof(double));
                  PQ_theta = (double *) calloc( 1, sizeof(double));
                  trench_length = (double *) calloc( 1, sizeof(double));
                  Yoffset = (double *) calloc( 1, sizeof(double));
                  /* 
                   * A line for the trench to follow.  Remember that X is north and 
                   * Y is east, so that is why they looked "swapped" in the equation.
                   */
                  slope[0] = (Trench[0].X2 - Trench[0].X1)/(Trench[0].Y2 - 
                                                                                                        Trench[0].Y1);
                  intercept[0] = Trench[0].X1 - slope[0]*Trench[0].Y1;
                  dummy = Trench[0].X2 - slope[0]*Trench[0].Y2;
                  if( debug == MATHBOTTOM) {
                        fprintf(stderr, "SIM_BOTTOM (trench init): A1(x,y) = (%.2lf, "
                                        "%.2lf), A2(x,y) = (%.2lf, %.2lf), slope = %.2lf,"
                                        " intercept = (0.0, %.2lf).\n",Trench[0].X1, Trench[0].Y1,
                                        Trench[0].X2, Trench[0].Y2, slope[0], intercept[0]);
                  }
                  if( fabs(intercept[0] - dummy) >= 0.01) {
                        exit(0);
                  }
                } else {
                  trenchfile = fopen( MathFile, "r" );
                  if( !trenchfile ) {
                        perror( MathFile );
                        exit( -1 );
                  }
                  i = 0;
                  while( fgets(sdummy, 81, trenchfile) != NULL ) {
                        if( sdummy[0] != '%' ) {
                          if( sdummy[0] == 'd')
                                sscanf( sdummy, "%s%lf", sdummy2, &Zc );
                          else if( sdummy[0] == 's')
                                sscanf( sdummy, "%s %lf", sdummy2, &def_slope );
                          else if( sdummy[0] == 't') {
                                sscanf( sdummy, "%s %d", sdummy2, &num_trench);
                                j = num_trench + 1;
                                Trench = (struct trench_struct *) calloc( j,
                                                                                   sizeof(struct trench_struct) );
                                slope = (double *) calloc( j, sizeof(double));
                                intercept = (double *) calloc(j, sizeof(double));
                                VP = (struct point_struct *) calloc( j, 
                                                                                   sizeof(struct point_struct));
                                VQ = (struct point_struct *) calloc( j, 
                                                                                   sizeof(struct point_struct));
                                PQ = (struct point_struct *) calloc( j, 
                                                                                   sizeof(struct point_struct));
                                VPxVQ = (struct point_struct *) calloc( j, 
                                                                                   sizeof(struct point_struct));
                                PV_theta = (double *) calloc( j, sizeof(double));
                                PQ_theta = (double *) calloc( j, sizeof(double));
                                trench_length = (double *) calloc( j, sizeof(double));
                                dd = (double *) calloc( j, sizeof(double));
                                dd1 = (double *) calloc( j, sizeof(double));
                                dd2 = (double *) calloc( j, sizeof(double));
                                Temp_Alt_Depth = (double *) calloc( j, sizeof(double));
                                Yoffset = (double *) calloc( j, sizeof(double));
                          } else {
                                sscanf( sdummy, "%lf%lf%lf%lf%lf%lf%lf%lf%d",
                                                &Trench[i].X1, &Trench[i].Y1, &Trench[i].Z1,
                                                &Trench[i].X2, &Trench[i].Y2, &Trench[i].Z2,
                                                &Trench[i].width, &Trench[i].slope_frac, 
                                                &trench_type );
                                i++;
                          }
                        } /* END OF '%' CHECK */
                  } /* END OF FILE INPUT */
                  for(i=0;i<num_trench;i++) {
                        slope[i] = (Trench[i].X2 - Trench[i].X1)/(Trench[i].Y2 - 
                                                                                                          Trench[i].Y1);
                        intercept[i] = Trench[i].X1 - slope[i]*Trench[i].Y1;
                        dummy = Trench[i].X2 - slope[i]*Trench[i].Y2;
                        if( fabs(intercept[i] - dummy) >= 0.01) {
                          exit(0);
                        }
                  }
                }
          } /* END OF ONCEFLAG */
          Yoffset[0] = slope[0] * GAltTip.X + intercept[0];
          Zoffset = Zc;
/* 
 * Distance to trench calculation - given two points (P,Q) on a line and 
 * third point (V) not on the line, the distance from the point V to the
 * line defined by (PQ) is:
 * 
 *  dd = ||VP x VQ||/||PQ||
 *
 */
/* NEW SECTION - points P and Q are now trench end coordinates as defined
 * by the input file or by the source code.
 */
          time = read_sensor(M_PRESENT_TIME);
          for( i = 0;i<num_trench;i++) {
                VP[i].X = Trench[i].X1 - RealPos.X;
                VP[i].Y = Trench[i].Y1 - RealPos.Y;
/*      VP[i].Z = Trench[i].Z1 - RealPos.Z;  <- Only 2-D for now. */
                VQ[i].X = Trench[i].X2 - RealPos.X;
                VQ[i].Y = Trench[i].Y2 - RealPos.Y;
/*      VQ[i].Z = Trench[i].Z2 - RealPos.Z; */
                PQ[i].X = Trench[i].X2 - Trench[i].X1;
                PQ[i].Y = Trench[i].Y2 - Trench[i].Y1;
/*      PQ[i].Z = Trench[i].Z2 - Trench[i].Z1; */
/*            VPxVQ[i].X = (VP[i].Y * VQ[i].Z) - (VP[i].Z * VQ[i].Y); */
/*      VPxVQ[i].Y = (VP[i].X * VQ[i].Z) - (VP[i].Z * VQ[i].X); */
        VPxVQ[i].Z = (VP[i].X * VQ[i].Y) - (VP[i].Y * VQ[i].X);
        VPxVQ[i].X = 0.0;                           /* FOR 2-D */
        VPxVQ[i].Y = 0.0;
                PQ[i].Z = 0.0;
                VP[i].Z = 0.0;
                VQ[i].Z = 0.0;
                dd[i] = sqrt((VPxVQ[i].X*VPxVQ[i].X) + (VPxVQ[i].Y*VPxVQ[i].Y) + 
                                         (VPxVQ[i].Z*VPxVQ[i].Z))/sqrt((PQ[i].X*PQ[i].X) + 
                                                                    (PQ[i].Y*PQ[i].Y) + (PQ[i].Z*PQ[i].Z));
                dd1[i] = sqrt(VP[i].X*VP[i].X + VP[i].Y*VP[i].Y + VP[i].Z*VP[i].Z);
                dd2[i] = sqrt(VQ[i].X*VQ[i].X + VQ[i].Y*VQ[i].Y + VQ[i].Z*VQ[i].Z);
                trench_length[i] = sqrt((PQ[i].X*PQ[i].X) + (PQ[i].Y*PQ[i].Y) +
                                                                (PQ[i].Z*PQ[i].Z));
                PV_theta[i] = atan2( (Trench[i].X2-RealPos.X),
                                                         (Trench[i].Y2-RealPos.Y) );
                PQ_theta[i] = atan2( (Trench[i].X1-RealPos.X),
                                                         (Trench[i].Y1-RealPos.Y) );
                if( debug == TRENCH_TRACK ) {
                  if( (time - last_time) >= delay_time ) {
                        last_time = time;
                        for(i=0;i<num_trench;i++) {
                          fprintf(stderr, "SIM_BOTTOM(math_trench): Time = %.2lf, "
                                          "RealPos(X,Y) = (%.1lf, %.1lf),\n"
                                          "                         i = %d, dd = %.2lf, "
                                          "dd1+dd2 = %.2lf,\n"
                                          "                         trench_length[i] = %.2lf\n",
                                          time, RealPos.X, RealPos.Y, i, dd[i], (dd1[i]+dd2[i]), 
                                          trench_length[i]);
                        }
                  }
                }
                if(trench_type == ELLIPSE ) {
/* 
 * An elliptical trench
 */
                  if(dd[i] >= 1.0 * Trench[i].width) {
                        Temp_Alt_Depth[i] = Zc;
                  } else {
                        sqtmp =  sqrt((Trench[i].width*Trench[i].width) - 
                                                  ((GAltTip.Y - Yoffset[i])*(GAltTip.Y - Yoffset[i])));
                        Z1 = Zoffset + (Trench[i].Z1/Trench[i].width) * sqtmp;
                        Z2 = Zoffset - (Trench[i].Z1/Trench[i].width) * sqtmp;
                        Temp_Alt_Depth[i] = dmax(Z1, Z2);
                  }
                }
                if(trench_type == TRAP ) {
/* 
 * A trapezoidal trench or a cliff.
 * Smaller Trench[i].slope_frac is a steeper wall.
 * Large Trench[i].width yields a mesa or cliff.
 */
/*
 * check #1 (seems to have a bug):
 if( (dd[i] >= Trench[i].width) || (fabs(PV_theta[i] - PQ_theta[i]) 
                                                          >= (PI/2)) ) {
 */
                  if( (dd[i] >= Trench[i].width) || ((dd1[i] + dd2[i]) >= 
                                                                                 (trench_length[i] + 10.0))) {
                        Temp_Alt_Depth[i] = Zc;
                  } else {
                        if ( (dd[i] <= Trench[i].width) && (dd[i] >= (1 - 
                                                                  Trench[i].slope_frac) * Trench[i].width) ) {
                          Temp_Alt_Depth[i] = -1.0 * ( Trench[i].Z1 / 
                                                             ( Trench[i].slope_frac * Trench[i].width) ) *
                                dd[i] + ( Zc + Trench[i].Z1 / Trench[i].slope_frac );
                        } else {
                          Temp_Alt_Depth[i] = Zc + Trench[i].Z1;
                        }
                  }
                } /* end of trap_trench */
                Alt_Depth = dmax( Alt_Depth, Temp_Alt_Depth[i]);
          } /* end of for num_trench loop */
          break;
            
        case CHIRP_BOTTOM:                      /* A chirp-function bottom */
        wavelength = base_wavelength / (GAltTip.X * scale_factor);
        Alt_Depth = base_depth + amplitude * sin( GAltTip.X / (wavelength * PI) );
        break;

    case SINE_BOTTOM:                      /* A simple Sine Wave Bottom */

    default:
        Alt_Depth = base_depth + amplitude * sin( GAltTip.X / (wavelength * PI) );
        break;
    }
    
    if ( fabs(Alt_Depth - GAltTip.Z) <= resolution ) {  /* Good range value */
        TempVect.X = AltTip.X - Alt_Mount.X;
        TempVect.Y = AltTip.Y - Alt_Mount.Y;
        TempVect.Z = AltTip.Z - Alt_Mount.Z;
/*
 * Check to see if we penetrated the bottom.
 */
        if ( RealPos.Z >= GAltTip.Z ) {
            Alt_Return = MAX_RANGE;
            if ( coll_flag ) {
                coll_flag = 0;
            }
        }
        else {
            if ( coll_flag == 0 ) {
                coll_flag = 1;
            }
            Alt_Return = ray_mag( &TempVect );
        }
        return(0);
    }
    else if ( (Alt_Depth - GAltTip.Z) >= 0.0 )
        return(1);                                   /* Too short */
    else
        return(-1);                                  /* Too long */
} /* END OF MATH BOTTOM */

/**************************************************************************
 ************************** A seafloor data file **************************
 **************************************************************************/

static int file_bottom( void )
{
    double coll_dist;
    int load_data( void );
/*
 * LOAD IN DATA FILE
 */
    if(LoadFlag == TRUE)
    {
        LoadFlag = FALSE;
        if(load_data() == -1)
            exit_gracefully(-1);
        Facet_Ray = (struct ray_struct *) calloc( (num_triangles+1), 
                                                 sizeof(struct ray_struct) );
        for(k = 1; k <= num_triangles; k++) {
            Facet_Ray[k].mag = -1.0;  /* Insures all unused rays are noted */
            Facet_Ray[k].intensity = -100.0;
            result = get_plane( &Triangle[k], k );
        }
    }
/*
 * Check to see if the vehicle is in the area defined by the data file.
 * If so, report the fact.  If not, return the true depth beneath the
 * vehicle.
 */
    if(TriangFlag == TRUE) {
/*        coll_dist =  bottom_collision( &RealPos ); */
        RealWaterColumn = triangle_search( &RealPos );
        update_sensor(REAL_ALT, (RealWaterColumn - RealPos.Z) );
        if( sonar_mode == FAST ) {
         Alt_Return = RealWaterColumn - RealPos.Z;
        }
        if( (debug != 0) && (counter == 100) ) {
            counter = 0;
        } else counter++;
    }
        if( sonar_mode == DETAILED) {
/*
 * The specular reflection routine - unlike the lambertian case, only rays 
 * that are perpendicular to the bottom facets will reflect back to the sensor.
 * While this is not realistic, it will do for a first cut (maybe an interval
 * spline later).
 *
 * The procedure is:
 *
 * 1) Choose a triangular facet (set up in load_data).
 * 2) Derive the plane parameters A, B, C, D defined by the three vertex points
 * of the facet.
 * 3) Find a normal to the plane which intersects the sensor.
 * 4) Check if the normal's point of intersection with the plane is inside of 
 *    the facet defined by the three points.
 * 5a) If not, go to the next facet.
 * 5b) If it is in the facet, get the ray length.
 * 6a) If the ray is longer than the sonar's limit range (defined in the unit's
 *    Manual), go to the next facet.
 * 6b) Otherwise, given the length and orientation of the ray, determine the 
 *     final intensity relative to the initial intensity.  Check at each step
 *     so we don't calculate any more than we have to.
 *     A. Check intensity leaving the beam pattern.  If below Detection 
 *        Threshold, stop.
 *     B. Check intensity after absorbtion in the water column (check the 
 *        round trip length).  If below Detection Threshold, stop.
 *     C. If bottom is not perfectly reflecting, check intensity after bottom
 *        reflection.  If below Detection Threshold, stop.
 *     D. Check intensity entering receiver beam pattern.
 * 7a) If final intensity falls below the Detection Threshold, go to the next 
 *    facet.
 * 7b) If intensity is above the Detection Threshold, return the range and 
 *    intensity to the sensor model (to be written).  Like the real TriTech,
 *    the sensor model can return either the first return (shortest ray) or
 *    each range and intensity for collection into "range bins," which are
 *    valid returns sorted by range groupings.
 */
          for (k = 1; k <= num_triangles; k++) {
                if (get_range( &Triangle[k], &GAlt, &Facet_Ray[k] ) 
                        <= MAX_RANGE ) {
                  Facet_Ray[k].intensity = SOURCE_INTENSITY;             
                  if (in_triangle( &Triangle[k], &Facet_Ray[k] ) ) {
/*
 * At this point, we have the locaton on the bottom where the normal to the 
 * facet intersects.  It is already assumed that this normal intersects the
 * transducer or we would not have reached this part of the code.
 *
 * The next thing to do is to determine what the ray magnitude and direction
 * is, relative to the transducer (i.e. the transducer normal).
 */
                        if (beam_pattern(&Facet_Ray[k]) >= DETECT_THRESH ) {
                          if (trans_loss(&Facet_Ray[k]) >= DETECT_THRESH ){
                                if (bottom_reflect(&Facet_Ray[k], reflect_model) >= 
                                        DETECT_THRESH ) {
                                  if (beam_pattern(&Facet_Ray[k]) >= 
                                          DETECT_THRESH ) {
                                        continue;
                                  } else Facet_Ray[k].mag = -1.0;
                                } else Facet_Ray[k].mag = -1.0;
                          } else Facet_Ray[k].mag = -1.0;
                        } else Facet_Ray[k].mag = -1.0;
                  }
                }
          }
/*
 * For first return altimeters, take the shortest ray whose intensity is above
 * the detection threshold value and return that.
 *
 * For range-bin systems, return a pointer to all of the valid rays.
 */
          if( read_sensor(M_PRESENT_TIME) >= 24.0)
                minray.mag = 0.0;
          
          minray.mag = 0.0;
          if( FIRST_RETURN ) {
                minray.mag = MAX_RANGE;
                for(k = 1; k <= num_triangles; k++) {
                  if( (Facet_Ray[k].mag != -1.0) && 
                          (Facet_Ray[k].intensity >= DETECT_THRESH) ) {
                    rmin( &Facet_Ray[k], &minray);
                  }
                }
                if( reflect_model == COMBINED ) {
                  if(minray.mag == MAX_RANGE) {
                        minray.mag = diffray.mag - (RealPos.Z + Alt_Mount.Z);
                        minray.X = diffray.X;
                        minray.Y = diffray.Y;
                        minray.Z = diffray.Z;
                        minray.P1 = diffray.P1;
                        minray.P2 = diffray.P2;
                        minray.P3 = diffray.P3;
                  }    
                }
                Alt_Return = minray.mag;
/* Run triangle walk on realpos.X & Y to get true bottom. */

/* jleonard 12/18/95 - logging more information for min return */
                if( log_mode == FULL ) {
                  update_sensor(SIM_SONAR_PHI_T, minray.phi_t);
                  update_sensor(SIM_SONAR_PHI_B, minray.phi_b);
                  update_sensor(SIM_SONAR_INTENSITY, minray.intensity);
                  update_sensor(SIM_TRIANGLE_X1, GlobalBottom[minray.P1].X);
                  update_sensor(SIM_TRIANGLE_Y1, GlobalBottom[minray.P1].Y);
                  update_sensor(SIM_TRIANGLE_Z1, GlobalBottom[minray.P1].Z);
                  update_sensor(SIM_TRIANGLE_X2, GlobalBottom[minray.P2].X);
                  update_sensor(SIM_TRIANGLE_Y2, GlobalBottom[minray.P2].Y);
                  update_sensor(SIM_TRIANGLE_Z2, GlobalBottom[minray.P2].Z);
                  update_sensor(SIM_TRIANGLE_X3, GlobalBottom[minray.P3].X);
                  update_sensor(SIM_TRIANGLE_Y3, GlobalBottom[minray.P3].Y);
                  update_sensor(SIM_TRIANGLE_Z3, GlobalBottom[minray.P3].Z);
                
                  update_sensor(SIM_CONTACT_X, minray.X);
                  update_sensor(SIM_CONTACT_Y, minray.Y);
                  update_sensor(SIM_CONTACT_Z, minray.Z); 
                }
                
          } else {
                Alt_Return = -1.0;
          }
        } /* end of if(mode == DETAILED) section */
    return(0);
} /* END OF FILE_BOTTOM */


/*****************************************************************************
 *************** Real data from past missions or other sources ***************
 *****************************************************************************/
 
static int historical_bottom( void )
{
        Alt_Return = 1.5;
        OA1_Return = 2.0;
        OA2_Return = 2.5;
        update_sensor(M_ALTITUDE, Alt_Return);

        return(0);
      }
/*****************************************************************************
 *************************** SUPPORTING FUNCTIONS ****************************
 *****************************************************************************
 *----------------------------------------------------------------------------*
 *   make_point_local  - converts a globally defined point to a locally       *
 * defined one, based on current vehicle pitch, roll and yaw.                 *
 *---------------------------------------------------------------------------*/
int make_point_local(struct point_struct *local_point, struct point_struct *global_point)
{

    local_point->X = global_point->X - RealPos.X;
    local_point->Y = global_point->Y - RealPos.Y;
    local_point->Z = global_point->Z - RealPos.Z;

    local_point->X = cos(RealPos.pitch)*cos(RealPos.yaw)*global_point->X + cos(RealPos.pitch)*sin(RealPos.yaw)*global_point->Y - sin(RealPos.pitch)*global_point->Z;

    local_point->Y = (sin(RealPos.roll)*sin(RealPos.pitch)*cos(RealPos.yaw) - cos(RealPos.roll)*sin(RealPos.yaw))*global_point->X + (sin(RealPos.roll)*sin(RealPos.pitch)*sin(RealPos.yaw) + cos(RealPos.roll)*cos(RealPos.yaw))*global_point->Y + sin(RealPos.roll)*cos(RealPos.pitch)*global_point->Z;

    local_point->Z = (cos(RealPos.roll)*sin(RealPos.pitch)*cos(RealPos.yaw) + sin(RealPos.roll)*sin(RealPos.yaw))*global_point->X + (cos(RealPos.roll)*sin(RealPos.pitch)*sin(RealPos.yaw) - sin(RealPos.roll)*cos(RealPos.yaw))*global_point->Y + cos(RealPos.roll)*cos(RealPos.yaw)*global_point->Z;

/* We assume a positive ocean space. */

  if ( local_point->Z <= 0.0 )
    return -1;
  else
    return 0;
}

/*----------------------------------------------------------------------------*
 *   make_point_global - converts a locally (vechicle-) defined point to a    *
 * globally defined one, based on current vehicle pitch, roll and yaw.        *
 *---------------------------------------------------------------------------*/
int make_point_global (struct point_struct *global_point, struct point_struct *local_point)
{

/* First transform to account for roll, pitch and yaw */

    global_point->X = cos(RealPos.pitch)*cos(RealPos.yaw)*local_point->X + (sin(RealPos.roll)*sin(RealPos.pitch)*cos(RealPos.yaw) - cos(RealPos.roll)*sin(RealPos.yaw))*local_point->Y + (cos(RealPos.roll)*sin(RealPos.pitch)*cos(RealPos.yaw) + sin(RealPos.roll)*sin(RealPos.yaw))*local_point->Z;

    global_point->Y = cos(RealPos.pitch)*sin(RealPos.yaw)*local_point->X + (sin(RealPos.roll)*sin(RealPos.pitch)*sin(RealPos.yaw) + cos(RealPos.roll)*cos(RealPos.yaw))*local_point->Y + (cos(RealPos.roll)*sin(RealPos.pitch)*sin(RealPos.yaw) - sin(RealPos.roll)*cos(RealPos.yaw))*local_point->Z;

    global_point->Z = -1*sin(RealPos.pitch)*local_point->X + sin(RealPos.roll)*cos(RealPos.pitch)*local_point->Y + cos(RealPos.roll)*cos(RealPos.pitch)*local_point->Z;

/* now push it out to its true location in space */

    global_point->X = global_point->X + RealPos.X;
    global_point->Y = global_point->Y + RealPos.Y;
    global_point->Z = global_point->Z + RealPos.Z;

  if ( global_point->Z <= 0.0 )
      return(-1);
   else
        return(0);
}

/*----------------------------------------------------------------------------*
 * make_ray_global() - converts a vehicle-relative unit normal ray into a     *
 * global-relative one by accounting for pitch, roll and yaw.  Returns a      *
 * pointer to that global ray.
 *---------------------------------------------------------------------------*/
void make_ray_global(struct ray_struct *global_ray, struct ray_struct *local_ray)
{
    int temp;
    struct point_struct temp_point, gtemp_point;

    temp_point.X = local_ray->X;  /* Origin of ray in local coords. */
    temp_point.Y = local_ray->Y;
    temp_point.Z = local_ray->Z;

    temp = make_point_global( &gtemp_point, &temp_point );

    global_ray->X = gtemp_point.X;
    global_ray->Y = gtemp_point.Y;
    global_ray->Z = gtemp_point.Z;

    global_ray->intensity = local_ray->intensity;

/* Assumes a unit ray was fed into it.  Accounts for pitch, yaw and roll. */

    global_ray->nx = cos(RealPos.pitch)*cos(RealPos.yaw)*local_ray->nx + (sin(RealPos.roll)*sin(RealPos.pitch)*cos(RealPos.yaw) - cos(RealPos.roll)*sin(RealPos.yaw))*local_ray->ny + (cos(RealPos.roll)*sin(RealPos.pitch)*cos(RealPos.yaw) + sin(RealPos.roll)*sin(RealPos.yaw))*local_ray->nz;

    global_ray->ny = cos(RealPos.pitch)*sin(RealPos.yaw)*local_ray->nx + (sin(RealPos.roll)*sin(RealPos.pitch)*sin(RealPos.yaw) + cos(RealPos.roll)*cos(RealPos.yaw))*local_ray->ny + (cos(RealPos.roll)*sin(RealPos.pitch)*sin(RealPos.yaw) - sin(RealPos.roll)*cos(RealPos.yaw))*local_ray->nz;

    global_ray->nz = -1*sin(RealPos.pitch)*local_ray->nx + sin(RealPos.roll)*cos(RealPos.pitch)*local_ray->ny + cos(RealPos.roll)*cos(RealPos.pitch)*local_ray->nz;

    global_ray->mag = sqrt( (global_ray->nx * global_ray->nx) + (global_ray->ny * global_ray->ny) + (global_ray->nz * global_ray->nz) );

    global_ray->nx = global_ray->nx/global_ray->mag;
    global_ray->ny = global_ray->ny/global_ray->mag;
    global_ray->nz = global_ray->nz/global_ray->mag;

}

/*---------------------------------------------------------------------------*
 * dmax() - returns max of two numbers.                                      *
 *---------------------------------------------------------------------------*/
double dmax(double N1, double N2)
{
    if(N1 >= N2) 
        return(N1);
    else 
        return(N2);
}

/*---------------------------------------------------------------------------*
 * dmin() - returns min of two numbers.                                      *
 *---------------------------------------------------------------------------*/
double dmin(double N1, double N2)
{
    if(N1 <= N2)
        return(N1);
    else
        return(N2);
}
/*----------------------------------------------------------------------------*
 * rmin - returns shorter of two rays                                         *
 *---------------------------------------------------------------------------*/
void rmin(struct ray_struct *R1, struct ray_struct *R2)
{
    if(R1->mag < R2->mag) {
        R2->mag = R1->mag;
        R2->X = R1->X;
        R2->Y = R1->Y;
        R2->Z = R1->Z;
        R2->nx = R1->nx;
        R2->ny = R1->ny;
        R2->nz = R1->nz;
        R2->phi_t = R1->phi_t;
        R2->phi_b = R1->phi_b;
        R2->intensity = R1->intensity;
        R2->P1 = R1->P1;
        R2->P2 = R1->P2;
        R2->P3 = R1->P3;
    }
}


/*----------------------------------------------------------------------------*
 * get_plane - given three (X,Y,Z) points, returns the plane defined by those *
 * points in two formats:                                                     *
 *                                                                            *
 * n = Xi + Yj + Zk (in unit normal format)                                   *
 *                                                                            *
 * Ax + By + Cz = D                                                           *
 *                                                                            *
 * From the equation of a plane, A = Nx, B = Ny, C = Nz.                      *
 *                                                                            *
 *---------------------------------------------------------------------------*/
int get_plane(struct triangle_struct *Tri, int k)
{
    double Xn, Yn, Zn, ray_len;        /* Normal vector components. */
    double Nx, Ny, Nz;                 /* Normal Unit vector components */
    struct bottom_point *P1, *P2, *P3; /* just to shorten up the equations. */
    double Xmin, Xmax, Xmid, Ymin, Ymax, Ymid;

    P1 = &GlobalBottom[Tri->P1];
    P2 = &GlobalBottom[Tri->P2];
    P3 = &GlobalBottom[Tri->P3];

    Xn = (P2->Y - P1->Y) * (P3->Z - P1->Z) - (P2->Z - P1->Z) * (P3->Y - P1->Y);
    Yn = -1 * ((P2->X - P1->X) * (P3->Z - P1->Z) - (P2->Z - P1->Z) * 
        (P3->X - P1->X));
    Zn = (P2->X - P1->X) * (P3->Y - P1->Y) - (P2->Y - P1->Y) * (P3->X - P1->X);
    ray_len = sqrt(Xn * Xn + Yn * Yn + Zn * Zn);
    if(ray_len == 0.0) {
        exit_gracefully(-1);
    }

    Nx = Xn/ray_len;
    Ny = Yn/ray_len;
    Nz = Zn/ray_len;
    Tri->A = Nx;
    Tri->B = Ny;
    Tri->C = Nz;
    Tri->D = Nx * P1->X + Ny * P1->Y + Nz * P1->Z;
/*
 * Get centroid of triangle.
 */
        Xmin = dmin( dmin(P1->X, P2->X), P3->X );
        Xmax = dmax( dmax(P1->X, P2->X), P3->X );
        Ymin = dmin( dmin(P1->Y, P2->Y), P3->Y );
        Ymax = dmax( dmax(P1->Y, P2->Y), P3->Y );
        if( (P1->X != Xmin) && (P1->X != Xmax) ) Xmid = P1->X;
        if( (P2->X != Xmin) && (P2->X != Xmax) ) Xmid = P2->X;
        if( (P3->X != Xmin) && (P3->X != Xmax) ) Xmid = P3->X;
        if( (P1->Y != Ymin) && (P1->Y != Ymax) ) Ymid = P1->Y;
        if( (P2->Y != Ymin) && (P2->Y != Ymax) ) Ymid = P2->Y;
        if( (P3->Y != Ymin) && (P3->Y != Ymax) ) Ymid = P3->Y;
        

        Tri->Xc = Xmin + (2/3) * (Xmax - Xmin) + (Xmax - Xmid)/3;
        Tri->Yc = Ymin + (2/3) * (Ymax - Ymin) + (Ymax - Ymid)/3;
        Tri->Zc = (Tri->D - (Tri->A * Tri->Xc) - (Tri->B * Tri->Yc))/Tri->C;
    return(0);
}

/*----------------------------------------------------------------------------*
 * get_range - given the plane defining the bottom (Tri) and the position of  *
 * the vehicle sensor (Mount) (not the same as the vehicle C.G.), return the  *
 * length (ray_d) of the perpendicular to the plane that intersects the sensor*
 * and point of intersection of that line on the plane (stored in any_ray).   *
 * It also creates the ray (any_ray) that points to the facet of interest     *
 * from the altimeter, referenced to the sonar surface.                       *
 *---------------------------------------------------------------------------*/
double get_range(struct triangle_struct *Tri, struct ray_struct *xducer, struct ray_struct *fac_ray)
{
    double t, ray_nx, ray_ny, ray_nz;
    double fooX, fooY, delta = 1.0;

    if( (reflect_model == SPECULAR) || (reflect_model == COMBINED) ) {

        t = ( Tri->D - (Tri->A * xducer->X) - (Tri->B * xducer->Y) - 
             (Tri->C * xducer->Z) ) / ( Tri->A*Tri->A + Tri->B*Tri->B + 
                                       Tri->C*Tri->C );
        
       fac_ray->X = Tri->A * t + xducer->X;  /* XYZ point of intersection on */
       fac_ray->Y = Tri->B * t + xducer->Y;  /* the facet of the ray between */
       fac_ray->Z = Tri->C * t + xducer->Z;  /* the sensor and the facet */
       
       ray_d = sqrt( (fac_ray->X - xducer->X) * (fac_ray->X - xducer->X) + 
                    (fac_ray->Y - xducer->Y) * (fac_ray->Y - xducer->Y) +
                    (fac_ray->Z - xducer->Z) * (fac_ray->Z - xducer->Z) );
   } else {
/*
 * In the non-specular case, represent the facet by its centroid.  Rays which
 * intersect the transducer are assumed to originate there.  This section 
 * calculates the range from the centroid to the transducer as well as deter-
 * mining the ray angle "phi" wrt the facet normal (phi_b) and the transducer
 * normal (phi_t).
 */
       fac_ray->X = Tri->Xc + xducer->X;
       fac_ray->Y = Tri->Yc + xducer->Y;
       fac_ray->Z = Tri->Zc + xducer->Z;
       ray_d = sqrt( (fac_ray->X - xducer->X) * (fac_ray->X - xducer->X) + 
                    (fac_ray->Y - xducer->Y) * (fac_ray->Y - xducer->Y) +
                    (fac_ray->Z - xducer->Z) * (fac_ray->Z - xducer->Z) );
   }
/* First facet relative.  Get phi_b */
    ray_nx = xducer->X - fac_ray->X;
    ray_ny = xducer->Y - fac_ray->Y;
    ray_nz = xducer->Z - fac_ray->Z;

    ray_nx = ray_nx/ray_d;
    ray_ny = ray_ny/ray_d;
    ray_nz = ray_nz/ray_d;

/* 
 * Note that Tri->A - Nx, Tri->B = Ny, Tri->C = Nz (unit normals) for facet.
 * EXCEPT that, because of the coordinate system, the normal is defined as
 * positive downwards.  For the purposes of determining the angle of
 * intersection, however, positive is defined as upwards.  So the sign must
 * be changed before phi_b can be calculated.
 */

    fac_ray->nx = -1.0 * Tri->A;
    fac_ray->ny = -1.0 * Tri->B;
    fac_ray->nz = -1.0 * Tri->C;

    fac_ray->phi_b = acos( (ray_nx * fac_ray->nx) + (ray_ny * fac_ray->ny) + 
                          (ray_nz * fac_ray->nz) );

/* Now xducer relative.  Get phi_t */
    ray_nx = -1 * ray_nx;
    ray_ny = -1 * ray_ny;
    ray_nz = -1 * ray_nz;
    
    fac_ray->phi_t = acos( (ray_nx * xducer->nx) + (ray_ny * xducer->ny) + 
                          (ray_nz * xducer->nz) );
    fac_ray->mag = ray_d;
    fac_ray->P1 = Tri->P1;
    fac_ray->P2 = Tri->P2;
    fac_ray->P3 = Tri->P3;

    if (debug == RAYCOMP) {
        fooX = fabs(RealPos.X - Tri->Xc);
        fooY = fabs(RealPos.Y - Tri->Yc);
    }

    return(ray_d);
}

/*----------------------------------------------------------------------------*
 * in_triangle - given the triangular bottom facet and the point of           *
 * intersection of a perpendicular ray between the vehicle sensor and the     *
 * plane which that facet defines, determine if that point of intersection is *
 * inside or outside of the facet.  To do this, look at the cross product of  *
 * two vectors; one between two points of the triangle (one edge) and the     *
 * other between one point of the triangle and the point of intersection.  Go *
 * counterclockwise about the triangle.  If the cross product is >= 0 then the*
 * point is outside.  If not, then it may be inside, so check the next        *
 * triangle side.  Keep checking until all three sides are confirmed or any   *
 * one is not.                                                                *
 *                                                                            *
 * If the point is in the triangle, return (1) and the Z coordinate of the    *
 * point of intersection. if not, return (0).                                 *
 *----------------------------------------------------------------------------*/
int in_triangle(struct triangle_struct *Tri, struct ray_struct *fac_ray)
{
    double A, B, C, D;
    double temp1, temp2, temp3;
    struct bottom_point *P1, *P2, *P3; /* just to shorten up the equations. */
    double fooX, fooY, delta = 1.0;

    P1 = &GlobalBottom[Tri->P1];
    P2 = &GlobalBottom[Tri->P2];
    P3 = &GlobalBottom[Tri->P3];

    if (debug != LOCATOR) {
            
        temp1 = (fac_ray->X * (P1->Y - P2->Y)) - (fac_ray->Y * (P1->X - P2->X))
            + ((P1->X * P2->Y) - (P1->Y * P2->X));
        temp2 = (fac_ray->X * (P2->Y - P3->Y)) - (fac_ray->Y * (P2->X - P3->X))
            + ((P2->X * P3->Y) - (P2->Y * P3->X));
        temp3 = (fac_ray->X * (P3->Y - P1->Y)) - (fac_ray->Y * (P3->X - P1->X))
            + ((P3->X * P1->Y) - (P3->Y * P1->X));
        
        if( temp1 <= 0.0 ) {
            fac_ray->mag = MAX_RANGE;
            return(0);
        } 
        else if( temp2 <= 0.0 ) {
            fac_ray->mag = MAX_RANGE;
            return(0);
        }
        else if( temp3 <= 0.0 ) {
            fac_ray->mag = MAX_RANGE;
            return(0);
        } else {

/*  Unnecessary.  Done in get_plane. 
            A = (P2->Y - P1->Y) * (P3->Z - P1->Z) - (P2->Z - P1->Z) * 
                (P3->Y - P1->Y);
            B = (P2->Z - P1->Z) * (P3->X - P1->X) - (P2->X - P1->X) * 
                (P3->Z - P1->Z);
            C = (P2->X - P1->X) * (P3->Y - P1->Y) - (P2->Y - P1->Y) * 
                (P3->X - P1->X);
            D = -1 * (A * P2->X + B * P2->Y + C * P2->Z);
            fac_ray->Z = ( A * fac_ray->X + B * fac_ray->Y + D) / (-1 * C);
*/
            return(1);
        }
        
    } else {             /* A temporary kludge until I find the bug in */
                         /* the above section. */
        
        fooX = fabs(RealPos.X - Tri->Xc);
        fooY = fabs(RealPos.Y - Tri->Yc);
        if( (fooX <= delta ) && (fooY <= delta ) ) {
            A = (P2->Y - P1->Y) * (P3->Z - P1->Z) - (P2->Z - P1->Z) * 
                (P3->Y - P1->Y);
            B = (P2->Z - P1->Z) * (P3->X - P1->X) - (P2->X - P1->X) * 
                (P3->Z - P1->Z);
            C = (P2->X - P1->X) * (P3->Y - P1->Y) - (P2->Y - P1->Y) * 
                (P3->X - P1->X);
            D = -1 * (A * P2->X + B * P2->Y + C * P2->Z);
            fac_ray->Z = ( A * fac_ray->X + B * fac_ray->Y + D) / (-1 * C);
            return(1);
        } else 
            fac_ray->mag = MAX_RANGE;
    }
}

/*----------------------------------------------------------------------------*
 * beam_pattern - the beam pattern for a circular disk transducer.  Given the *
 * incident ray, and assuming a wavelength and transducer size (defined at the*
 * beginning of this file), return the relative strength of the signal        *
 * intensity in db.                                                           *
 *----------------------------------------------------------------------------*/
double beam_pattern( struct ray_struct *fac_ray )
{
    double temp1, temp2, loss;

    if( debug == NOBEAM) {
        loss = 0;
    } else {
        temp1 = PI * DIAMETER / LAMBDA;
        temp2 = (2.0 * bessj1(temp1 * sin(fac_ray->phi_t))) / 
            ( temp1 * sin(fac_ray->phi_t) );
        loss =  10.0 * log10( (temp2 * temp2) );
    }
    fac_ray->intensity = fac_ray->intensity + loss;
    return(fac_ray->intensity);
}

/*----------------------------------------------------------------------------*
 * trans_loss - given the length of the ray and its initial intensity, and    *
 * assuming a frequency of 500 khz (and therefore a frequency dependant       *
 * absorbtion constant of approx. 100 dB/km), return the final intensity      *
 * due to geometric spreading and frequency-dependant absorbtion.  Note       *
 * that the absorbtion is doubled to account for the round trip path.  This   *
 * assumes a monostatic case.                                                 *
 *----------------------------------------------------------------------------*/
double trans_loss( struct ray_struct *fac_ray)
{
    double net, alpha, freq2, loss;

    freq2 = FREQ * FREQ;

    alpha = ( (0.1 * freq2/(1 + freq2)) + (40.0 * freq2/(4100.0 + freq2)) + (2.75e-4 * freq2) + 0.003 ) * KYD_TO_KM;
/*
 * net has the '2*' factor for an assumed round trip.  This means a monostatic 
 * sonar is assumed.
 */
    net = 2.0*(20.0*log10(fac_ray->mag)) + ((alpha*fac_ray->mag)/1000.0);
    fac_ray->intensity - fac_ray->intensity - net;
    return(fac_ray->intensity);
}

/*----------------------------------------------------------------------------*
 * bessj1 - from Numerical Recipes in C.  First order Bessel Function J1(x)   *
 * used to model the beam pattern of a circular disk sonar transducer.        *
 *----------------------------------------------------------------------------*/
double bessj1(double x)
{
        double ax,z;
        double xx,y,ans,ans1,ans2;

        if ((ax=fabs(x)) < 8.0) {
                y=x*x;
                ans1=x*(72362614232.0+y*(-7895059235.0+y*(242396853.1
                        +y*(-2972611.439+y*(15704.48260+y*(-30.16036606))))));
                ans2=144725228442.0+y*(2300535178.0+y*(18583304.74
                        +y*(99447.43394+y*(376.9991397+y*1.0))));
                ans=ans1/ans2;
        } else {
                z=8.0/ax;
                y=z*z;
                xx=ax-2.356194491;
                ans1=1.0+y*(0.183105e-2+y*(-0.3516396496e-4
                        +y*(0.2457520174e-5+y*(-0.240337019e-6))));
                ans2=0.04687499995+y*(-0.2002690873e-3
                        +y*(0.8449199096e-5+y*(-0.88228987e-6
                        +y*0.105787412e-6)));
                ans=sqrt(0.636619772/ax)*(cos(xx)*ans1-z*sin(xx)*ans2);
                if (x < 0.0) ans = -ans;
        }
        return ans;
}
/*----------------------------------------------------------------------------*
 * ray_mag - Return the magnitude (length) of a given ray.                    *
 *----------------------------------------------------------------------------*/
double ray_mag( struct ray_struct *fac_ray )
{
    double mag;

    mag = sqrt( (fac_ray->X * fac_ray->X) +  (fac_ray->Y * fac_ray->Y) + (fac_ray->Z * fac_ray->Z) );

    return(mag);
}
/*----------------------------------------------------------------------------*
 * bottom_reflect - given the location of the bottom-ray intersection, the    *
 * intensity of the incident ray and the triangle of intersection, determine  *
 * the reflectivity of the bottom and the intensity of the reflected ray.     *
 * Update the ray structure accordingly.  The method of calculation depends on*
 * if it is a SPECULAR or LAMBERTIAN reflection model.                        *
*-----------------------------------------------------------------------------*/
double bottom_reflect( struct ray_struct *fac_ray, int reflect_model )
{
    double temp1;

    if(( reflect_model == SPECULAR) ||  (reflect_model == COMBINED)) {
        return(fac_ray->intensity);
    } else {
/* 
 * Assumed to be lambertian scattering, rough bottom, with no absorbtion
 * (not all that realistic for high frequency and silt, but it'll do for now).
 * See Urick's book, pp. 278
 */
        temp1 =  cos(fac_ray->phi_b);
        fac_ray->intensity = fac_ray->intensity - 5 - 
            (10 * log10( temp1 * temp1 ));
        return(fac_ray->intensity);
    }
}

/*----------------------------------------------------------------------------*
 * load_data - load in seabed data file.                                      *
 *                                                                            *
 *   DATA FILE FORMAT:  Data files can be in many formats.  The first items   *
 * in the data file are comments (first and last characters defining the      *
 * comment field are ':').  The first part states what format the file is,    *
 * the second part states the size of the file in the appropriate units.      *
 *                                                                            *
 * Currently recognised formats are:                                          *
 *                                                                            *
 *      Implicit Array - A 2-dimensional array of depth values.  The X and Y  *
 * coordinates are implicit in the array vertices,  the depth is stored at    *
 * the appropriate vertex.  Stored in binary as 4-byte reals, with file size  *
 * of DIMENXxDIMENYx4.  Note also that in the Indy, type double is 8-bytes    *
 * and float is 4-bytes.                                                      *
 *    First Line                                                              *
 *         :Implicit [4 or 8 byte] [# rows] [#columns]:                       *
 *                                                                            *
 *      XYZ Array - An array of three columns by n rows.  Assumed to be in    *
 * ASCII format,  seperated by spaces.  Each row is one XYZ combination.      *
 *      First Line:  :XYZ [# rows]:                                           *
 *                                                                            *
 *      Triangulated Array - An array of pre-triangulated data points.        *
 * There are really three arrays in the file:                                 *
 *    The first is all of the vertices in XYZ data point format.  Their index *
 *      number is their place in the sequence (i.e. the third XYZ data point  *
 *      is Vertex #3).                                                        *
 *    The second is the list of all the dependencies of each vertex (how many *
 *      other vertices are adjacent to the current one [integer format].      *
 *    The third is a list of the explicit index numbers in a counterclockwise *
 *      sequence about the point if interest (i.e. if index #1 has five       *
 *      neighbors which happen to be #'s 6, 5, 12, 18, and 22, then the entry *
 *      looks like:  6 5 12 18 22                                             *
 *    All three arrays are the same length, so only one size indicator is     *
 *      required at the beginning of the file.                                *
 *    First Line                                                              *
 *         :TRIANG [# nodes]:                                                 *
 *                                                                            *
 *---------------------------------------------------------------------------*/

int load_data( void )
{

    char filetype[80];
    char *Seabed;           /* name of seabed bottom data file. */
    FILE *bottomfile;       /* pointer to bottom data file */
    FILE *trifile;          /* pointer to file of triangles */
    double zoffset = 0.0;   /* Offset applied to nodes. Large is deep.*/
    double xoffset = 0.0;   /* Offset to determine map origin */
    double yoffset = 2000.0;/* Offset to determine map origin */
    double Xdelta = 22.0;   /* EW distance between grid points */
                            /*  (implicit format uses 6.5) */
                            /* Charles.tri is in 100ths of min. Delta = 22 */
    double Ydelta = -15.0;  /* NS distance between grid points */
                            /*  (implicit format uses 6.5) */
                            /* Charles.tri is in 100ths of min. Delta = 15 */
    double zscale = 1.0;    /* Vertical scaling applied to bottom */
    double tempinput[MAXROWS][2];  /* To be fed into the delaunay converter */
    int tempoutput[MAXROWS][3];    /* Takes the resulting triangle nodes */
    int n_input, n_output;
        int conv, status;
        
    Seabed = getenv("BOTTOMFILE");
    if(Seabed == NULL) {
          Seabed = "env/bottomfile";
    } 
    bottomfile = fopen( Seabed, "r" );
    if( !bottomfile ) { 
          perror( Seabed );
          exit(-1);        
    }
    conv = fscanf(bottomfile, ":%s ", filetype );
    if (strncmp(filetype,"Implicit", 8) == 0) {
          conv = fscanf(bottomfile, "%d %d %d:", &bytecount,  &nrows,  &ncols );
          if ( conv != 3 ) {
            exit( -1 );
          }
/*
 * Create the bottom array to hold the data points.
 */
          GlobalBottom = (struct bottom_point *) calloc( (nrows * ncols) + 1, 
                                                 sizeof(struct bottom_point) );
          k = 0;
          for(i = 0; i <= nrows; i++)
            fread(bottom_array+i, bytecount, ncols, bottomfile);
          for(i = 0; i < nrows; i++) {
            for(j = 0; j < ncols; j++) {
                  bottom[i][j] = (zoffset - bottom_array[i][j]) * zscale;
                  GlobalBottom[k].X = ((double) i * Xdelta) + xoffset;
                  GlobalBottom[k].Y = ((double) j * Ydelta) + yoffset;
                  GlobalBottom[k].Z = bottom[i][j];
                  tempinput[k][1] = GlobalBottom[k].X;
                  tempinput[k][2] = GlobalBottom[k].Y;
                  k++;
            }
          }
    } else if(strncmp(filetype,"XYZ", 3) == 0) {
          conv = fscanf(bottomfile, "%d:", &nrows );
          if ( conv != 1 ) {
            return(-1);
          }
          if(nrows > MAXROWS) {
            return(-1);
          }
/*
 * Create the bottom array to hold the data points.
 */
          GlobalBottom = (struct bottom_point *) calloc( nrows+1, 
                                                        sizeof(struct bottom_point) );
          for(i = 1; i <= nrows; i++)
            fscanf(bottomfile,"%lf%lf%lf", 
                           GlobalBottom[i].X, GlobalBottom[i].Y, GlobalBottom[i].Z);
    } else if(strncmp(filetype,"TRIANG", 6) == 0) {
          TriangFlag = TRUE;
/*
 * Read number of nodes.
 */
          conv = fscanf(bottomfile, "%d:", &nrows );
          if ( conv != 1 ) {
            return(-1);
          }
          if (nrows > MAXROWS) {
            return(-1);
          }
/*
 * Create the bottom array to hold the data points.
 */
          GlobalBottom = (struct bottom_point *) calloc( nrows+1, 
                                                 sizeof(struct bottom_point) );
/*
 * Read in the (x, y, z) coordinates of the nodes.
 */
          for(i = 1; i <= nrows; i++) {
            fscanf(bottomfile,"%lf %lf %lf", &GlobalBottom[i].X, 
                           &GlobalBottom[i].Y, &GlobalBottom[i].Z);
            GlobalBottom[i].X =  (GlobalBottom[i].X * Xdelta) + xoffset;
            GlobalBottom[i].Y =  (GlobalBottom[i].Y * Ydelta) + yoffset;
            GlobalBottom[i].Z = fabs( GlobalBottom[i].Z ) + zoffset;
          } /* To insure positive Z is down */
/*
 * Create the adjacency table.
 */
          adj = (int *) calloc( nrows, sizeof(int) );
          Order = (struct order_struct *) calloc( nrows+1, 
                                                                  sizeof(struct order_struct) );
/*
 * Read in the number of adjacencies for each row.
 */
          for(i = 1; i <= nrows; i++)  /* Picks up rows (nrows+1) to (2*nrows) */
            fscanf( bottomfile,"%d", &adj[i] );
          Order[0].adjs = 0;
          Order[0].node = (int *) calloc( 1, sizeof(int) );
/*
 * Create adjacency tables for each node.
 */
          for(i = 1; i <= nrows; i++)  {
            Order[i].adjs = adj[i] - adj[(i-1)];
            Order[i].node = (int *) calloc( (adj[i] - adj[(i-1)] + 1), 
                                                                                sizeof(int) );
          }
/*
 * Load in all adjacencies.
 */
          for(i = 1; i <= nrows; i++)
            for( j = 1; j <= Order[i].adjs; j++) {
                  conv = fscanf(bottomfile, " %d", &(Order[i].node[j]));
            }
/*
 *   Run through the triangles, just to be sure we're getting them correctly.
 * This also determines exactly how many triangles there are in the bottom 
 * file.  For every node i, look at the list of adjacent nodes j.  The nodes 
 * forming the triangle associated with the node i are (i, j, j+1).
 *   If we are looking at the next-to-last vertex, check to see if the last 
 * vertex is an edge (the node number = 0).  If so, then no more triangles are
 * associated with that node.  If not, then the last triangle is (i, j=1, 
 * last_j).  For example, if node number 1 has the adjacencies: (18 5 6 7 2 0)
 * then the triangles associated with node 1 are (counterclockwise about 1): 
 * [1 18 5],[1 5 6],[1 6 7],[1 7 2].  If the adjacencies associated with node
 * 1 are: (18 5 6 7 2) then the triangles are [1 18 5],[1 5 6],[1 6 7],[1 7 2]
 * and [1 2 18].
 */
          for (i = 1; i <= nrows; i++) {
            for (j = 1; j < Order[i].adjs; j++)  {
                  if ( j == (Order[i].adjs-1) ) {
                    if (Order[i].node[j+1] != 0) {
                          num_triangles++;
                          num_triangles++;
                    } 
                  } else {
                    num_triangles++;
                  }
            }
          }
          Triangle = (struct triangle_struct *) calloc( num_triangles+1, 
                                                        sizeof(struct triangle_struct) );
/*
 * Load up the Triangles with node information.
 */
          num_triangles = 0;
          for (i = 1; i <= nrows; i++) {
            for (j = 1; j < Order[i].adjs; j++)  {
                  if ( j == (Order[i].adjs-1) ) {
                    if (Order[i].node[j+1] != 0) {
                          num_triangles++;
                          Triangle[num_triangles].P1 = i;
                          Triangle[num_triangles].P2 = Order[i].node[j];
                          Triangle[num_triangles].P3 = Order[i].node[j+1];
                          num_triangles++;
                          Triangle[num_triangles].P1 = i;
                          Triangle[num_triangles].P2 = Order[i].node[j+1];
                          Triangle[num_triangles].P3 = Order[i].node[1];
                    } else
                          continue;
                  } else {
                    num_triangles++;
                    Triangle[num_triangles].P1 = i;
                    Triangle[num_triangles].P2 = Order[i].node[j];
                    Triangle[num_triangles].P3 = Order[i].node[j+1];
                  }
            }
          }
    } else {
          return(-1);
    }
    if( debug == EXTRAFILE ){
          trifile = fopen("Tri_Charles.dat","w");
          for(i = 1; i<=num_triangles; i++) {
            fprintf(trifile, "%.2lf %.2lf %.2lf\n %.2lf %.2lf %.2lf\n %.2lf"
                                " %.2lf %.2lf\n", GlobalBottom[Triangle[i].P1].X, 
                                GlobalBottom[Triangle[i].P1].Y, 
                                GlobalBottom[Triangle[i].P1].Z,
                                GlobalBottom[Triangle[i].P2].X,
                                GlobalBottom[Triangle[i].P2].Y,
                                GlobalBottom[Triangle[i].P2].Z,
                                GlobalBottom[Triangle[i].P3].X,
                                GlobalBottom[Triangle[i].P3].Y,
                                GlobalBottom[Triangle[i].P3].Z);
          }
          i = fclose( trifile );
    }
    return(0);
} /* END OF LOAD_DATA */

/*----------------------------------------------------------------------------*
 * triangle_search - determines if the vehicle position is in the data field  *
 * or not.  If so, then returns the depth at the vehicle location.  If not,   *
 * it warns the user.                                                         *
 *---------------------------------------------------------------------------*/
double triangle_search(struct state_vect *RealPos)
{
    int i, j;
    double A, B, C, D, Depth;
    double temp1, temp2, temp3, time;
    struct bottom_point *P1, *P2, *P3; /* just to shorten up the equations. */
/*
 * If the point is to the right, then get the adjacent triangle.  If it is
 * not, then go to the next line segment (triangle side) and check that one.
 *
 * "right of" means outside the triangle.
 */
    time = read_sensor(M_PRESENT_TIME);
    if( Near_Node == -1 ) start = 1;
    else start = Near_Node;
    start = 1;
    for (i = start; i <= nrows; i++) {
        j = 1;
        while( j < Order[i].adjs) {
            if( Order[i].node[j+1] == 0) {
                P1 = &GlobalBottom[i];
                P2 = &GlobalBottom[Order[i].node[j]];
                P3 = &GlobalBottom[Order[i].node[1]];
            } else {
                P1 = &GlobalBottom[i];
                P2 = &GlobalBottom[Order[i].node[j]];
                P3 = &GlobalBottom[Order[i].node[j+1]];
            }
            temp1 =  RealPos->X * (P1->Y - P2->Y) - RealPos->Y * 
                (P1->X - P2->X) + (P1->X * P2->Y - P1->Y * P2->X);
            temp2 =  RealPos->X * (P2->Y - P3->Y) - RealPos->Y * 
                (P2->X - P3->X) + (P2->X * P3->Y - P2->Y * P3->X);
            temp3 =  RealPos->X * (P3->Y - P1->Y) - RealPos->Y * 
                (P3->X - P1->X) + (P3->X * P1->Y - P3->Y * P1->X);

            if( temp1 >= 0.0 ) {
                if( Order[i].node[j] != 0 ) j++;
                else { 
                    exit_gracefully(-1);
                }
            }
            else if( temp2 >= 0.0 ) {
                if( Order[i].node[j+1] != 0 ) j++;
                else { 
                    exit_gracefully(-1);
                }
            }
            else if( temp3 >= 0.0 ) {
                if( Order[i].node[j+1] != 0 ) j++;
                else { 
                    exit_gracefully(-1);
                }
            }
            else {
                A = (P2->Y - P1->Y) * (P3->Z - P1->Z) - (P2->Z - P1->Z) * 
                    (P3->Y - P1->Y);
                B = (P2->Z - P1->Z) * (P3->X - P1->X) - (P2->X - P1->X) * 
                    (P3->Z - P1->Z);
                C = (P2->X - P1->X) * (P3->Y - P1->Y) - (P2->Y - P1->Y) * 
                    (P3->X - P1->X);
                D = -1 * ( A * P2->X + B * P2->Y + C * P2->Z );
                Depth = ( A * RealPos->X + B * RealPos->Y + D ) / ( -1 * C );
                Near_Node = i;

                if( log_mode == FULL ) {
                  update_sensor(REAL_TRIANGLE_X1, P1->X);
                  update_sensor(REAL_TRIANGLE_Y1, P1->Y);
                  update_sensor(REAL_TRIANGLE_Z1, P1->Z);
                  update_sensor(REAL_TRIANGLE_X2, P2->X);
                  update_sensor(REAL_TRIANGLE_Y2, P2->Y);
                  update_sensor(REAL_TRIANGLE_Z2, P2->Z);
                  update_sensor(REAL_TRIANGLE_X3, P3->X);
                  update_sensor(REAL_TRIANGLE_Y3, P3->Y);
                  update_sensor(REAL_TRIANGLE_Z3, P3->Z);
                  
                  update_sensor(REAL_CONTACT_X, RealPos->X);
                  update_sensor(REAL_CONTACT_Y, RealPos->Y);
                  update_sensor(REAL_CONTACT_Z, Depth); 
                }
                if( reflect_model == COMBINED ) {
                    diffray.mag = Depth;
                    diffray.X = RealPos->X;
                    diffray.Y = RealPos->Y;
                    diffray.Z = RealPos->Z;
                    diffray.nx = -1.0;
                    diffray.ny = -1.0;
                    diffray.nz = -1.0;
                    diffray.phi_t = 0.0;
                    diffray.phi_b = 0.0;
                    diffray.intensity = DETECT_THRESH + 1.0;
                    diffray.P1 = i;
                    diffray.P2 = Order[i].node[j];
                    if( Order[i].node[j+1] == 0) {
                        diffray.P3 = Order[i].node[1];
                    } else {
                        diffray.P3 = Order[i].node[j+1];
                    }
                }
                return( Depth );
            }
        }
    }
}


/*----------------------------------------------------------------------------*
 * bottom_collision - determines if the vehicle hits the bottom.  Does so
 * by looking at a series of points which define the underside of teh vehicle.
 * If so, returns a warning to the user.
 *
 * Later, this routine will return the collison node, its compliance (harder
 * complience means stiffer bottm and/or vehicle part, softer compliance 
 * means more give.  e.g. tailfin is more compliant in mud then the nose).
 * and the amount of overlap, which affects the collison force.
 *---------------------------------------------------------------------------*/
double bottom_collision(struct state_vect *RealPos)
{
    int i, j, k, nodes = 10;
    double A, B, C, D;
    double temp1, temp2, temp3, time;
    struct bottom_point *P1, *P2, *P3; /* just to shorten up the equations. */
    struct node_struct Node[11];
    struct node_struct minnode;

    for (i = 1; i <= 11; i++ ) {
        Node[i].Xo = 0.0;
        Node[i].Yo = 0.0;
        Node[i].Zo = 0.0;
        Node[i].compliance = 0.0;
        Node[i].dist = MAX_RANGE;
    }
/*
 * If the point is to the right, then get the adjacent triangle.  If it is
 * not, then go to the next line segment (triangle side) and check that one.
 *
 * "right of" means outside the triangle.
 */
    minnode.dist = MAX_RANGE;
    time = read_sensor(M_PRESENT_TIME);

    if( Near_Node == -1 ) start = 1;
    else start = Near_Node;
    start = 1;
    for( k = 1; k <= nodes; k++) {
        for (i = start; i <= nrows; i++) {
            j = 1;
            while( j < Order[i].adjs) {
                if( Order[i].node[j+1] == 0) {
                    P1 = &GlobalBottom[i];
                    P2 = &GlobalBottom[Order[i].node[j]];
                    P3 = &GlobalBottom[Order[i].node[1]];
                } else {
                    P1 = &GlobalBottom[i];
                    P2 = &GlobalBottom[Order[i].node[j]];
                    P3 = &GlobalBottom[Order[i].node[j+1]];
                }
                temp1 =  (RealPos->X + Node[k].Xo) * (P1->Y - P2->Y) - 
                    (RealPos->Y + Node[k].Yo) * (P1->X - P2->X) + 
                        (P1->X * P2->Y - P1->Y * P2->X);
                temp2 =  (RealPos->X + Node[k].Xo) * (P2->Y - P3->Y) - 
                    (RealPos->Y + Node[k].Yo) * (P2->X - P3->X) + 
                        (P2->X * P3->Y - P2->Y * P3->X);
                temp3 =  (RealPos->X + Node[k].Xo) * (P3->Y - P1->Y) - 
                    (RealPos->Y + Node[k].Yo) * (P3->X - P1->X) + 
                        (P3->X * P1->Y - P3->Y * P1->X);
                if( temp1 <= 0.0 ) {
                    if( Order[i].node[j] != 0 )
                        j++;
                    else { 
                        exit_gracefully(-1);
                    }
                }
                else if( temp2 <= 0.0 ) {
                    if( Order[i].node[j+1] != 0 ) 
                        j++;
                    else { 
                        exit_gracefully(-1);
                    }
                }
                else if( temp3 <= 0.0 ) {
                    if( Order[i].node[j+1] != 0 ) 
                        j++;
                    else { 
                        exit_gracefully(-1);
                    }
                }
                else {
                    A = (P2->Y - P1->Y) * (P3->Z - P1->Z) - (P2->Z - P1->Z) * 
                        (P3->Y - P1->Y);
                    B = (P2->Z - P1->Z) * (P3->X - P1->X) - (P2->X - P1->X) * 
                        (P3->Z - P1->Z);
                    C = (P2->X - P1->X) * (P3->Y - P1->Y) - (P2->Y - P1->Y) * 
                        (P3->X - P1->X);
                    D = -1 * ( A * P2->X + B * P2->Y + C * P2->Z );
                    Node[k].dist = ( A * (RealPos->X + Node[k].Xo) + B * 
                             (RealPos->Y + Node[k].Yo) + D ) / ( -1 * C );
                    Near_Node = i;
                }
            }
        }
        nodemin( &Node[k], &minnode );
/* Need to update with the needed information for sim to model the collison.
 * e.g. compliance, node coordinates and/or position relative to C.G.
 *
        update_sensor( CONTACT_NODE_Xo, minnode.Xo );
        update_sensor( CONTACT_NODE_Yo, minnode.Yo );
        update_sensor( CONTACT_NODE_Zo, minnode.Zo );
        update_sensor( CONTACT_NODE_DIST, minnode.dist );
        update_sensor( CONTACT_NODE_COMP, minnode.compliance );
*/
    }
    return( minnode.dist );
}

/*----------------------------------------------------------------------------*
 * rmin - returns shorter of two rays                                         *
 *---------------------------------------------------------------------------*/
void nodemin(struct node_struct *N1, struct node_struct *N2)
{
    if(N1->dist < N2->dist) {
        N2->dist = N1->dist;
        N2->Xo = N1->Xo;
        N2->Yo = N1->Yo;
        N2->Zo = N1->Zo;
        N2->compliance = N1->compliance;
    }
}

