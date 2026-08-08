/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: sim.c,v 2.21 1998/01/20 00:31:51 bamoran Exp $
 *----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*
 | sim.c    Odyssey II hydrodynamic simulator with Franz Hover's updates |
 |                                                                       |
 | Last mod:  6/9/95 Franz Hover                                         |
 | History: copied to SGI from Mac by June 1, 1994 by jleonard           |
 |          compilation of vehicle simulation routines written by        |
 |          Jim Bellingham and other Sea Grant staff (198?-1994)         |
 |          90% of the code came from Draper, a student of Abkowitz.     |
 |                                                                       |
 |          From March to June, 1995 this code was significantly         |
 |          improved by Franz Hover.  Errors were removed and the        |
 |          model was updated based on theoretical considerations,       |
 |          Draper's modeling program, and analysis of mission data      |
 |                                                                       |
 | NOTE: all vehicle physical parameters (mass, inertia, CG/CB,         |
 | hydrodynamic and fin coefficients) are read from vs_* variables in   |
 | the data structure, populated by load_vehicle_def() before           |
 | init_simulation() is called.  There are no external coefficient      |
 | files.                                                                |
 *-----------------------------------------------------------------------*/

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "common_defs.h"
#include "vehicle.h"
#include "odyssey_data_struct.h"

/*-----------------------------------------------------------------------*
 | structure of this file: 
 |   global variables declarations       (there are a lot of them)
 |   main()                              simple interface to test (for now)
 |   start_sub(),                        calls lhs
 |   lhs()                               precompute lhs of hydro equations
 |   motion()                            main computation
 |   hydro()                             calcs rhs of hydro equations
 |   fins()                              computes forces caused by fins
 |   act_model()                         fin actuator model (just a delay) 
 |   thruster_force(),thruster_torque()  thruster model
 |   matinv()                            matrix routines
 | 
 *-----------------------------------------------------------------------*/


/*------------------- function prototypes -------------------------------*/

void lhs();
void fins(double *, double, double);

/*------------------- generic constants -------------------------------*/

#define FTOM   0.3048               /* from feet to meters */
#define FTOM2  (0.3048 * 0.3048)
#define FTOM3  (0.3048 * 0.3048 * 0.3048)

const double rho = 1.025e3;	/* kg/m^3 density of seawater */

/*------------------- global variables -------------------------------*/

static double rhs[7];       /* calculated in hydro, used in motion */
static double ainv[7][7];   /* calculated by lhs, used in motion */

/* coordinate transformation matrix, we use only indices 1-6 */
static double ctrn[7][7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
			    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
			    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
			    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
			    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
			    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
			    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};              

static int jjj;  /* runge-kutta index (shouldn't be global) but is ? */

/*---- Forces and moments due to the fins ----*/
static double F1[4],F2[4],F3[4],F4[4];
static double M1[4],M2[4],M3[4],M4[4];


/*----------------------- vehicle parameters ------------------------------*/

/* All vehicle physical parameters are read from vs_* variables in the
   data structure.  Defaults live in edit_struct.c; runtime values are
   loaded by load_vehicle_def() from the vehicle definition (.vd) file
   before init_simulation() is called.  There are no bare globals or
   external coefficient files here.

   x is fwd, y stbd, z down.

   Fin numbering: #1 lower rudder, #2 port elevator,
                  #3 upper rudder,  #4 stbd elevator.
   Fin model from PNA / Whicker & Fehlner 1958.

   Thruster model from J. G. Bellingham 1/94, fsh 5/95.          */


/*----------------------------------------------------------------------*/

void lhs()			/* calculates left-hand-side of the eqn */
{
    double a[7][7],work[7][7];
    int i, j, debug=0;

				/* fsh checked inertia matrix 3/22/94 */

    a[1][1] =  read_sensor(VS_MASS) - read_sensor(VS_XUDOT);	/* surge force */
    a[1][2] = 0.;
    a[1][3] = 0.;
    a[1][4] = 0.;
    a[1][5] = read_sensor(VS_MASS) * read_sensor(VS_ZG);
    a[1][6] = -read_sensor(VS_MASS) * read_sensor(VS_YG);

    a[2][1] = 0.;		/* sway force */
    a[2][2] =  read_sensor(VS_MASS) - read_sensor(VS_YVDOT) ;
    a[2][3] = 0.;
    a[2][4] = -read_sensor(VS_MASS) * read_sensor(VS_ZG) - read_sensor(VS_YPDOT);
    a[2][5] = 0.;
    a[2][6] =  read_sensor(VS_MASS) * read_sensor(VS_XG) - read_sensor(VS_YRDOT);

    a[3][1] = 0.;		/* heave force */
    a[3][2] = 0.;
    a[3][3] =  read_sensor(VS_MASS) - read_sensor(VS_ZWDOT);
    a[3][4] =  read_sensor(VS_MASS) * read_sensor(VS_YG);
    a[3][5] = -read_sensor(VS_MASS) * read_sensor(VS_XG) - read_sensor(VS_ZQDOT);
    a[3][6] = 0.;
  
    a[4][1] = 0.;		/* roll moment */
    a[4][2] = -read_sensor(VS_MASS) * read_sensor(VS_ZG) - read_sensor(VS_KVDOT);
    a[4][3] =  read_sensor(VS_MASS) * read_sensor(VS_YG);
    a[4][4] =  read_sensor(VS_IXX) - read_sensor(VS_KPDOT);
    a[4][5] = -read_sensor(VS_IXY);
    a[4][6] = -read_sensor(VS_IZX);
  
    a[5][1] =  read_sensor(VS_MASS) * read_sensor(VS_ZG);	/* pitch moment */
    a[5][2] = 0.;
    a[5][3] = -read_sensor(VS_MASS) * read_sensor(VS_XG) - read_sensor(VS_MWDOT);
    a[5][4] = -read_sensor(VS_IXY);
    a[5][5] =  read_sensor(VS_IYY) - read_sensor(VS_MQDOT);
    a[5][6] = -read_sensor(VS_IYZ);

    a[6][1] = -read_sensor(VS_MASS) * read_sensor(VS_YG);	/* yaw moment */
    a[6][2] =  read_sensor(VS_MASS) * read_sensor(VS_XG) - read_sensor(VS_NVDOT);
    a[6][3] = 0.;
    a[6][4] = -read_sensor(VS_IZX);
    a[6][5] = -read_sensor(VS_IYZ);
    a[6][6] =  read_sensor(VS_IZZ) - read_sensor(VS_NRDOT);
  
    if(debug)
    {
	printf("Inertia matrix: ------------ \n") ;
	for(i=1;i<=6;i++){
	    for(j=1;j<=6;j++)
		printf("%6.2f  ", a[i][j]) ;
	    printf("\n");
	}
	printf("\n");
    }

    matinv(a,ainv,6,work);
}
                
/*----------------------------------------------------------------------*
   Subroutine motion() uses 4th-order Runge-Kutta method.  Variables are:
     
   edot(1)  edot(2)  edot(3)  edot(4)  edot(5)  edot(6)
     ]        ]        ]        ]        ]        ]
    udot     vdot     wdot     pdot     qdot     rdot

   e(1)      e(2)     e(3)     e(4)     e(5)     e(6)
     ]        ]        ]        ]        ]        ]
     u        v        w        p        q        r

   p(1)      p(2)     p(3)     p(4)     p(5)     p(6)
     ]        ]        ]        ]        ]        ]
     x        y        z       phi     theta     psi
 *-----------------------------------------------------------------------*/

void motion( double e[], double p[], double Amps, double Del13,
	     double Del24, double dt ) 
{
    double eo[7],pdot[7][5], edot[7][5];
    double po[7];
    double ds4,dc4,ds5,dc5,ds6,dc6,dt5,ds5dc6,ds5ds6;
    int i,j,k;
    double dto2, dto6;

    dto2  = 0.5*dt;
    dto6  = dt/6.0;

    for (i=1;i<=6;i++)
    {
	eo[i]=e[i];
	po[i]=p[i];
	for (j=1;j<=4;j++)
	{
	    edot[i][j]=0.0;
	    pdot[i][j]=0.0;
	}
    }
				/* start 4th order runga kutta integration */
    for (jjj=1;jjj<=4;jjj++)
    {				/* get forces and moments: rhs[1] to rhs[6] */
	hydro( e, p, Amps, Del13, Del24, dt );
				/* multiply by inverse of inertial matrix */
	for (i=1;i<=6;i++)
	    for (k=1;k<=6;k++)
		edot[i][jjj] = edot[i][jjj] +  ainv[i][k] * rhs[k];

				/* ctrn takes e to pdot */
	ds4 = sin(p[4]);
	dc4 = cos(p[4]);
	ds5 = sin(p[5]);
	dc5 = cos(p[5]);
	ds6 = sin(p[6]);
	dc6 = cos(p[6]);
	dt5 = tan(p[5]);
	ds5dc6 = ds5 * dc6;
	ds5ds6 = ds5 * ds6;
				/* this first 3x3 block is just 3-axis
				   rotation */
	ctrn[1][1] =  dc5 * dc6;
	ctrn[1][2] = -dc4 * ds6 + ds4 * ds5dc6;
	ctrn[1][3] =  ds4 * ds6 + dc4 * ds5dc6;

	ctrn[2][1] =  dc5 * ds6;
	ctrn[2][2] =  dc4 * dc6 + ds4 * ds5ds6;
	ctrn[2][3] = -ds4 * dc6 + dc4 * ds5ds6;

	ctrn[3][1] = -ds5;
	ctrn[3][2] =  dc5 * ds4;
	ctrn[3][3] =  dc5 * dc4;
				/* this 3x3 block relates body attitude
				   rates to Euler angle rates of change */
	ctrn[4][4] =  1.0;
	ctrn[4][5] =  ds4 * dt5;
	ctrn[4][6] =  dc4 * dt5;

	ctrn[5][4] =  0.0;
	ctrn[5][5] =  dc4;
	ctrn[5][6] = -ds4;

	ctrn[6][4] =  0.0;
	ctrn[6][5] =  ds4/dc5;
	ctrn[6][6] =  dc4/dc5;

	for (i=1;i<=6;i++) {	/* get pdot */
	    for (k=1;k<=6;k++) {
		pdot[i][jjj] = pdot[i][jjj] + ctrn[i][k] * e[k];
	    }
	}
				/* apply a Runge-Kutta half- or whole step */
	if ((jjj == 1) || (jjj == 2)) {
	    for (i=1;i<=6;i++) { 
		e[i] = eo[i] + edot[i][jjj] * dto2;
		p[i] = po[i] + pdot[i][jjj] * dto2;
	    }
	}
	else {
	    for (i=1;i<=6;i++) 
	    {
		e[i] = eo[i] + edot[i][jjj] * dt;
		p[i] = po[i] + pdot[i][jjj] * dt;
	    }
	}
    } /* for (jjj=1;jjj<=4;jjj++)  loop ends */


    for (i=1;i<=6;i++)		/* compute the weighted sum */ 
    {
	e[i] = eo[i] + (edot[i][1] + 2. * (edot[i][2] + edot[i][3]) + 
			edot[i][4]) * dto6;
	p[i] = po[i] + (pdot[i][1] + 2. * (pdot[i][2] + pdot[i][3]) + 
			pdot[i][4]) * dto6;
    }

				/* add in effects of steady current... */ 
    p[2] = p[2] + read_sensor(REAL_CURRENT_EAST) * dt;
    p[1] = p[1] + read_sensor(REAL_CURRENT_NORTH) * dt;
				/* update parameters for noisy sensor cases */
    update_sensor(REAL_U, e[1]);
    update_sensor(REAL_V, e[2]);
    update_sensor(REAL_W, e[3]);
    update_sensor(REAL_P, e[4]);
    update_sensor(REAL_Q, e[5]);
    update_sensor(REAL_R, e[6]);
  
    update_sensor(REAL_UD, (e[1] - eo[1]) / dt);
    update_sensor(REAL_VD, (e[2] - eo[2]) / dt);
    update_sensor(REAL_WD, (e[3] - eo[3]) / dt);
    update_sensor(REAL_PD, (e[4] - eo[4]) / dt);
    update_sensor(REAL_QD, (e[5] - eo[5]) / dt);
    update_sensor(REAL_RD, (e[6] - eo[6]) / dt);
  
    update_sensor(REAL_X, p[1]);
    update_sensor(REAL_Y, p[2]);
    update_sensor(REAL_Z, p[3]);
    update_sensor(REAL_PSI, p[6]);
    update_sensor(REAL_THETA, p[5]);
    update_sensor(REAL_PHI, p[4]);
}

/*-------------------------------------------------------------------------
  Subroutine hydro() gets the right-hand sides of the dynamic equations.
------------------------------------------------------------------------- */

void hydro( double *e, double *p, double Amps, double Del13, double
	    Del24, double dt )

{
  static double Xgp, Ygp, Zgp, Xbp, Ybp, Zbp, Wp, Bp;
  double pp,qq,rr,pr,pq,qr,uv,uw,uq,ur,vv,ww;
  double vp,vr,wp,wq,cp4,cp5,sp4,sp5;
  double wabw,vabv,uabu,pabp,qabq,rabr;
  double real_amps;
  double rud_ang, elev_ang, TForce, TTorque, TVolt;
  double vt , wt ;

  static double old_rud_ang = 0 ,       old_elev_ang = 0. ;
  static double old_rud_ang_shaft = 0. , old_elev_ang_shaft  = 0. ;
  double rud_ang_shaft, elev_ang_shaft ;
  static double rud_ang_com_delay[100], elev_ang_com_delay[100] ;
  int i ;

  static double old_TForce = 0.0, old_TTorque = 0.0, old_TVolt = 0.0;
  double Wdw1, Wdw2, W,B ;
  const int debug=0;
  
				/* if weight dropped, then vehicle feels 0 */
  if (read_sensor(C_WEIGHT1_DROP)==0.0) 
      Wdw1 = read_sensor(REAL_WDW1);
  else 
      Wdw1 = 0.0;
        
  if (read_sensor(C_WEIGHT2_DROP)==0.0) 
      Wdw2 = read_sensor(REAL_WDW2);
  else 
      Wdw2 = 0.0;
        
  W = read_sensor(REAL_VEH_W);
  B = read_sensor(REAL_VEH_B);
				/* get weight and buoyancy of vehicle */
  Wp = W + Wdw1 + Wdw2;
  Bp = B + read_sensor(VS_BDW1) + read_sensor(VS_BDW2);
  Xgp = (read_sensor(VS_XG)*W + read_sensor(VS_XDW1)*Wdw1 + read_sensor(VS_XDW2)*Wdw2) / Wp;
  Ygp = (read_sensor(VS_YG)*W + read_sensor(VS_YDW1)*Wdw1 + read_sensor(VS_YDW2)*Wdw2) / Wp;
  Zgp = (read_sensor(VS_ZG)*W + read_sensor(VS_ZDW1)*Wdw1 + read_sensor(VS_ZDW2)*Wdw2) / Wp;
  Xbp = (read_sensor(VS_XB)*W + read_sensor(VS_XDW1)*read_sensor(VS_BDW1) + read_sensor(VS_XDW2)*read_sensor(VS_BDW2)) / Bp;
  Ybp = (read_sensor(VS_YB)*W + read_sensor(VS_YDW1)*read_sensor(VS_BDW1) + read_sensor(VS_YDW2)*read_sensor(VS_BDW2)) / Bp;
  Zbp = (read_sensor(VS_ZB)*W + read_sensor(VS_ZDW1)*read_sensor(VS_BDW1) + read_sensor(VS_ZDW2)*read_sensor(VS_BDW2)) / Bp;

  update_sensor(REAL_EXCESS_BUOYANCY, Bp-Wp);

  vv = e[2] * e[2];
  ww = e[3] * e[3];
  uv = e[1] * e[2];
  uw = e[1] * e[3];
  ur = e[1] * e[6];
  uq = e[1] * e[5];
  vr = e[2] * e[6];
  vp = e[2] * e[4];
  wp = e[3] * e[4];
  wq = e[3] * e[5];
  pr = e[4] * e[6];
  pq = e[4] * e[5];
  qr = e[5] * e[6];
  pp = e[4] * e[4];
  qq = e[5] * e[5];
  rr = e[6] * e[6];
  uabu = e[1] * fabs(e[1]);
  vabv = e[2] * fabs(e[2]);
  wabw = e[3] * fabs(e[3]);
  pabp = e[4] * fabs(e[4]);
  qabq = e[5] * fabs(e[5]);
  rabr = e[6] * fabs(e[6]);
  cp4 = cos(p[4]);  
  cp5 = cos(p[5]);
  sp4 = sin(p[4]);
  sp5 = sin(p[5]);       

  vt = e[2] + read_sensor(VS_FIN1_X)*e[6] ;
  wt = e[3] - read_sensor(VS_FIN1_X)*e[5] ;

  if (read_sensor(M_PRESENT_TIME) <= .2) {
    old_rud_ang = -read_sensor(VS_RUD_HYST_CENTER) ;
    old_elev_ang = -read_sensor(VS_ELEV_HYST_CENTER) ;
  }

  if (jjj == 1)          /* if first step of the runge-kutta integration,...  */
  {

                         /* two big time steps delay */
    int act_delay = (int)read_sensor(VS_ACT_DELAY);
    for(i=act_delay-1;i>=0;i--){
      rud_ang_com_delay[i+1]  = rud_ang_com_delay[i] ;
      elev_ang_com_delay[i+1] = elev_ang_com_delay[i] ;
    }

    rud_ang_com_delay[0] = Del13;
    elev_ang_com_delay[0] = Del24;

				/* enforce speed constraint on motor shaft */
    rud_ang_shaft = act_model( rud_ang_com_delay[act_delay], 
			       old_rud_ang_shaft, dt );
				/* put through a hysteresis loop to get
                                   actual rudder */
    rud_ang = hysteresis( rud_ang_shaft, old_rud_ang_shaft,  
			  old_rud_ang, read_sensor(VS_RUD_HYST_WIDTH), read_sensor(VS_RUD_HYST_CENTER) ) ;

    elev_ang_shaft = act_model( elev_ang_com_delay[act_delay],
				 old_elev_ang_shaft, dt );
    elev_ang = hysteresis( elev_ang_shaft, old_elev_ang_shaft,
			   old_elev_ang, read_sensor(VS_ELEV_HYST_WIDTH), read_sensor(VS_ELEV_HYST_CENTER)) ;

    update_sensor(M_ELEVATOR_ANGLE,elev_ang) ;
    update_sensor(M_RUDDER_ANGLE,rud_ang) ;

    real_amps = Amps  ;

    TForce = thruster_force(real_amps,e[1]);    /* using u here should be a little*/
    TTorque = thruster_torque(real_amps,e[1]);  /* more accurate than ut */

                             /* store the previous fin angles and thruster parameters */
    old_rud_ang = rud_ang ;
    old_elev_ang = elev_ang ;
    old_rud_ang_shaft = rud_ang_shaft ;
    old_elev_ang_shaft = elev_ang_shaft ;
    
    old_TForce = TForce;
    old_TTorque = TTorque;
    old_TVolt = TVolt;
  }
  else
  {
    rud_ang = old_rud_ang ;
    elev_ang = old_elev_ang ;
    TForce = old_TForce;
    TTorque = old_TTorque;
    TVolt = old_TVolt;
  }
  
  fins(e,rud_ang,elev_ang);            /* get fin forces and moments 
                                   F1,F2,F3,F4,M1,M2,M3,M4 */
  
  rhs[1] = TForce                   /* AXIAL FORCES */
         - (Wp-Bp)*sp5 
         + read_sensor(VS_MASS)*(vr - wq + Xgp*(qq + rr) - Ygp*pq - Zgp*pr) 
         + F1[1] + F2[1] + F3[1] + F4[1]
         + read_sensor(VS_XVV)*vv + read_sensor(VS_XWW)*ww + read_sensor(VS_XVR)*vr + read_sensor(VS_XWQ)*wq + read_sensor(VS_XRR)*rr + read_sensor(VS_XQQ)*qq;

                          /* apply correction in operating speed range, fsh */
  if (e[1] > 0.7)
         rhs[1] += read_sensor(VS_XUABU)*uabu*(2.455-2.175*e[1]+.75*e[1]*e[1]) ;
  else
         rhs[1] += read_sensor(VS_XUABU)*uabu*(2.455-2.175*.7+.75*.7*.7) ;

  if ((debug)&&(jjj == 1)) {
      printf("%5.2e %5.2e %5.2e %5.2e %5.2e %5.2e %5.2e %5.2e %5.2e %5.2e %5.2e %5.2e ",
             read_sensor(M_PRESENT_TIME),rud_ang,
             TForce,
             read_sensor(VS_MASS)*(vr - wq + Xgp*(qq + rr) - Ygp*pq - Zgp*pr),
             F1[1] + F2[1] + F3[1] + F4[1],
             read_sensor(VS_XVV)*vv , read_sensor(VS_XWW)*ww , read_sensor(VS_XVR)*vr ,read_sensor(VS_XWQ)*wq , read_sensor(VS_XRR)*rr , read_sensor(VS_XQQ)*qq,
             read_sensor(VS_XUABU)*uabu*(2.455-2.175*e[1]+.75*e[1]*e[1])) ;
  }
  
  rhs[2] = (Wp-Bp)*cp5*sp4         /* LATERAL FORCES */
         + read_sensor(VS_YVABV)*vabv
         + read_sensor(VS_YUV)*uv + read_sensor(VS_YUR)*ur + read_sensor(VS_YRABR)*rabr + read_sensor(VS_YWP)*wp 
         + read_sensor(VS_MASS)*(wp - ur + Ygp*(rr+pp) -Zgp*qr - Xgp*pq) 
         + F1[2] + F2[2] + F3[2] + F4[2];

  if ((debug)&&(jjj == 1)) {
          printf("%5.2e %5.2e %5.2e %5.2e %5.2e %5.2e %5.2e ",
                 read_sensor(VS_YVABV)*vabv,
                 read_sensor(VS_YUV)*uv , read_sensor(VS_YUR)*ur , read_sensor(VS_YRABR)*rabr , read_sensor(VS_YWP)*wp, 
                 read_sensor(VS_MASS)*(wp - ur + Ygp*(rr+pp) -Zgp*qr - Xgp*pq),
                 F1[2] + F2[2] + F3[2] + F4[2]);
  }
  
  rhs[3] = (Wp-Bp)*cp5*cp4   /* NORMAL FORCES */
         + read_sensor(VS_ZWABW)*wabw
         + read_sensor(VS_ZUQ)*uq + read_sensor(VS_ZUW)*uw + read_sensor(VS_ZQABQ)*qabq + read_sensor(VS_ZVP)*vp
           + read_sensor(VS_MASS)*(uq - vp + Zgp*(pp + qq) - Xgp*pr - Ygp*qr)
         + F1[3] + F2[3] + F3[3] + F4[3];
  
  rhs[4] = -(Zgp*Wp - Zbp*Bp)*cp5*sp4    /* ROLL MOMENTS */
           + (Ygp*Wp - Ybp*Bp)*cp5*cp4   
         + read_sensor(VS_KPABP)*pabp 
         - (read_sensor(VS_IZZ) - read_sensor(VS_IYY))*qr 
         + read_sensor(VS_IZX)*pq - read_sensor(VS_IXY)*pr + read_sensor(VS_IYZ)*qq -read_sensor(VS_IYZ)*rr 
         - read_sensor(VS_MASS)*(Ygp*(vp-uq) + Zgp*(wp-ur))
         + M1[1] + M2[1] + M3[1] + M4[1] 
         + TTorque + read_sensor(VS_KVT2)*4.*vt*wt*(wt*wt-vt*vt)/(vt*vt + wt*wt + .0001) ;
  
  rhs[5] = -(Zgp*Wp - Zbp*Bp)*sp5        /* PITCH MOMENTS */
           - (Xgp*Wp - Xbp*Bp)*cp5*cp4
         + read_sensor(VS_MWABW)*wabw
         + read_sensor(VS_MQABQ)*qabq
         + read_sensor(VS_MUW)*uw  
         + read_sensor(VS_MUQ)*uq + read_sensor(VS_MPR)*pr
           + (read_sensor(VS_IZZ) - read_sensor(VS_IXX))*pr
         - read_sensor(VS_IZX)*pp - read_sensor(VS_IYZ)*pq +read_sensor(VS_IXY)*qr + read_sensor(VS_IZX)*rr
         - read_sensor(VS_MASS)*(Zgp*(wq - vr) + Xgp*(uq - vp))                     
         + M1[2] + M2[2] + M3[2] + M4[2];

  rhs[6] = (Ygp*Wp - Ybp*Bp)*sp5          /* YAW MOMENTS */ 
           + (Xgp*Wp - Xbp*Bp)*cp5*sp4   
         + read_sensor(VS_NUV)*uv + read_sensor(VS_NRABR)*rabr 
         + read_sensor(VS_NUR)*ur + read_sensor(VS_NVABV)*vabv 
         + read_sensor(VS_NPQ)*pq
         + (read_sensor(VS_IXX) - read_sensor(VS_IYY))*pq
         + read_sensor(VS_IXY)*pp + read_sensor(VS_IYZ)*pr - read_sensor(VS_IXY)*qq - read_sensor(VS_IZX)*qr 
         - read_sensor(VS_MASS)*Xgp*(ur-wp) + read_sensor(VS_MASS)*Ygp*(wq - vr)
               + M1[3] + M2[3] + M3[3] + M4[3];

  if ((debug)&&(jjj == 1)) {
          printf("%5.2e %5.2e %5.2e %5.2e %5.2e %5.2e %5.2e\n",
                 read_sensor(VS_NUV)*uv,
                 read_sensor(VS_NRABR)*rabr, 
                 read_sensor(VS_NUR)*ur,
                 read_sensor(VS_NVABV)*vabv,
                 read_sensor(VS_NPQ)*pq,
                 (read_sensor(VS_IXX) - read_sensor(VS_IYY))*pq + read_sensor(VS_IXY)*pp + read_sensor(VS_IYZ)*pr - read_sensor(VS_IXY)*qq - read_sensor(VS_IZX)*qr 
                 - read_sensor(VS_MASS)*Xgp*(ur-wp) + read_sensor(VS_MASS)*Ygp*(wq - vr),  
                 M1[3] + M2[3] + M3[3] + M4[3]);
  }
  
  update_sensor(REAL_FX, rhs[1]);
  update_sensor(REAL_FY, rhs[2]);
  update_sensor(REAL_FZ, rhs[3]);
  update_sensor(REAL_FK, rhs[4]);
  update_sensor(REAL_FM, rhs[5]);
  update_sensor(REAL_FN, rhs[6]);

}

/* ------------------------------------------------------------------
   Subroutine fins() computes forces generated by control surfaces
   ------------------------------------------------------------------ */

void fins( double *e, double rud_ang, double elev_ang )
{
  double r1[4] = {0., read_sensor(VS_FIN1_X), read_sensor(VS_FIN1_Y), read_sensor(VS_FIN1_Z)};
  double r2[4] = {0., read_sensor(VS_FIN2_X), read_sensor(VS_FIN2_Y), read_sensor(VS_FIN2_Z)};
  double r3[4] = {0., read_sensor(VS_FIN3_X), read_sensor(VS_FIN3_Y), read_sensor(VS_FIN3_Z)};
  double r4[4] = {0., read_sensor(VS_FIN4_X), read_sensor(VS_FIN4_Y), read_sensor(VS_FIN4_Z)};
  static double norm_v1, norm_v2, norm_v3, norm_v4;
  static double alpha1,alpha2,alpha3,alpha4;
  static double CL1,CL2,CL3,CL4,CD1,CD2,CD3,CD4;
  static double cons,CDC,aa;
  static double LW1,LW2,LW3,LW4,DW1,DW2,DW3,DW4;
  static double LF1,LF2,LF3,LF4,DF1,DF2,DF3,DF4;
  static double v1[4],v2[4],v3[4],v4[4];
  static double s13,s24,c13,c24;

  s13 = sin(rud_ang);
  s24 = sin(elev_ang);
  c13 = cos(rud_ang);
  c24 = cos(elev_ang);
    
  v1[1] = e[1] + e[5]*r1[3] - e[6]*r1[2];        /* fin velocities in body coords.  */
  v1[2] = e[2] - e[4]*r1[3] + e[6]*r1[1];
  v1[3] = e[3] + e[4]*r1[2] - e[5]*r1[1];
  
  v2[1] = e[1] + e[5]*r2[3] - e[6]*r2[2];
  v2[2] = e[2] - e[4]*r2[3] + e[6]*r2[1];
  v2[3] = e[3] + e[4]*r2[2] - e[5]*r2[1];
  
  v3[1] = e[1] + e[5]*r3[3] - e[6]*r3[2];
  v3[2] = e[2] - e[4]*r3[3] + e[6]*r3[1];
  v3[3] = e[3] + e[4]*r3[2] - e[5]*r3[1];

  v4[1] = e[1] + e[5]*r4[3] - e[6]*r4[2];
  v4[2] = e[2] - e[4]*r4[3] + e[6]*r4[1];
  v4[3] = e[3] + e[4]*r4[2] - e[5]*r4[1];

                                                 /* now get angle of attack for each fin */
  norm_v1 = sqrt( v1[1]*v1[1] + v1[2]*v1[2] + v1[3]*v1[3] );
  if (norm_v1 < 0.001)
    alpha1 = 0.0;
  else
    alpha1 = asin(-v1[2]/norm_v1) + rud_ang;

  norm_v2 = sqrt( v2[1]*v2[1] + v2[2]*v2[2] + v2[3]*v2[3] );
  if (norm_v2 < 0.001)
    alpha2 = 0.0;
  else
    alpha2 = atan2(v2[3],v2[1]) + elev_ang;
  
  norm_v3 = sqrt( v3[1]*v3[1] + v3[2]*v3[2] + v3[3]*v3[3] );
  if (norm_v3 < 0.001)
    alpha3=0.0;
  else
    alpha3 = asin(-v3[2]/norm_v3) + rud_ang;
  
  norm_v4 = sqrt( v4[1]*v4[1] + v4[2]*v4[2] + v4[3]*v4[3] );
  if (norm_v4 < 0.001)
    alpha4=0.0;
  else
    alpha4 = atan2(v4[3],v4[1]) + elev_ang;
  
                                  /* lift and drag coefficients */
  CDC = read_sensor(VS_FIN_CD_CROSS)/read_sensor(VS_FIN_ASPECT_RATIO);

                                  /* note that if stall angle is exceeded: NO LIFT */
  if (fabs(alpha1) < read_sensor(VS_FIN_STALL_ANGLE))
    CL1 = read_sensor(VS_FIN_DCL)*alpha1 + CDC*alpha1*fabs(alpha1);
  else CL1 = 0. ;
  if (fabs(alpha2) < read_sensor(VS_FIN_STALL_ANGLE))
    CL2 = read_sensor(VS_FIN_DCL)*alpha2 + CDC*alpha2*fabs(alpha2);
  else CL2 = 0. ;
  if (fabs(alpha3) < read_sensor(VS_FIN_STALL_ANGLE))
    CL3 = read_sensor(VS_FIN_DCL)*alpha3 + CDC*alpha3*fabs(alpha3);
  else CL3 = 0. ;
  if (fabs(alpha4) < read_sensor(VS_FIN_STALL_ANGLE))
    CL4 = read_sensor(VS_FIN_DCL)*alpha4 + CDC*alpha4*fabs(alpha4);
  else CL4 = 0. ;

  aa = 1.0/(PI*read_sensor(VS_FIN_ASPECT_RATIO)*read_sensor(VS_FIN_OSWALD));
  
  CD1 = read_sensor(VS_FIN_CDO) + aa*CL1*CL1;
  CD2 = read_sensor(VS_FIN_CDO) + aa*CL2*CL2;
  CD3 = read_sensor(VS_FIN_CDO) + aa*CL3*CL3;
  CD4 = read_sensor(VS_FIN_CDO) + aa*CL4*CL4;

                                        /* lift and drag forces, in flow coords... */  
  cons = (rho*read_sensor(VS_FIN_AREA))/2.0;
  
  LW1 = cons*norm_v1*norm_v1*CL1;      /* positive when the lift vector is close to normal vector */
  LW2 = cons*norm_v2*norm_v2*CL2;
  LW3 = cons*norm_v3*norm_v3*CL3;
  LW4 = cons*norm_v4*norm_v4*CL4;
  
  DW1 = cons*norm_v1*norm_v1*CD1;      /* always positive */
  DW2 = cons*norm_v2*norm_v2*CD2;
  DW3 = cons*norm_v3*norm_v3*CD3;
  DW4 = cons*norm_v4*norm_v4*CD4;
  
  LF1 = LW1*cos(alpha1) + DW1*sin(alpha1);        /* force in the fin normal direction */
  LF2 = LW2*cos(alpha2) + DW2*sin(alpha2);
  LF3 = LW3*cos(alpha3) + DW3*sin(alpha3);
  LF4 = LW4*cos(alpha4) + DW4*sin(alpha4);
  
  DF1 = -LW1*sin(alpha1) + DW1*cos(alpha1);        /* force in the fin-aft direction */
  DF2 = -LW2*sin(alpha2) + DW2*cos(alpha2);        
  DF3 = -LW3*sin(alpha3) + DW3*cos(alpha3);
  DF4 = -LW4*sin(alpha4) + DW4*cos(alpha4);
  
  F1[1] = -LF1*s13 + (-DF1)*c13;        /* finally, transform into the body frame */
  F1[2] =  LF1*c13 + (-DF1)*s13;
  F1[3] = 0.0;
  
  F2[1] = -LF2*s24 + (-DF2)*c24;
  F2[2] = 0.0;
  F2[3] = -LF2*c24 - (-DF2)*s24;
  
  F3[1] = -LF3*s13 + (-DF3)*c13;
  F3[2] =  LF3*c13 + (-DF3)*s13;
  F3[3] = 0.0;
  
  F4[1] = -LF4*s24 + (-DF4)*c24;
  F4[2] = 0.0;
  F4[3] = -LF4*c24 - (-DF4)*s24;
  
  M1[1] = r1[2]*F1[3] - r1[3]*F1[2];                /* moments induced by these forces */
  M1[2] = -r1[1]*F1[3] + r1[3]*F1[1];
  M1[3] = r1[1]*F1[2] - r1[2]*F1[1];
  
  M2[1] = r2[2]*F2[3] - r2[3]*F2[2];
  M2[2] = -r2[1]*F2[3] + r2[3]*F2[1];
  M2[3] = r2[1]*F2[2] - r2[2]*F2[1];
  
  M3[1] = r3[2]*F3[3] - r3[3]*F3[2];
  M3[2] = -r3[1]*F3[3] + r3[3]*F3[1];
  M3[3] = r3[1]*F3[2] - r3[2]*F3[1];
  
  M4[1] = r4[2]*F4[3] - r4[3]*F4[2];
  M4[2] = -r4[1]*F4[3] + r4[3]*F4[1];
  M4[3] = r4[1]*F4[2] - r4[2]*F4[1];

  return;
}

/*--------------------- routines from actuator.c -----------------------*
* act_model() :  move at either zero, full forward, or full backward speed.
* thruster_force() : computation of the thruster force from the Amps
* thruster_torque() : computation of the thruster torque from the Amps
* ----------------------------------------------------------------------*/

double act_model(double des_angle, double angle, double dt)
{
  double new_angle = 0.0;
  double can_move_in_dt ;
  const int debug=0;      

  can_move_in_dt = read_sensor(VS_FIN_SPEED)*dt;        
                                 /* how far fin can go in dt seconds */

  if (fabs(angle-des_angle) <= can_move_in_dt) {
    new_angle = des_angle;
  }
  else {
    if (angle > des_angle)
      new_angle = angle - can_move_in_dt;
    if (angle < des_angle)
      new_angle = angle + can_move_in_dt;
  }

  if (debug) 
      printf("canmove in dt=%lf  %lf  %lf  %lf\n", can_move_in_dt,des_angle,angle,new_angle); 

  return(new_angle);
}

/*----------------------------------------------------------------------
  Hysteresis: applied to an arbitrary input, probably elevator or
  rudder.  Width and center of loop are defined with other physical
  variables.

  fsh 8 May 1995
------------------------------------------------------------------------*/

double hysteresis( double x, double x_last, double y_last, double wide, double center )
{
  double right_intercept, left_intercept, to_right, to_left ;
  double dx, y = 0.0;

  right_intercept = center + wide/2. ;
  left_intercept = center - wide/2. ;

  dx = x - x_last ;
                             /* get last horizontal distance from right and left lines*/
  to_right = -x_last + (y_last + right_intercept) ; 
  to_left = -x_last + (y_last + left_intercept) ;


  if (dx == 0.) y = y_last ;

/* CASE 1:  a move inside the loop -------------------------------*/

  if (dx <= to_right && dx > 0.) y = y_last ;
  if (dx >= to_left  && dx < 0.) y = y_last ;

/* CASE 2:  a move that goes outside the loop ---------------------*/

  if (dx > to_right && dx > 0.)
    y = y_last + x - (y_last + right_intercept) ;
  if (dx < to_left && dx < 0.)
    y = y_last + x - (y_last + left_intercept) ;

  return(y) ;
}


/*=====================================================================
  Modified this to reflect nature of Odyssey II's current control
  on the thruster controller...  Calculate torque (Kt*amps) then
  calculate propellor RPM for that torque and water speed.  This in
  turn allows one to compute thrust....
  J. G. Bellingham        1/94
  fsh update     5/95    
 *====================================================================*/
        
struct thruster_state {
    double amps;
    double inflow;
    double prop_omega;
    double motor_omega;
    double thrust;
    double vehicle_torque;
    double motor_voltage;
    double elec_power;
    double drivetrain_torque;
};

static struct thruster_state last_thruster_state;
static int last_thruster_state_valid = 0;
static double battery_energy_used = 0.0;
static double battery_charge_used = 0.0;
static double battery_energy_capacity = 0.0;
static double battery_charge_capacity = 0.0;
static double battery_initial_soc = 1.0;
static double battery_terminal_voltage = 48.0;
static double battery_v_rc = 0.0;
static double battery_temp = 296.15;
static int battery_bms_tripped = 0;
static int battery_initialized = 0;
static double last_battery_time = -1.0;

static double clip_abs( double x, double limit )
{
    if (limit <= 0.0) return x;
    if (fabs(x) > limit) {
        if (x > 0.0) return limit;
        return -limit;
    }
    return x;
}

static void compute_thruster_state( double real_amps, double w_speed,
                                    struct thruster_state *ts )
{
    double current_limit, voltage_limit;
    double amps, inflow, area, eta, pitch, kt_fit;
    double numerator, denominator;
    double gear_ratio, motor_ke, motor_r, motor_kt, gear_eff, seal_friction;
    double kt_scale, kq_scale, eta_scale, area_scale, wake_fraction;

    current_limit = read_sensor(VS_THRUSTER_CURRENT_LIMIT);
    voltage_limit = read_sensor(VS_THRUSTER_VOLTAGE_LIMIT);

    amps = clip_abs(real_amps, current_limit);

    wake_fraction = read_sensor(VS_THRUSTER_WAKE_FRACTION);
    if (wake_fraction <= 0.0) wake_fraction = 1.0;
    inflow = wake_fraction * w_speed;

    kt_scale = read_sensor(VS_THRUSTER_KT_SCALE);
    kq_scale = read_sensor(VS_THRUSTER_KQ_SCALE);
    eta_scale = read_sensor(VS_THRUSTER_ETA_SCALE);
    area_scale = read_sensor(VS_THRUSTER_AREA_SCALE);

    if (kt_scale < 0.0) kt_scale = 0.0;
    if (kq_scale < 0.0) kq_scale = 0.0;
    if (eta_scale <= 0.0) eta_scale = 1.0;
    if (area_scale <= 0.0) area_scale = 1.0;

    /*
       Legacy effective calibration model. The effective area is left as
       vs_thruster_duct_area for backwards compatibility. The physical
       propeller diameter is stored separately as vs_thruster_prop_diameter
       for later Kt(J)/Kq(J) mapping and documentation.
    */
    kt_fit = read_sensor(VS_THRUSTER_KT) * kt_scale;
    eta = read_sensor(VS_THRUSTER_ETA) * eta_scale;
    pitch = read_sensor(VS_THRUSTER_PITCH);
    area = read_sensor(VS_THRUSTER_DUCT_AREA) * area_scale;

    numerator = kt_fit*amps + .5*area*eta*pitch*rho*inflow*inflow;
    denominator = .5*area*eta*eta*eta*pitch*pitch*pitch*rho;

    if (denominator > 0.0 && numerator > 0.0)
        ts->prop_omega = sqrt(numerator/denominator);
    else
        ts->prop_omega = 0.0;

    ts->thrust = rho * area *
        (pow(eta*pitch*ts->prop_omega,2) - inflow*eta*pitch*ts->prop_omega);

    ts->vehicle_torque = read_sensor(VS_THRUSTER_KT) * amps * kq_scale;

    gear_ratio = read_sensor(VS_THRUSTER_GEAR_RATIO);
    motor_ke = read_sensor(VS_THRUSTER_MOTOR_KE);
    motor_r = read_sensor(VS_THRUSTER_MOTOR_R);
    motor_kt = read_sensor(VS_THRUSTER_MOTOR_KT);
    gear_eff = read_sensor(VS_THRUSTER_GEAR_EFF);
    seal_friction = read_sensor(VS_THRUSTER_SEAL_FRICTION);

    if (gear_ratio <= 0.0) gear_ratio = 1.0;
    if (gear_eff <= 0.0) gear_eff = 1.0;

    ts->motor_omega = gear_ratio * ts->prop_omega;
    ts->motor_voltage = motor_r*amps + motor_ke*ts->motor_omega;

    /* Voltage saturation is a fault/limit hook. With the default high limit,
       this has no effect and the legacy calibration is preserved. */
    if (voltage_limit > 0.0 && fabs(ts->motor_voltage) > voltage_limit) {
        amps = amps * voltage_limit / fabs(ts->motor_voltage);
        numerator = kt_fit*amps + .5*area*eta*pitch*rho*inflow*inflow;
        if (denominator > 0.0 && numerator > 0.0)
            ts->prop_omega = sqrt(numerator/denominator);
        else
            ts->prop_omega = 0.0;
        ts->thrust = rho * area *
            (pow(eta*pitch*ts->prop_omega,2) - inflow*eta*pitch*ts->prop_omega);
        ts->vehicle_torque = read_sensor(VS_THRUSTER_KT) * amps * kq_scale;
        ts->motor_omega = gear_ratio * ts->prop_omega;
        ts->motor_voltage = motor_r*amps + motor_ke*ts->motor_omega;
    }

    ts->elec_power = ts->motor_voltage * amps;
    ts->drivetrain_torque = gear_eff*gear_ratio*motor_kt*amps;
    if (ts->prop_omega > 0.0)
        ts->drivetrain_torque -= seal_friction;
    else if (ts->prop_omega < 0.0)
        ts->drivetrain_torque += seal_friction;

    ts->amps = amps;
    ts->inflow = inflow;
}

static double interp_table( double x, const double *xs, const double *ys, int n )
{
    int i;

    if (n <= 0) return 0.0;
    if (x <= xs[0]) return ys[0];
    if (x >= xs[n-1]) return ys[n-1];

    for (i = 0; i < n-1; i++) {
        if (x >= xs[i] && x <= xs[i+1]) {
            if (xs[i+1] == xs[i]) return ys[i];
            return ys[i] + (ys[i+1] - ys[i]) *
                (x - xs[i]) / (xs[i+1] - xs[i]);
        }
    }
    return ys[n-1];
}

static double battery_capacity_temp_scale( int chem, double temp_k )
{
    double temp_c;

    temp_c = temp_k - 273.15;
    if (chem == 2) {
        if (temp_c <= -20.0) return 0.55;
        if (temp_c < 25.0) return 0.55 + 0.45*(temp_c + 20.0)/45.0;
        return 1.0;
    }

    if (temp_c <= -2.0) return 0.70;
    if (temp_c < 23.0) return 0.70 + 0.30*(temp_c + 2.0)/25.0;
    return 1.0;
}

static double battery_cell_ocv( int chem, double soc, double temp_k,
                                double age )
{
    static const double ag_soc[] = {
        0.00, 0.05, 0.10, 0.15, 0.25, 0.40, 0.60, 0.80, 0.90, 0.95, 1.00
    };
    static const double ag_ocv[] = {
        0.80, 1.10, 1.35, 1.44, 1.48, 1.50, 1.51, 1.52, 1.53, 1.56, 1.62
    };
    static const double li_soc[] = {
        0.00, 0.05, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 0.95, 1.00
    };
    static const double li_ocv[] = {
        2.80, 3.20, 3.45, 3.62, 3.70, 3.74, 3.78, 3.82, 3.88, 3.96, 4.05, 4.12, 4.20
    };
    double ocv, temp_c, temp_ref_c, temp_coeff, age_coeff;

    if (soc < 0.0) soc = 0.0;
    if (soc > 1.0) soc = 1.0;
    if (age < 0.0) age = 0.0;

    temp_c = temp_k - 273.15;
    if (chem == 2) {
        ocv = interp_table(soc, li_soc, li_ocv,
                           sizeof(li_soc)/sizeof(li_soc[0]));
        temp_ref_c = 25.0;
        temp_coeff = -0.0004;
        age_coeff = 0.04;
    }
    else {
        ocv = interp_table(soc, ag_soc, ag_ocv,
                           sizeof(ag_soc)/sizeof(ag_soc[0]));
        temp_ref_c = 23.0;
        temp_coeff = -0.0008;
        age_coeff = 0.08;
    }

    ocv += temp_coeff * (temp_ref_c - temp_c);
    ocv -= age_coeff * age;
    if (ocv < 0.0) ocv = 0.0;
    return ocv;
}

static double battery_soc_res_scale( int chem, double soc )
{
    static const double ag_soc[] = {
        0.05, 0.10, 0.20, 0.40, 0.60, 0.80, 1.00
    };
    static const double ag_scale[] = {
        6.0, 3.0, 1.5, 1.1, 1.0, 1.0, 1.1
    };
    static const double li_soc[] = {
        0.05, 0.10, 0.20, 0.40, 0.60, 0.80, 1.00
    };
    static const double li_scale[] = {
        4.0, 2.0, 1.4, 1.05, 1.0, 1.0, 1.2
    };

    if (chem == 2)
        return interp_table(soc, li_soc, li_scale,
                            sizeof(li_soc)/sizeof(li_soc[0]));
    return interp_table(soc, ag_soc, ag_scale,
                        sizeof(ag_soc)/sizeof(ag_soc[0]));
}

static double battery_temp_res_scale( int chem, double temp_k )
{
    static const double ag_temp[] = {271.15, 296.15, 308.15};
    static const double ag_scale[] = {2.5, 1.0, 0.8};
    static const double li_temp[] = {253.15, 273.15, 298.15, 318.15};
    static const double li_scale[] = {6.0, 2.0, 1.0, 0.8};

    if (chem == 2)
        return interp_table(temp_k, li_temp, li_scale,
                            sizeof(li_temp)/sizeof(li_temp[0]));
    return interp_table(temp_k, ag_temp, ag_scale,
                        sizeof(ag_temp)/sizeof(ag_temp[0]));
}

static void update_battery_load( struct thruster_state *ts )
{
    double hotel_load, battery_voltage, battery_power, battery_current;
    double present_time, dt, charge_remaining, energy_remaining, soc;
    double ocv, resistance, r0_cell, r1, c1, tau, target_v_rc;
    double series_cells, parallel_strings, age, cap_scale, r_scale;
    double therm_mass, therm_res, heat, seawater_k, weak_v, temp_scale;
    double est_cap, est_soc, measured_current, measured_voltage;
    double bms_vtrip, bms_itrip, bms_ttrip, max_current, min_voltage;
    int chem;

    hotel_load = read_sensor(VS_HOTEL_LOAD);
    if (hotel_load < 0.0) hotel_load = 0.0;

    battery_voltage = read_sensor(VS_BATTERY_NOMINAL_VOLTAGE);
    if (battery_voltage <= 0.0) battery_voltage = 48.0;

    if (!battery_initialized) {
        chem = (int) (read_sensor(VS_BATT_CHEM) + 0.5);
        if (chem != 2) chem = 1;

        cap_scale = read_sensor(VS_BATT_CAP_SCALE);
        if (cap_scale < 0.0) cap_scale = 0.0;

        battery_temp = read_sensor(WATER_TEMP) + 273.15;
        temp_scale = battery_capacity_temp_scale(chem, battery_temp);
        battery_charge_capacity = read_sensor(VS_BATT_CAPACITY) *
            cap_scale * temp_scale;
        battery_energy_capacity = read_sensor(VS_BATT_ENERGY_CAP) *
            cap_scale * temp_scale;
        battery_initial_soc = read_sensor(VS_BATT_INITIAL_SOC);

        if (battery_initial_soc < 0.0) battery_initial_soc = 0.0;
        if (battery_initial_soc > 1.0) battery_initial_soc = 1.0;

        if (battery_charge_capacity <= 0.0 && battery_energy_capacity > 0.0)
            battery_charge_capacity = battery_energy_capacity / battery_voltage;
        if (battery_energy_capacity <= 0.0 && battery_charge_capacity > 0.0)
            battery_energy_capacity = battery_charge_capacity * battery_voltage;

        battery_charge_used = (1.0 - battery_initial_soc) * battery_charge_capacity;
        battery_energy_used = (1.0 - battery_initial_soc) * battery_energy_capacity;
        battery_terminal_voltage = battery_voltage;
        battery_v_rc = 0.0;
        battery_bms_tripped = 0;
        battery_initialized = 1;
    }

    battery_power = fabs(ts->elec_power) + hotel_load;
    if (battery_terminal_voltage > 1.0)
        battery_current = battery_power / battery_terminal_voltage;
    else
        battery_current = battery_power / battery_voltage;
    battery_current += read_sensor(VS_BATT_PARASITIC);
    if (battery_current < 0.0) battery_current = 0.0;

    present_time = read_sensor(M_PRESENT_TIME);
    if (last_battery_time >= 0.0 && present_time >= last_battery_time)
        dt = present_time - last_battery_time;
    else
        dt = 0.0;

    charge_remaining = battery_charge_capacity - battery_charge_used;
    energy_remaining = battery_energy_capacity - battery_energy_used;
    if (charge_remaining < 0.0) charge_remaining = 0.0;
    if (energy_remaining < 0.0) energy_remaining = 0.0;

    if (battery_charge_capacity > 0.0)
        soc = charge_remaining / battery_charge_capacity;
    else if (battery_energy_capacity > 0.0)
        soc = energy_remaining / battery_energy_capacity;
    else
        soc = 0.0;

    chem = (int) (read_sensor(VS_BATT_CHEM) + 0.5);
    if (chem != 2) chem = 1;

    series_cells = read_sensor(VS_BATT_SERIES_CELLS);
    parallel_strings = read_sensor(VS_BATT_PARALLEL_STRS);
    if (series_cells <= 0.0) series_cells = 1.0;
    if (parallel_strings <= 0.0) parallel_strings = 1.0;

    age = read_sensor(VS_BATT_AGE);
    if (age < 0.0) age = 0.0;

    r0_cell = read_sensor(VS_BATT_R0_CELL);
    r_scale = read_sensor(VS_BATT_R_SCALE);
    if (r_scale <= 0.0) r_scale = 1.0;

    resistance = r0_cell * series_cells / parallel_strings *
        battery_soc_res_scale(chem, soc) *
        battery_temp_res_scale(chem, battery_temp) *
        (1.0 + age) * r_scale;
    resistance += read_sensor(VS_BATT_R_WIRING) + read_sensor(VS_BATT_R_CONN);
    resistance += fabs(read_sensor(VS_BATT_IMBALANCE)) * r0_cell * series_cells;
    if (resistance < 0.0) resistance = 0.0;

    r1 = read_sensor(VS_BATT_R1) * r_scale;
    c1 = read_sensor(VS_BATT_C1);
    if (r1 < 0.0) r1 = 0.0;
    if (c1 < 0.0) c1 = 0.0;
    if (r1 > 0.0 && c1 > 0.0 && dt > 0.0) {
        tau = r1 * c1;
        target_v_rc = battery_current * r1;
        if (dt >= tau)
            battery_v_rc = target_v_rc;
        else
            battery_v_rc += (target_v_rc - battery_v_rc) * dt / tau;
    }
    else if (r1 <= 0.0)
        battery_v_rc = 0.0;

    weak_v = read_sensor(VS_BATT_WEAK_CELL_V);
    weak_v += fabs(read_sensor(VS_BATT_IMBALANCE)) *
        battery_cell_ocv(chem, soc, battery_temp, age);

    ocv = series_cells * battery_cell_ocv(chem, soc, battery_temp, age)
        - weak_v;
    if (ocv < 0.0) ocv = 0.0;

    battery_terminal_voltage = ocv - battery_current*resistance - battery_v_rc;
    min_voltage = read_sensor(VS_BATT_MIN_VOLTAGE);
    if (soc <= 0.0 || battery_terminal_voltage < 0.0)
        battery_terminal_voltage = 0.0;

    max_current = read_sensor(VS_BATT_MAX_CURRENT);
    if (max_current > 0.0 && battery_current > max_current)
        battery_current = max_current;

    bms_vtrip = read_sensor(VS_BATT_BMS_VTRIP);
    bms_itrip = read_sensor(VS_BATT_BMS_ITRIP);
    bms_ttrip = read_sensor(VS_BATT_BMS_TTRIP);
    if (read_sensor(VS_BATT_BMS_ENABLE) > 0.5) {
        if ((bms_vtrip > 0.0 && battery_terminal_voltage < bms_vtrip) ||
            (bms_itrip > 0.0 && battery_current > bms_itrip) ||
            (bms_ttrip > 0.0 && battery_temp > bms_ttrip))
            battery_bms_tripped = 1;
    }
    if (min_voltage > 0.0 && battery_terminal_voltage < min_voltage &&
        soc <= 0.0)
        battery_terminal_voltage = 0.0;
    if (battery_bms_tripped) {
        battery_terminal_voltage = 0.0;
        battery_current = 0.0;
        battery_power = 0.0;
    }

    heat = battery_current*battery_current*(resistance + r1);
    if (battery_temp > read_sensor(VS_BATT_RUNAWAY_TEMP))
        heat += read_sensor(VS_BATT_RUNAWAY_HEAT);

    therm_mass = read_sensor(VS_BATT_THERM_MASS);
    therm_res = read_sensor(VS_BATT_THERM_RES);
    seawater_k = read_sensor(WATER_TEMP) + 273.15;
    if (therm_mass > 0.0 && therm_res > 0.0 && dt > 0.0)
        battery_temp += dt * (heat - (battery_temp - seawater_k)/therm_res)
            / therm_mass;

    battery_power = battery_terminal_voltage * battery_current;

    battery_energy_used += battery_power * dt;
    battery_charge_used += battery_current * dt;
    last_battery_time = present_time;

    charge_remaining = battery_charge_capacity - battery_charge_used;
    energy_remaining = battery_energy_capacity - battery_energy_used;
    if (charge_remaining < 0.0) charge_remaining = 0.0;
    if (energy_remaining < 0.0) energy_remaining = 0.0;
    if (battery_charge_capacity > 0.0)
        soc = charge_remaining / battery_charge_capacity;
    else if (battery_energy_capacity > 0.0)
        soc = energy_remaining / battery_energy_capacity;
    else
        soc = 0.0;

    measured_current = battery_current + read_sensor(VS_BATT_CURR_BIAS);
    measured_voltage = battery_terminal_voltage + read_sensor(VS_BATT_VOLT_BIAS);

    est_cap = read_sensor(VS_BATT_EST_CAP);
    if (est_cap <= 0.0) est_cap = battery_charge_capacity;
    est_soc = read_sensor(VS_BATT_EST_INIT_SOC) - battery_charge_used / est_cap;
    if (est_soc < 0.0) est_soc = 0.0;
    if (est_soc > 1.0) est_soc = 1.0;

    update_sensor( REAL_BATTERY_POWER, battery_power );
    update_sensor( REAL_BATTERY_CURRENT, battery_current );
    update_sensor( REAL_BATTERY_VOLTAGE, battery_terminal_voltage );
    update_sensor( REAL_BATTERY_SOC, soc );
    update_sensor( REAL_BATTERY_OCV, ocv );
    update_sensor( REAL_BATT_RESISTANCE, resistance );
    update_sensor( REAL_BATT_V_RC, battery_v_rc );
    update_sensor( REAL_BATT_TEMP, battery_temp );
    update_sensor( REAL_BATT_FAULT_HEAT, heat );
    update_sensor( REAL_BATT_CAPACITY, battery_charge_capacity );
    update_sensor( REAL_BATT_ENERGY_CAP, battery_energy_capacity );
    update_sensor( REAL_BATT_CHARGE_REM, charge_remaining );
    update_sensor( REAL_BATT_ENERGY_REM, energy_remaining );
    update_sensor( REAL_BATTERY_ENERGY_USED, battery_energy_used );
    update_sensor( REAL_BATT_CHARGE_USED, battery_charge_used );

    update_sensor( M_LOAD_CHARGE, battery_charge_used );
    update_sensor( M_LOAD_VOLTAGE, measured_voltage );
    update_sensor( M_BATTERY_VOLTAGE, measured_voltage );
    update_sensor( M_BATTERY_CURRENT, measured_current );
    update_sensor( M_BATTERY_TEMP, battery_temp );
    update_sensor( M_BATT_SOC_EST, est_soc );
}

static void update_thruster_diagnostics( struct thruster_state *ts )
{
    /* m_propeller_speed is existing telemetry; keep it for compatibility
       while also logging simulator truth in real_thruster_* channels. */
    update_sensor( M_PROPELLER_SPEED, 2*PI*60/5.54*ts->prop_omega );
    update_sensor( REAL_THR_PROP_SPEED, ts->prop_omega );
    update_sensor( REAL_THR_MOTOR_SPEED, ts->motor_omega );
    update_sensor( REAL_THRUSTER_VOLTAGE, ts->motor_voltage );
    update_sensor( REAL_THRUSTER_POWER, ts->elec_power );
    update_sensor( REAL_THRUSTER_TORQUE, ts->vehicle_torque );
    update_sensor( REAL_THRUSTER_THRUST, ts->thrust );
    update_sensor( REAL_THR_INFLOW_SPEED, ts->inflow );
    update_sensor( REAL_THR_DRIVE_TORQUE, ts->drivetrain_torque );
    update_sensor( REAL_THRUSTER_CURRENT, ts->amps );

    /* In simulator mode these measured channels are ideal telemetry from the
       truth model. Sensor faults should corrupt m_* rather than real_* values. */
    update_sensor( M_COPLEY_VOLTAGE, ts->motor_voltage );
    update_sensor( M_COPLEY_CURRENT, ts->amps );

    update_battery_load(ts);
}

double thruster_force( double real_amps, double w_speed )
{
    compute_thruster_state(real_amps, w_speed, &last_thruster_state);
    last_thruster_state_valid = 1;
    update_thruster_diagnostics(&last_thruster_state);
    return last_thruster_state.thrust;
}

/*======================================================================
  Modified this to reflect nature of Odyssey II's current control
  on the thruster controller...torque is proportional to current...
  J. G. Bellingham        1/94
 *======================================================================*/

double thruster_torque( double real_amps, double w_speed )
{
    if (!last_thruster_state_valid ||
        last_thruster_state.amps != real_amps ||
        last_thruster_state.inflow != read_sensor(VS_THRUSTER_WAKE_FRACTION)*w_speed)
        compute_thruster_state(real_amps, w_speed, &last_thruster_state);

    return last_thruster_state.vehicle_torque;
}

/*====================================================================*/


/*-----------------------------------------------------------------*
 | matrix calculations, not sure where their from, probably 
 | numerical recipes in fortran (note array element[0] unused!)
 *-----------------------------------------------------------------*/

/***************************************************************
  subroutine matinv(a,ainv,n,adum)

  this routine computes the inverse of a matrix.
  matrix inversion is performed by crout elimination

   input matrix :  a
  output matrix :  ainv
          order :  n
     error flag :  ier
         adum   :  dummy matrix of order  n x n .

----------------------------------------------------------------- */

int matinv( double a[][7], double ainv[][7], int n, double adum[][7] )
{
int ier,i,j,k,ic,ii,km,kp;
double p,t,s,dp,dt;
double eps;
eps= 1.0e-36;
/*  "ier" is a flag that is set to "1" if the matrix is singular. */
ier = 0;
/*
 special case for n=1
*/
if (n==1)
  {
   if ( fabs(a[1][1]) <= eps) 
      {
      ier=1;
      return (ier);
      }
   else
      {
      ainv[1][1] = 1.0/a[1][1];
      return (ier);
      }
  };    
/*
     copy  a  into  adum  because  a  is changed during computation.
*/
for (i=1;i<=n;i++)
    for (j=1;j<=n;j++)
        adum[i][j]=a[i][j];
/*
     initialize  ainv  to  i .
*/
for (i=1;i<=n;i++)
    for (j=1;j<=n;j++)
       {
       ainv[i][j] = 0.0e0;
       if (i==j) ainv[i][j] = 1.0e0;
       };
/*
    find the largest element in col-1.
*/
ic = 0;
ii = 0;
t  = fabs(adum[1][1]);
for (i=2;i<=n;i++)
   {
   if (t-fabs(adum[i][1]) < 0.0e0)  
       {
       ii = i;
       t  = fabs(adum[i][1]);
       }
   };
if (ii!=0) 
   {
    ic++;

    for (j=1;j<=n;j++)
       {
        s = ainv[1][j];
        ainv[1][j] = ainv[ii][j];
        ainv[ii][j] = s;
       };
   
    for (j=1;j<=n;j++)
       {
        s = adum[1][j];
        adum[1][j] = adum[ii][j];
        adum[ii][j] = s;
       };
   };
   
p = adum[1][1];

if (fabs(p)<=eps) 
  {
   /*
   c determinant is zero.
   */
   ier = 1;
   return (ier);
  }

/*
c  determinant is non-zero, continue
*/
for (j=2;j<=n;j++)
    adum[1][j] = adum[1][j]/p;

for (j=1;j<=n;j++)
    ainv[1][j] = ainv[1][j]/p;
 
for (k=2;k<=n;k++)      
   {    /*start big loop*/
    km = k - 1;
    t = -1.0;

    for (i=k;i<=n;i++)     
       {
        dp = adum[i][k];

        for (j=1;j<=km;j++) 
           dp = dp - adum[i][j] * adum[j][k];
           
        adum[i][k] = dp;
        if (t-fabs(adum[i][k]) < 0.0e0) 
          {
           t = fabs(adum[i][k]);
           ii = i;
          }
       }
     
    if (ii!=k)
      {
       ic++;

       for (j=1;j<=n;j++) 
          {
           s = ainv[k][j];
           ainv[k][j] = ainv[ii][j];
           ainv[ii][j] = s;
          };

       for (j=1;j<=n;j++)       
          {
           s = adum[k][j];
           adum[k][j] = adum[ii][j];
           adum[ii][j] = s;
          };
      }
    
    dt = adum[k][k];
    if (fabs(dt) <= eps) 
      {
       /*
       c determinant is zero.
       */
       ier = 1;
       return (ier);
      }
    p = p * dt;
    if (k!=n)
      {
       kp = k + 1;

       for (j=kp;j<=n;j++)     
          {
           dp = adum[k][j];

           for (i=1;i<=km;i++)   
              dp = dp - adum[k][i] * adum[i][j];
 
           adum[k][j] = dp/dt;
          };
      }
        
    for (j=1;j<=n;j++)   
       {
        dp = ainv[k][j];

        for (i=1;i<=km;i++)   
           dp = dp - adum[k][i] * ainv[i][j];

         ainv[k][j] = dp/dt;
        };
   };  /*end big loop*/

if ((ic%2)!=0) p=-p;
  
ii = n;

for (k=2;k<=n;k++)  
   {
    kp = ii;
    ii--;

    for (j=1;j<=n;j++)   
       {
        dp = ainv[ii][j];

        for (i=kp;i<=n;i++) 
            dp = dp - adum[ii][i] * ainv[i][j];

        ainv[ii][j] = dp;
       };
   };

return (ier);
}