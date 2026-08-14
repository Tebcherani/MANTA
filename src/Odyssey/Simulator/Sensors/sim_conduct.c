/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: sim_conduct.c,v 2.21 1998/01/20 00:31:51 bamoran Exp $
 *----------------------------------------------------------------------*/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "odyssey.h"

#include "vehicle.h"

#define DECIBAR_TO_PA 10000.0

/**************************************************************************
 ************************* SIM_CONDUCT FUNCTION  **************************
 **************************************************************************/

double SalCond(double, double, double, int);
double Sal(double, double);
double Dsal(double, double);
double Rt35(double);
double A(double);
double B(double);
double C(double);


static double conduct = 0.0, current_depth;
static double transition1 = 6.0, transition2 = 18.0, transition3 = 32.0;

void update_water_conductivity(void)
{
/*
 * A simple "layer cake" with no significance other than to insure
 * changing values.
 */
    current_depth = read_sensor(REAL_Z);
    if(current_depth <= transition1) {
        conduct = SalCond(23.1759, 40.0, 5.0 * current_depth, 1);
    } else if(current_depth <= transition2) {
        conduct = SalCond(23.1759, 40.0, 4.0 * current_depth, 1);
    } else if(current_depth <= transition3) {
        conduct = SalCond(23.1759, 40.0, 2.0 * current_depth, 1);
    }

    update_sensor( WATER_COND, conduct );
    return;
}

/*  NOTE: Sal to Cond works, but I am suspicious of Cond to Sal.
 *
 *   SalCond(Cond_Sal, Temp, Press, Flag) - Given the Conductivity 
 * Ratio, Temperature & Pressure and Flag = 0, returns the Salinity.
 *
 * UNITS:
 *                Pressure         P          Decibars ( = 10^4 Pa)
 *                Temperature      T          Deg. Celcius
 *                Conductivity     Cond       Ratio
 *                Salinity         Sal78      ???
 *
 * Given Salinity, Temperature & Pressure and Flag = 1, returns the
 * Conductivity Ratio.
 *
 * Returns 0 for Conductivity ratio < 0.0005
 * Returns 0 for Salinity < 0.02
 * Check:
 *      SalCond = 1.888091, Cond_Sal = 40.0, T = 40.0, P = 10000., Flag = 1
 *      SalCond = 40.0, Cond_Sal = 1.888091, T = 40.0, P = 10000., Flag = 0
 */

double SalCond(double Cond_Sal, double T, double P, int Flag)
{
        double R, Rt, Sal78, Si, diff, Rtt, At, Bt, Cp, Dt;
        int i;

        Dt = T - 15.0;

        if(Flag == 0)  {             /* Convert conductivity to salinity */
                R = Cond_Sal;
                Rt = R/(Rt35(T) * (1.0 + C(P)/(B(T) + A(T) * R)));
                Rt = sqrt(fabs(Rt));
                Sal78 = Sal(Rt, Dt);
                if(Sal78 <= 0.0005)
                        return(0.0);
                else
                        return(Sal78);
        } else  {                   /* Convert salinity to conductivity */
                Rt = sqrt(Cond_Sal/35.0);
                Si = Sal(Rt, Dt);
                diff = fabs(Si - Cond_Sal);
                for(i = 0; i <= 10; i++)  {
                        while(diff >= 1.0e-4)  {
                                Rt = Rt + (Cond_Sal - Si)/Dsal(Rt, Dt);
                                Si = Sal(Rt, Dt);
                                diff = abs(Si - Cond_Sal);
                        }
                }
                Rtt = Rt35(T) * Rt * Rt;
                At = A(T);
                Bt = B(T);
                Cp = C(P);
                Cp = Rtt * (Cp + Bt);
                Bt = Bt - Rtt * At;
/*
 * Get the Ratio by solving the quadratic.
 */
                 R = sqrt(fabs(Bt*Bt + 4.0*At*Cp)) - Bt;
                 Sal78 = 0.5 * R/At;
                 if(Sal78 <= 0.02)
                         return(0.0);
                 else
                         return(Sal78);
        }
}

/*
 *  Sal(Rt, Dt) = Salinity at Conductivity ratio R at temp t, Relative
 * to adjusted temperature Dt = T - 15.
 */
double Sal(double Xr, double Xt)
{
double dummy;

         dummy = ((((2.7081 * Xr - 7.0261) * Xr + 14.0941) * Xr + 25.3851) * 
                Xr - 0.1692) * Xr + 0.0080 + (Xt/(1.0 + 0.0162 * Xt)) * (((((
                   -0.0144 * Xr + 0.0636) * Xr - 0.0375) * Xr - 0.0066) * Xr 
                                - 0.0056) * Xr + 0.0005);
        return(dummy);
        
}

/*
 * Dsal(Rt, Dt) - Derivative of salinity.
 */
double Dsal(double Xr, double Xt)
 {
  double dummy;
        dummy = ((((13.5405 * Xr - 28.1044) * Xr + 42.2823) * Xr + 50.7702) * 
           Xr - 0.1692) + (Xt/(1.0 + 0.0162 * Xt)) * ((((-0.0720 * Xr + 0.2544)
              * Xr - 0.1125) * Xr - 0.0132) * Xr - 0.0056);
        return(dummy);
 }
 
/*
 * Rt35 = Baseline Ratio: C(35, T, 0)/C(35, 15, 0) variation with Temp.
 */
double Rt35(double Xt)
{
  double dummy;
        dummy = (((1.0031e-9 * Xt - 6.9698e-7) * Xt + 1.104259e-4) * Xt + 
                 2.00564e-2) * Xt + 0.6766097;
         return(dummy);
}

 /*
  * A, B, C Polynomials from the Rp Equation.
  */
double C(double Xp)
  {
  double dummy;
        dummy = ((3.989e-15 * Xp - 6.370e-10) * Xp + 2.070e-5) * Xp;
        return(dummy);
  }

double B(double Xt)        /* From Lewis 1980 */
  {
  double dummy;
        dummy = (4.464e-4 * Xt + 3.426e-2) * Xt + 1.0;
        return(dummy);
  }
  
double A(double Xt)        /* From Lewis 1980 */
  {
  double dummy;
        dummy = -3.107e-3 * Xt + 0.4215;
        return(dummy);
  }
  
