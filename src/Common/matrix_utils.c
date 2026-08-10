#include <math.h>

#include "matrix_utils.h"

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