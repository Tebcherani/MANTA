/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: ekf_work.c,v 2.21 1998/01/20 00:31:51 bamoran Exp $
 *----------------------------------------------------------------------*/

#include "ekf.h"

/* The following routines are from ~jleonard/ox/scan/ekf_work.c */

void pred_state_covar( matrix_t F, matrix_t P_prev, matrix_t Q, matrix_t
		       P_pred, int XDIM )
{
    static int init= -1;
    static matrix_t temp1;
    static matrix_t temp2;
    static matrix_t Ftrans;

    if (init)
    {
        init = 0;
        temp1 = mat_new(MAX_XDIM,MAX_XDIM);
        temp2 = mat_new(MAX_XDIM,MAX_XDIM);
        Ftrans = mat_new(MAX_XDIM,MAX_XDIM);
    }

    qmat_trans(F,Ftrans,XDIM,XDIM);
    qmat_mpy(F,P_prev,temp1,XDIM,XDIM,XDIM);
    qmat_mpy(temp1,Ftrans,temp2,XDIM,XDIM,XDIM);
    qmat_add(temp2,Q,P_pred,XDIM,XDIM,PLUS);
}

void innov_covar(H,P_pred,R, S,XDIM,ZDIM)
matrix_t H,P_pred, R, S;
int XDIM, ZDIM;
{
static int init= -1;
static matrix_t temp1;
static matrix_t temp2;
static matrix_t Htrans;

if (init)
        {
        init = 0;
        temp1 = mat_new(MAX_ZDIM,MAX_XDIM);
        temp2 = mat_new(MAX_ZDIM,MAX_ZDIM);
        Htrans = mat_new(MAX_XDIM,MAX_ZDIM);  /* change 19 April 89 */
        }

qmat_mpy(H,P_pred,temp1,ZDIM,XDIM,XDIM);
qmat_trans(H,Htrans,ZDIM,XDIM);
qmat_mpy(temp1,Htrans,temp2,ZDIM,XDIM,ZDIM);
qmat_add(temp2,R,S,ZDIM,ZDIM,PLUS);
}


void filter_gain(P_pred,H,S,W,XDIM,ZDIM)
matrix_t P_pred, H, S, W;
int XDIM, ZDIM;
{
static int init= -1;
static matrix_t Htrans;
static matrix_t temp1;
static matrix_t Sinv;
 
if (init)
        {
        init = 0;
        temp1 = mat_new(MAX_XDIM,MAX_ZDIM);
        Htrans = mat_new(MAX_XDIM,MAX_ZDIM);
        Sinv = mat_new(MAX_ZDIM,MAX_ZDIM);
        }

qmat_trans(H,Htrans,ZDIM,XDIM);

qmat_mpy(P_pred,Htrans,temp1,XDIM,XDIM,ZDIM);
qmat_inv(S,Sinv,ZDIM);
qmat_mpy(temp1,Sinv,W,XDIM,ZDIM,ZDIM);

}

void innov(z, zhat_pred, nu,ZDIM)
matrix_t z, zhat_pred, nu;
int ZDIM;
{
qmat_add(z,zhat_pred,nu,ZDIM,1,MINUS);
}
 
void norm_innov(nu,S, n_innov,ZDIM)
matrix_t nu, S, n_innov;
int ZDIM; 
{

static int init= -1;
static matrix_t nutrans;
static matrix_t temp1; 
static matrix_t Sinv; 
 
if (init) 
        { 
        init = 0; 
        temp1 = mat_new(MAX_ZDIM, MAX_ZDIM); 
        nutrans = mat_new(1,MAX_ZDIM); 
        Sinv = mat_new(MAX_ZDIM,MAX_ZDIM);
        } 

qmat_trans(nu,nutrans,ZDIM,1);
qmat_inv(S,Sinv,ZDIM);
qmat_mpy(nutrans,Sinv,temp1,1,ZDIM,ZDIM);
qmat_mpy(temp1,nu,n_innov,1,ZDIM,1); 
}
 
void pred_state(W, nu, xhat_pred, xhat_est,XDIM, ZDIM)
matrix_t W, nu, xhat_pred, xhat_est;
int XDIM, ZDIM;
{

static int init= -1;
static matrix_t temp1;  
  
if (init)  
        {  
        init = 0;  
        temp1 = mat_new(MAX_ZDIM, 1); 
        }   

qmat_mpy(W,nu,temp1,XDIM,ZDIM,1);
qmat_add(xhat_pred,temp1,xhat_est,XDIM,1,PLUS);
}
 
void state_covar(W,S,P_pred, P_est, XDIM, ZDIM)
matrix_t W,S,P_pred, P_est;
int XDIM, ZDIM;
{
static int init= -1;
static matrix_t Wtrans;
static matrix_t temp1;  
static matrix_t temp2;  
 
if (init)
        {
        init = 0;
        temp1 = mat_new(MAX_XDIM, MAX_ZDIM);
        temp2 = mat_new(MAX_XDIM, MAX_XDIM);
        Wtrans = mat_new(MAX_ZDIM, MAX_XDIM); 
        }
qmat_trans(W,Wtrans,XDIM,ZDIM);
qmat_mpy(W,S,temp1,XDIM,ZDIM,ZDIM);
qmat_mpy(temp1,Wtrans,temp2,XDIM,ZDIM,XDIM);
qmat_add(P_pred, temp2,P_est,XDIM,XDIM,MINUS);
}

void norm_state_err(P_est, x, xhat_est, norm_staterr,XDIM)
matrix_t P_est, x, xhat_est, norm_staterr;
int XDIM;
{
static int init= -1;
static matrix_t xhat_tilde;
static matrix_t xhat_tilde_tr;
static matrix_t temp2;
static matrix_t Pinv;

if (init)
        {
        init = 0;
        temp2 = mat_new(MAX_XDIM, MAX_XDIM);
        xhat_tilde = mat_new(MAX_XDIM, 1);
        xhat_tilde_tr = mat_new(1,MAX_XDIM);
        Pinv = mat_new(MAX_XDIM, MAX_XDIM);
        }

qmat_inv(P_est,Pinv,XDIM);
qmat_add(x,xhat_est,xhat_tilde,XDIM,1,MINUS);
qmat_trans(xhat_tilde, xhat_tilde_tr,XDIM,1);
qmat_mpy(xhat_tilde_tr,Pinv,temp2,1,XDIM,XDIM);
qmat_mpy(temp2,xhat_tilde,norm_staterr,1,XDIM,1);

}




