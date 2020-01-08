#ifndef WORKSPACE_H
#define WORKSPACE_H

/*
 * This file was autogenerated by OSQP-Python on December 27, 2019 at 13:22:07.
 * 
 * This file contains the prototypes for all the workspace variables needed
 * by OSQP. The actual data is contained inside workspace.c.
 */

#include "types.h"
#include "qdldl_interface.h"

// Data structure prototypes
extern csc Pdata;
extern csc Adata;
extern c_float qdata[1000];
extern c_float ldata[1000];
extern c_float udata[1000];
extern OSQPData data;

// Settings structure prototype
extern OSQPSettings settings;

// Scaling structure prototypes
extern c_float Dscaling[1000];
extern c_float Dinvscaling[1000];
extern c_float Escaling[1000];
extern c_float Einvscaling[1000];
extern OSQPScaling scaling;

// Prototypes for linsys_solver structure
extern csc linsys_solver_L;
extern c_float linsys_solver_Dinv[2000];
extern c_int linsys_solver_P[2000];
extern c_float linsys_solver_bp[2000];
extern c_float linsys_solver_sol[2000];
extern c_float linsys_solver_rho_inv_vec[1000];
extern qdldl_solver linsys_solver;

// Prototypes for solution
extern c_float xsolution[1000];
extern c_float ysolution[1000];

extern OSQPSolution solution;

// Prototype for info structure
extern OSQPInfo info;

// Prototypes for the workspace
extern c_float work_rho_vec[1000];
extern c_float work_rho_inv_vec[1000];
extern c_float work_x[1000];
extern c_float work_y[1000];
extern c_float work_z[1000];
extern c_float work_xz_tilde[2000];
extern c_float work_x_prev[1000];
extern c_float work_z_prev[1000];
extern c_float work_Ax[1000];
extern c_float work_Px[1000];
extern c_float work_Aty[1000];
extern c_float work_delta_y[1000];
extern c_float work_Atdelta_y[1000];
extern c_float work_delta_x[1000];
extern c_float work_Pdelta_x[1000];
extern c_float work_Adelta_x[1000];
extern c_float work_D_temp[1000];
extern c_float work_D_temp_A[1000];
extern c_float work_E_temp[1000];

extern OSQPWorkspace workspace;

#endif // ifndef WORKSPACE_H
