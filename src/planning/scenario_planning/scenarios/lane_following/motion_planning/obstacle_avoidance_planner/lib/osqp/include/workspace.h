#ifndef WORKSPACE_H
#define WORKSPACE_H

/*
 * This file was autogenerated by OSQP-Python on January 20, 2020 at 15:05:38.
 *
 * This file contains the prototypes for all the workspace variables needed
 * by OSQP. The actual data is contained inside workspace.c.
 */

#include "qdldl_interface.h"
#include "types.h"

// Data structure prototypes
extern csc Pdata;
extern csc Adata;
extern c_float qdata[200];
extern c_float ldata[200];
extern c_float udata[200];
extern OSQPData data;

// Settings structure prototype
extern OSQPSettings settings;

// Scaling structure prototypes
extern c_float Dscaling[200];
extern c_float Dinvscaling[200];
extern c_float Escaling[200];
extern c_float Einvscaling[200];
extern OSQPScaling scaling;

// Prototypes for linsys_solver structure
extern csc linsys_solver_L;
extern c_float linsys_solver_Dinv[400];
extern c_int linsys_solver_P[400];
extern c_float linsys_solver_bp[400];
extern c_float linsys_solver_sol[400];
extern c_float linsys_solver_rho_inv_vec[200];
extern qdldl_solver linsys_solver;

// Prototypes for solution
extern c_float xsolution[200];
extern c_float ysolution[200];

extern OSQPSolution solution;

// Prototype for info structure
extern OSQPInfo info;

// Prototypes for the workspace
extern c_float work_rho_vec[200];
extern c_float work_rho_inv_vec[200];
extern c_float work_x[200];
extern c_float work_y[200];
extern c_float work_z[200];
extern c_float work_xz_tilde[400];
extern c_float work_x_prev[200];
extern c_float work_z_prev[200];
extern c_float work_Ax[200];
extern c_float work_Px[200];
extern c_float work_Aty[200];
extern c_float work_delta_y[200];
extern c_float work_Atdelta_y[200];
extern c_float work_delta_x[200];
extern c_float work_Pdelta_x[200];
extern c_float work_Adelta_x[200];
extern c_float work_D_temp[200];
extern c_float work_D_temp_A[200];
extern c_float work_E_temp[200];

extern OSQPWorkspace workspace;

#endif  // ifndef WORKSPACE_H