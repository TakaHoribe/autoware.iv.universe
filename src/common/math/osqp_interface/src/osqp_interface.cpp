/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Author: Robin Karlsson
 */

#include "osqp_interface/osqp_interface.h"

namespace osqp
{
inline bool isEqual(double x, double y)
{
  const double epsilon = 1e-6;
  return std::abs(x - y) <= epsilon * std::abs(x);
}

std::tuple<std::vector<float>, std::vector<float>> optimize(const Eigen::MatrixXf &P, const Eigen::MatrixXf &A,
                                                            const std::vector<float> &q, const std::vector<float> &l,
                                                            const std::vector<float> &u)
{
  /*******************
   * SET UP MATRICES
   *******************/
  // CSC matrices
  CSC_Matrix P_csc = convEigenMatrixToCSCMatrix(P);
  CSC_Matrix A_csc = convEigenMatrixToCSCMatrix(A);
  // Dynamic float arrays
  std::vector<c_float> q_tmp(q.begin(), q.end());
  std::vector<c_float> l_tmp(l.begin(), l.end());
  std::vector<c_float> u_tmp(u.begin(), u.end());
  c_float* q_dyn = q_tmp.data();
  c_float* l_dyn = l_tmp.data();
  c_float* u_dyn = u_tmp.data();

  /**********************
   * OBJECTIVE FUNCTION
   **********************/
  // Number of constraints
  c_int constr_m = A.rows();
  // Number of parameters
  c_int param_n = P.rows();
  // Number of elements
  c_int P_elem_N = P.size();
  c_int A_elem_N = A.size();

  // Problem settings
  OSQPSettings* settings = (OSQPSettings*)c_malloc(sizeof(OSQPSettings));

  // Structures
  OSQPWorkspace* work;  // Workspace
  OSQPData* data;       // OSQPData

  // Populate data
  data = (OSQPData*)c_malloc(sizeof(OSQPData));

  data->m = constr_m;
  data->n = param_n;
  data->P = csc_matrix(data->n, data->n, P_elem_N, P_csc.elem_val.data(), P_csc.row_idx.data(), P_csc.col_idx.data());
  data->q = q_dyn;

  /**********************
   * LINEAR CONSTRAINTS
   **********************/
  data->A = csc_matrix(data->m, data->n, A_elem_N, A_csc.elem_val.data(), A_csc.row_idx.data(), A_csc.col_idx.data());
  data->l = l_dyn;
  data->u = u_dyn;

  /************
   * OPTIMIZE
   ************/
  // Define Solver settings as default
  osqp_set_default_settings(settings);
  settings->alpha = 1.6;  // Change alpha parameter
  settings->eps_rel = 1.0E-4;
  settings->eps_abs = 1.0E-4;
  settings->eps_prim_inf = 1.0E-4;
  settings->eps_dual_inf = 1.0E-4;
  settings->warm_start = true;
  settings->max_iter = 4000;


  // Setup workspace
  work = osqp_setup(data, settings);

  // Solve Problem
  osqp_solve(work);

  /********************
   * EXTRACT SOLUTION
   ********************/
  c_float* sol_x = work->solution->x;
  c_float* sol_y = work->solution->y;
  std::vector<float> sol_primal(sol_x, sol_x + static_cast<int>(param_n));
  std::vector<float> sol_lagrange_multiplier(sol_y, sol_y + static_cast<int>(param_n));
  // Result tuple
  std::tuple<std::vector<float>, std::vector<float>> result = std::make_tuple(sol_primal, sol_lagrange_multiplier);

  /***********
   * CLEANUP
   ***********/
  osqp_cleanup(work);
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);

  return result;
}

CSC_Matrix convEigenMatrixToCSCMatrix(const Eigen::MatrixXf& A)
{
  // Input dense matrix dimensions
  int A_rows = A.rows();
  int A_cols = A.cols();

  /************************************************************************
   * Generate 'sparse matrix B' from nonzero elements in 'dense matrix A'
   ************************************************************************/

  // Generate list of nonzero elements
  std::vector<Eigen::Triplet<c_float>> triplet_list;
  triplet_list.reserve(A.size());
  for (int i = 0; i < A_rows; i++)
  {
    for (int j = 0; j < A_cols; j++)
    {
      float A_val = A(i, j);
      if (!isEqual(A_val, 0.0))
      {
        triplet_list.push_back(Eigen::Triplet<c_float>(i, j, A_val));
      }
    }
  }
  // Generate 'sparse matrix B' and fill with nonzero elements in list
  Eigen::SparseMatrix<c_float> B(A_rows, A_cols);
  B.setFromTriplets(triplet_list.begin(), triplet_list.end());

  /************************************************************************
   * Generate 'Compressed Sparse Column (CSC) Matrix A_csc' struct object
   ************************************************************************/

  // Generate CSC matrix B
  int B_nonzero_N = B.nonZeros();
  B.makeCompressed();

  // Extract pointer arrays
  c_float* val_ptr = B.valuePtr();
  int* inn_ptr = B.innerIndexPtr();
  int* out_ptr = B.outerIndexPtr();

  // Copy values of pointer arrays into vectors in CSC struct
  // Array lengths:
  //     elem_val : nonzero element count
  //     row_idx  : nonzero element count
  //     col_idx  : input matrix column count + 1
  CSC_Matrix A_csc;
  A_csc.elem_val.assign(val_ptr, val_ptr + B_nonzero_N);
  A_csc.col_idx.assign(out_ptr, out_ptr + A_cols + 1);
  A_csc.row_idx.assign(inn_ptr, inn_ptr + B_nonzero_N);

  return A_csc;
}

}  // namespace osqp
