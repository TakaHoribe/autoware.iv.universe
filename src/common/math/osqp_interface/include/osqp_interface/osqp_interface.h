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

#ifndef OSQP_INTERFACE_H
#define OSQP_INTERFACE_H

#include <eigen3/Eigen/SparseCore>
#include <vector>
#include <tuple>
#include "osqp.h"

namespace osqp
{
// Solves convex quadratic programs (QPs) using the OSQP solver.
//
// The function returns a tuple containing the solution as two float vectors.
// The first element of the tuple contains the 'primal' solution. The second element contains the 'lagrange multiplier'
// solution.
//
// About OSQP  https://osqp.org/docs/
//
// Problem definition:
//  minimize    1/2 * xt * A * x + qt * x
//  subject to  l <= A * x <= u
//
// How to use:
//   1. Generate the Eigen matrices P, A and vectors q, l, u according to the problem.
//   2. Call the optimization function
//        Ex: std::tuple<std::vector<float>, std::vector<float>> result;
//            result = osqp::optimize(P, A, q, l, u);
//   3. Access the optimal parameters
//        Ex: std::vector<float> param = std::get<0>(result);
std::tuple<std::vector<float>, std::vector<float>> optimize(const Eigen::MatrixXf &P, const Eigen::MatrixXf &A,
                                                            const std::vector<float> &q, const std::vector<float> &l,
                                                            const std::vector<float> &u);

// Struct for containing a 'Compressed-Column-Sparse Matrix'
// @param    elem_val Vector of non-zero values. Ex: [4,1,1,2]
// @param    row_idx  Row index corresponding to values. Ex: [0, 1, 0, 1] (Eigen: 'inner')
// @param    col_idx  List of 'val' indices where each column starts. Ex: [0, 2, 4] (Eigen: 'outer')
struct CSC_Matrix
{
  std::vector<c_float> elem_val;
  std::vector<c_int> row_idx;
  std::vector<c_int> col_idx;
};

// Converts a 'Eigen matrix A' into a 'CSC matrix' struct.
CSC_Matrix convEigenMatrixToCSCMatrix(const Eigen::MatrixXf& A);

// Converts a 'Eigen vector matrix x' into a 'dynamic float array'.
c_float *convEigenVecToDynFloatArray(const Eigen::MatrixXf &x);

} // namespace osqp

#endif // OSQP_INTERFACE_H
