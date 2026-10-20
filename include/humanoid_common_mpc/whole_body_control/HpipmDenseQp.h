/******************************************************************************
Copyright (c) 2026, Wei-Hsuan Cheng. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR
THE USE OR OTHER DEALINGS IN THE SOFTWARE.
******************************************************************************/

#pragma once

#include <memory>
#include <vector>

#include <ocs2_core/Types.h>

namespace ocs2::humanoid {

/**
 * Dense QP solved with HPIPM's interior-point method:
 *
 *   min  0.5 x' H x + g' x
 *   s.t. A x  = b
 *        D x <= f
 *        lb <= x <= ub
 *
 * HPIPM is used because it is already a dependency of this workspace through
 * ocs2_sqp; qpOASES, which the reference quadruped WBC uses, is not available
 * here. HPIPM wants column-major data, which is Eigen's default, so the matrices
 * are handed over without a copy-transpose.
 *
 * All HPIPM workspaces are allocated once per problem SHAPE and reused. The WBC
 * changes shape whenever the number of stance feet changes, which on a biped is
 * only a handful of distinct shapes, so in steady state this allocates nothing.
 */
class HpipmDenseQp {
 public:
  struct Settings {
    int iterMax{30};
    scalar_t tolStat{1e-6};   //!< stationarity
    scalar_t tolEq{1e-7};     //!< equality-constraint residual
    scalar_t tolIneq{1e-7};   //!< inequality-constraint residual
    scalar_t tolComp{1e-7};   //!< complementarity
    scalar_t regPrim{1e-8};   //!< primal regularisation, keeps H positive definite
  };

  HpipmDenseQp();
  explicit HpipmDenseQp(Settings settings);
  ~HpipmDenseQp();

  HpipmDenseQp(const HpipmDenseQp&) = delete;
  HpipmDenseQp& operator=(const HpipmDenseQp&) = delete;

  /**
   * Solves the QP. Returns false if HPIPM did not converge, in which case
   * `solution` holds whatever iterate it stopped at - the caller decides whether
   * to use it or fall back, because a WBC must never simply throw in the control
   * loop.
   *
   * `boundedIndices` lists the decision variables that carry a box bound, and
   * `lowerBound`/`upperBound` hold one entry per listed index. Unbounded
   * variables are simply left out rather than given an infinite bound, which
   * HPIPM's interior-point method does not handle gracefully.
   */
  bool solve(const matrix_t& H,
             const vector_t& g,
             const matrix_t& A,
             const vector_t& b,
             const matrix_t& D,
             const vector_t& f,
             const std::vector<int>& boundedIndices,
             const vector_t& lowerBound,
             const vector_t& upperBound,
             vector_t& solution);

  int lastIterationCount() const { return lastIterations_; }
  int lastStatus() const { return lastStatus_; }

 private:
  struct Impl;  //!< hides the HPIPM C structs from every translation unit that includes this
  std::unique_ptr<Impl> impl_;
  Settings settings_;
  int lastIterations_{0};
  int lastStatus_{0};
};

}  // namespace ocs2::humanoid
