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

#include "humanoid_common_mpc/whole_body_control/HpipmDenseQp.h"

#include <algorithm>
#include <limits>

extern "C" {
#include <hpipm_d_dense_qp.h>
#include <hpipm_d_dense_qp_dim.h>
#include <hpipm_d_dense_qp_ipm.h>
#include <hpipm_d_dense_qp_sol.h>
}

namespace ocs2::humanoid {

struct HpipmDenseQp::Impl {
  // Problem shape the current workspaces were built for; -1 forces a rebuild.
  int nv{-1}, ne{-1}, nb{-1}, ng{-1};

  d_dense_qp_dim dim{};
  d_dense_qp qp{};
  d_dense_qp_sol sol{};
  d_dense_qp_ipm_arg arg{};
  d_dense_qp_ipm_ws ws{};

  std::vector<char> dimMem, qpMem, solMem, argMem, wsMem;
  std::vector<int> idxb;

  ~Impl() = default;

  bool matchesShape(int nv_, int ne_, int nb_, int ng_) const {
    return nv == nv_ && ne == ne_ && nb == nb_ && ng == ng_;
  }

  void build(int nv_, int ne_, int nb_, int ng_, const HpipmDenseQp::Settings& s) {
    nv = nv_;
    ne = ne_;
    nb = nb_;
    ng = ng_;

    dimMem.assign(static_cast<size_t>(d_dense_qp_dim_memsize()), 0);
    d_dense_qp_dim_create(&dim, dimMem.data());
    d_dense_qp_dim_set_all(nv, ne, nb, ng, 0, 0, &dim);

    qpMem.assign(static_cast<size_t>(d_dense_qp_memsize(&dim)), 0);
    d_dense_qp_create(&dim, &qp, qpMem.data());

    solMem.assign(static_cast<size_t>(d_dense_qp_sol_memsize(&dim)), 0);
    d_dense_qp_sol_create(&dim, &sol, solMem.data());

    argMem.assign(static_cast<size_t>(d_dense_qp_ipm_arg_memsize(&dim)), 0);
    d_dense_qp_ipm_arg_create(&dim, &arg, argMem.data());
    d_dense_qp_ipm_arg_set_default(::SPEED, &arg);

    int iterMax = s.iterMax;
    auto tolStat = s.tolStat;
    auto tolEq = s.tolEq;
    auto tolIneq = s.tolIneq;
    auto tolComp = s.tolComp;
    auto regPrim = s.regPrim;
    d_dense_qp_ipm_arg_set_iter_max(&iterMax, &arg);
    d_dense_qp_ipm_arg_set_tol_stat(&tolStat, &arg);
    d_dense_qp_ipm_arg_set_tol_eq(&tolEq, &arg);
    d_dense_qp_ipm_arg_set_tol_ineq(&tolIneq, &arg);
    d_dense_qp_ipm_arg_set_tol_comp(&tolComp, &arg);
    d_dense_qp_ipm_arg_set_reg_prim(&regPrim, &arg);

    wsMem.assign(static_cast<size_t>(d_dense_qp_ipm_ws_memsize(&dim, &arg)), 0);
    d_dense_qp_ipm_ws_create(&dim, &arg, &ws, wsMem.data());

    // idxb lists only the variables that actually carry a bound. HPIPM treats a
    // box constraint as two-sided, so handing it +/-inf for unbounded variables
    // makes the interior-point method chase infinite slacks and stall - which is
    // exactly what it did before this was narrowed down.
    idxb.resize(static_cast<size_t>(nb));
  }
};

HpipmDenseQp::HpipmDenseQp() : HpipmDenseQp(Settings{}) {}

HpipmDenseQp::HpipmDenseQp(Settings settings) : impl_(std::make_unique<Impl>()), settings_(settings) {}

HpipmDenseQp::~HpipmDenseQp() = default;

bool HpipmDenseQp::solve(const matrix_t& H,
                         const vector_t& g,
                         const matrix_t& A,
                         const vector_t& b,
                         const matrix_t& D,
                         const vector_t& f,
                         const std::vector<int>& boundedIndices,
                         const vector_t& lowerBound,
                         const vector_t& upperBound,
                         vector_t& solution) {
  const int nv = static_cast<int>(H.rows());
  const int ne = static_cast<int>(A.rows());
  const int ng = static_cast<int>(D.rows());
  const int nb = static_cast<int>(boundedIndices.size());

  if (!impl_->matchesShape(nv, ne, nb, ng)) {
    impl_->build(nv, ne, nb, ng, settings_);
  }
  std::copy(boundedIndices.begin(), boundedIndices.end(), impl_->idxb.begin());

  // HPIPM reads column-major; Eigen's default storage order already matches, so
  // these hand over the buffers directly. const_cast is needed only because the
  // C API is not const-correct; HPIPM copies the data in.
  auto* Hp = const_cast<double*>(H.data());
  auto* gp = const_cast<double*>(g.data());

  d_dense_qp_set_H(Hp, &impl_->qp);
  d_dense_qp_set_g(gp, &impl_->qp);
  if (ne > 0) {
    d_dense_qp_set_A(const_cast<double*>(A.data()), &impl_->qp);
    d_dense_qp_set_b(const_cast<double*>(b.data()), &impl_->qp);
  }
  if (nb > 0) {
    d_dense_qp_set_idxb(impl_->idxb.data(), &impl_->qp);
    d_dense_qp_set_lb(const_cast<double*>(lowerBound.data()), &impl_->qp);
    d_dense_qp_set_ub(const_cast<double*>(upperBound.data()), &impl_->qp);
  }
  if (ng > 0) {
    // One-sided rows: D x <= f. The lower side is switched off with a zero mask
    // rather than set to -inf, which is how HPIPM expresses "no bound here".
    vector_t lg = vector_t::Zero(ng);
    vector_t lgMask = vector_t::Zero(ng);
    vector_t ugMask = vector_t::Ones(ng);
    d_dense_qp_set_C(const_cast<double*>(D.data()), &impl_->qp);
    d_dense_qp_set_lg(lg.data(), &impl_->qp);
    d_dense_qp_set_ug(const_cast<double*>(f.data()), &impl_->qp);
    d_dense_qp_set_lg_mask(lgMask.data(), &impl_->qp);
    d_dense_qp_set_ug_mask(ugMask.data(), &impl_->qp);
  }

  d_dense_qp_ipm_solve(&impl_->qp, &impl_->sol, &impl_->arg, &impl_->ws);

  d_dense_qp_ipm_get_status(&impl_->ws, &lastStatus_);
  d_dense_qp_ipm_get_iter(&impl_->ws, &lastIterations_);

  solution.resize(nv);
  d_dense_qp_sol_get_v(&impl_->sol, solution.data());

  return lastStatus_ == 0 && solution.allFinite();
}

}  // namespace ocs2::humanoid
