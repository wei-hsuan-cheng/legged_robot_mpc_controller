/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.
Copyright (c) 2024, 1X Technologies. All rights reserved.

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
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <humanoid_common_mpc/common/Types.h>
#include <cmath>

namespace ocs2::humanoid {

class BreakFrequencyAlphaFilter final {
 public:
  /**
   * Constructor
   *
   * @param [in] breakFrequency: Break frequency (cut-off frequency) in Hz.
   */
  BreakFrequencyAlphaFilter(scalar_t breakFrequency, const vector_t& y_init)
      : breakDeltaT_(1 / (2 * M_PI * breakFrequency)), y_last_(y_init) {}

  vector_t getFilteredVector(const vector_t& x, scalar_t time) {
    assert(x.size() == y_last_.size());
    if (!initialized_ || time < lastTimeFilterCalled_) {
      initialized_ = true;
      lastTimeFilterCalled_ = time;
      return y_last_;
    }

    const scalar_t deltaTime = time - lastTimeFilterCalled_;
    const scalar_t alpha = deltaTime / (deltaTime + breakDeltaT_);
    y_last_ = alpha * x + (1 - alpha) * y_last_;
    lastTimeFilterCalled_ = time;
    return y_last_;
  }

  void reset(const vector_t& y_init) {
    assert(y_init.size() == y_last_.size());
    y_last_ = y_init;
    initialized_ = false;
  }

 private:
  scalar_t breakDeltaT_;
  vector_t y_last_;
  scalar_t lastTimeFilterCalled_{0.0};
  bool initialized_{false};
};

}  // namespace ocs2::humanoid
