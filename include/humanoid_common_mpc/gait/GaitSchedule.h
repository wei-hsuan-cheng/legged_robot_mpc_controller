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

#include <limits>
#include <mutex>

#include <ocs2_core/reference/ModeSchedule.h>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/gait/ModeSequenceTemplate.h"

namespace ocs2::humanoid {

/**
 * Online gait adaptation, e.g. from a learned policy (RL-augmented MPC, Kamohara et al. 2025):
 * every phase that has not started yet runs period-scale times its template duration, with
 * its swing apex raised by swingHeightOffset and placed at (0.5 + apexShift) of the swing.
 * The default is the template unchanged.
 */
struct GaitAdaptation {
  scalar_t periodScale = 1.0;
  scalar_t swingHeightOffset = 0.0;  // [m]
  scalar_t apexShift = 0.0;          // fraction of the swing duration

  bool operator==(const GaitAdaptation& other) const {
    return periodScale == other.periodScale && swingHeightOffset == other.swingHeightOffset && apexShift == other.apexShift;
  }
  bool operator!=(const GaitAdaptation& other) const { return !(*this == other); }
};

class GaitSchedule {
 public:
  /** The adaptation a phase was scheduled with. periodScale 0: a fixed duration, never retimed. */
  struct PhaseAdaptation {
    scalar_t periodScale = 0.0;
    scalar_t swingHeightOffset = 0.0;
    scalar_t apexShift = 0.0;
  };

  GaitSchedule(ModeSchedule initModeSchedule, ModeSequenceTemplate initModeSequenceTemplate, scalar_t phaseTransitionStanceTime);

  /**
   * @param [in] lowerBoundTime: The smallest time for which the ModeSchedule should be defined.
   * @param [in] upperBoundTime: The greatest time for which the ModeSchedule should be defined.
   */
  ModeSchedule getModeSchedule(scalar_t lowerBoundTime, scalar_t upperBoundTime);

  ModeSchedule getCurrentModeSchedule() const { return modeSchedule_; };

  /** Per-phase swing clearance, aligned with the current mode sequence. */
  const scalar_array_t& getSwingHeightSequence() const { return swingHeightSequence_; }

  /** Per-phase adaptation, aligned with the current mode sequence. */
  const std::vector<PhaseAdaptation>& getPhaseAdaptations() const { return phaseAdaptations_; }

  /**
   * Adapts every phase that starts after the one in progress at `time`: their durations are
   * rescaled in place (the contact sequence is unchanged, and so is the end of the current
   * phase) and they take the new swing offsets; phases tiled later inherit the adaptation.
   * Takes effect once per phase, at the first call after that phase started; calls later in the
   * same phase are ignored. Retiming the horizon at every solve moved its contact switches
   * under the warm start faster than one SQP iteration could follow.
   */
  void setAdaptation(const GaitAdaptation& adaptation, scalar_t time);

  /** Back to stance from `time` on, with the default template and no adaptation, as at start-up. */
  void reset(scalar_t time);

  /**
   * Used to insert a new user defined logic in the given time period.
   *
   * @param [in] startTime: The initial time from which the new mode sequence template should start.
   * @param [in] finalTime: The final time until when the new mode sequence needs to be defined.
   */
  void insertModeSequenceTemplate(const ModeSequenceTemplate& modeSequenceTemplate, scalar_t startTime, scalar_t finalTime);

  /** Creates a gait schedule from an initial mode schedule and a default mode sequence template. */
  static std::shared_ptr<GaitSchedule> createGaitSchedule(const ModeSchedule& initModeSchedule,
                                                          const ModeSequenceTemplate& defaultModeSequenceTemplate,
                                                          const ModelSettings& modelSettings,
                                                          bool verbose = false);

  void updateModeSchedule(const ModeSchedule& modeSchedule);

 private:
  /**
   * Extends the switch information from lowerBoundTime to upperBoundTime based on the template mode sequence.
   *
   * @param [in] startTime: The initial time from which the mode schedule should be appended with the template.
   * @param [in] finalTime: The final time to which the mode schedule should be appended with the template.
   */
  void tileModeSequenceTemplate(scalar_t startTime, scalar_t finalTime);

 private:
  ModeSchedule modeSchedule_;
  scalar_array_t swingHeightSequence_;
  std::vector<PhaseAdaptation> phaseAdaptations_;
  ModeSequenceTemplate modeSequenceTemplate_;  //!< as inserted; tiling applies adaptation_
  const ModeSequenceTemplate defaultModeSequenceTemplate_;
  GaitAdaptation adaptation_;
  scalar_t adaptedPhaseStart_ = std::numeric_limits<scalar_t>::quiet_NaN();  //!< start of the phase adapted last
  scalar_t phaseTransitionStanceTime_;
};

}  // namespace ocs2::humanoid
