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

#include "humanoid_common_mpc/gait/GaitSchedule.h"

#include <algorithm>
#include <stdexcept>

#include <ocs2_core/misc/Lookup.h>

#include <ocs2_core/misc/LoadData.h>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
GaitSchedule::GaitSchedule(ModeSchedule initModeSchedule, ModeSequenceTemplate initModeSequenceTemplate, scalar_t phaseTransitionStanceTime)
    : modeSchedule_(std::move(initModeSchedule)),
      swingHeightSequence_(modeSchedule_.modeSequence.size(), initModeSequenceTemplate.swingHeight),
      phaseAdaptations_(modeSchedule_.modeSequence.size()),
      modeSequenceTemplate_(initModeSequenceTemplate),
      defaultModeSequenceTemplate_(std::move(initModeSequenceTemplate)),
      phaseTransitionStanceTime_(phaseTransitionStanceTime) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void GaitSchedule::insertModeSequenceTemplate(const ModeSequenceTemplate& modeSequenceTemplate, scalar_t startTime, scalar_t finalTime) {
  modeSequenceTemplate_ = modeSequenceTemplate;
  auto& eventTimes = modeSchedule_.eventTimes;
  auto& modeSequence = modeSchedule_.modeSequence;

  // find the index on which the new gait should be added
  const size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(), startTime) - eventTimes.begin();

  // delete the old logic from the index
  if (index < eventTimes.size()) {
    eventTimes.erase(eventTimes.begin() + index, eventTimes.end());
    modeSequence.erase(modeSequence.begin() + index + 1, modeSequence.end());
    swingHeightSequence_.erase(swingHeightSequence_.begin() + index + 1, swingHeightSequence_.end());
    phaseAdaptations_.erase(phaseAdaptations_.begin() + index + 1, phaseAdaptations_.end());
  }

  // add an intermediate stance phase
  scalar_t phaseTransitionStanceTime = phaseTransitionStanceTime_;
  if (!modeSequence.empty() && modeSequence.back() == ModeNumber::STANCE) {
    phaseTransitionStanceTime = 0.0;
  }

  if (phaseTransitionStanceTime > 0.0) {
    eventTimes.push_back(startTime);
    modeSequence.push_back(ModeNumber::STANCE);
    swingHeightSequence_.push_back(-1.0);
    phaseAdaptations_.push_back({});
  }

  // tile the mode sequence template from startTime+phaseTransitionStanceTime to finalTime.
  tileModeSequenceTemplate(startTime + phaseTransitionStanceTime, finalTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ModeSchedule GaitSchedule::getModeSchedule(scalar_t lowerBoundTime, scalar_t upperBoundTime) {
  auto& eventTimes = modeSchedule_.eventTimes;
  auto& modeSequence = modeSchedule_.modeSequence;
  const size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(), lowerBoundTime) - eventTimes.begin();

  if (index > 0) {
    // delete the old logic from index and set the default start phase to stance
    eventTimes.erase(eventTimes.begin(),
                     eventTimes.begin() + index - 1);  // keep the one before the last to make it stance
    modeSequence.erase(modeSequence.begin(), modeSequence.begin() + index - 1);
    swingHeightSequence_.erase(swingHeightSequence_.begin(), swingHeightSequence_.begin() + index - 1);
    phaseAdaptations_.erase(phaseAdaptations_.begin(), phaseAdaptations_.begin() + index - 1);

    // set the default initial phase
    modeSequence.front() = ModeNumber::STANCE;
  }

  // Start tiling at time
  const auto tilingStartTime = eventTimes.empty() ? upperBoundTime : eventTimes.back();

  // delete the last default stance phase
  eventTimes.erase(eventTimes.end() - 1, eventTimes.end());
  modeSequence.erase(modeSequence.end() - 1, modeSequence.end());
  swingHeightSequence_.pop_back();
  phaseAdaptations_.pop_back();

  // tile the template logic
  tileModeSequenceTemplate(tilingStartTime, upperBoundTime);
  return modeSchedule_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitSchedule::tileModeSequenceTemplate(scalar_t startTime, scalar_t finalTime) {
  auto& eventTimes = modeSchedule_.eventTimes;
  auto& modeSequence = modeSchedule_.modeSequence;
  const auto& templateTimes = modeSequenceTemplate_.switchingTimes;
  const auto& templateModeSequence = modeSequenceTemplate_.modeSequence;
  const size_t numTemplateSubsystems = modeSequenceTemplate_.modeSequence.size();

  // If no template subsystem is defined, the last subsystem should continue for ever
  if (numTemplateSubsystems == 0) {
    return;
  }

  if (!eventTimes.empty() && startTime <= eventTimes.back()) {
    throw std::runtime_error("The initial time for template-tiling is not greater than the last event time.");
  }

  // add a initial time
  eventTimes.push_back(startTime);

  // concatenate from index
  while (eventTimes.back() < finalTime) {
    for (size_t i = 0; i < templateModeSequence.size(); i++) {
      modeSequence.push_back(templateModeSequence[i]);
      swingHeightSequence_.push_back(modeSequenceTemplate_.swingHeight);
      phaseAdaptations_.push_back({adaptation_.periodScale, adaptation_.swingHeightOffset, adaptation_.apexShift});
      scalar_t deltaTime = adaptation_.periodScale * (templateTimes[i + 1] - templateTimes[i]);
      eventTimes.push_back(eventTimes.back() + deltaTime);
    }  // end of i loop
  }  // end of while loop

  // default final phase
  modeSequence.push_back(ModeNumber::STANCE);
  swingHeightSequence_.push_back(-1.0);
  phaseAdaptations_.push_back({});
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitSchedule::setAdaptation(const GaitAdaptation& adaptation, scalar_t time) {
  if (adaptation.periodScale <= 0.0) {
    throw std::invalid_argument("[GaitSchedule] periodScale must be positive");
  }
  auto& eventTimes = modeSchedule_.eventTimes;
  // Mode i is active over [eventTimes[i-1], eventTimes[i]); the one at `time` and its end stay as
  // they are, every later phase is retimed from its template duration.
  const size_t current = std::upper_bound(eventTimes.begin(), eventTimes.end(), time) - eventTimes.begin();
  const scalar_t phaseStart = current > 0 ? eventTimes[current - 1] : -std::numeric_limits<scalar_t>::infinity();
  if (phaseStart == adaptedPhaseStart_) {
    return;
  }
  adaptedPhaseStart_ = phaseStart;
  if (adaptation == adaptation_) {
    return;
  }
  adaptation_ = adaptation;
  scalar_t previousOldEnd = current < eventTimes.size() ? eventTimes[current] : 0.0;
  for (size_t i = current + 1; i < phaseAdaptations_.size(); ++i) {
    auto& phase = phaseAdaptations_[i];
    if (i < eventTimes.size()) {
      const scalar_t oldDuration = eventTimes[i] - previousOldEnd;
      previousOldEnd = eventTimes[i];
      const scalar_t duration = phase.periodScale > 0.0 ? oldDuration / phase.periodScale * adaptation.periodScale : oldDuration;
      eventTimes[i] = eventTimes[i - 1] + duration;
    }
    if (phase.periodScale > 0.0) {
      phase.periodScale = adaptation.periodScale;
    }
    phase.swingHeightOffset = adaptation.swingHeightOffset;
    phase.apexShift = adaptation.apexShift;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitSchedule::reset(scalar_t time) {
  modeSchedule_ = ModeSchedule({time}, {ModeNumber::STANCE, ModeNumber::STANCE});
  swingHeightSequence_.assign(modeSchedule_.modeSequence.size(), -1.0);
  phaseAdaptations_.assign(modeSchedule_.modeSequence.size(), {});
  modeSequenceTemplate_ = defaultModeSequenceTemplate_;
  adaptation_ = GaitAdaptation{};
  adaptedPhaseStart_ = std::numeric_limits<scalar_t>::quiet_NaN();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::shared_ptr<GaitSchedule> GaitSchedule::createGaitSchedule(const ModeSchedule& initModeSchedule,
                                                               const ModeSequenceTemplate& defaultModeSequenceTemplate,
                                                               const ModelSettings& modelSettings,
                                                               bool verbose) {
  const auto defaultGait = [&] {
    Gait gait{};
    gait.duration = defaultModeSequenceTemplate.switchingTimes.back();
    // Events: from time -> phase
    std::for_each(defaultModeSequenceTemplate.switchingTimes.begin() + 1, defaultModeSequenceTemplate.switchingTimes.end() - 1,
                  [&](double eventTime) { gait.eventPhases.push_back(eventTime / gait.duration); });
    // Modes:
    gait.modeSequence = defaultModeSequenceTemplate.modeSequence;
    return gait;
  }();

  // display
  if (verbose) {
    std::cerr << "\n#### Modes Schedule: ";
    std::cerr << "\n#### =============================================================================\n";
    std::cerr << "Initial Modes Schedule: \n" << initModeSchedule;
    std::cerr << "Default Modes Sequence Template: \n" << defaultModeSequenceTemplate;
    std::cerr << "#### =============================================================================\n";
  }

  return std::make_shared<GaitSchedule>(initModeSchedule, defaultModeSequenceTemplate, modelSettings.phaseTransitionStanceTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void GaitSchedule::updateModeSchedule(const ModeSchedule& modeSchedule) {
  modeSchedule_ = modeSchedule;
  swingHeightSequence_.assign(modeSchedule_.modeSequence.size(), -1.0);
  phaseAdaptations_.assign(modeSchedule_.modeSequence.size(), {});
}

}  // namespace ocs2::humanoid
