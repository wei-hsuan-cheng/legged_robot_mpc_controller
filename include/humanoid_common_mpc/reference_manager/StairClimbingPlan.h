/******************************************************************************
Copyright (c) 2026. All rights reserved.

Fixed-sequence stair climbing plan for the humanoid MPC.

The plan is compiled once, at trigger time, from the ground-truth staircase
geometry and gait timing parameters. It provides:
  - a non-periodic mode sequence (gait) covering the whole climb,
  - per-foot planned foothold positions (world frame) for every swing,
  - per-foot piecewise-constant support heights consumed by the
    SwingTrajectoryPlanner lift-off / touch-down height sequences,
  - a base (pelvis) reference trajectory with zero pitch and roll.

Frame convention: the staircase frame has its origin at the center of the 0th
step (ground level), x pointing up the stairs (ascent direction, given by the
stairs yaw about world +z) and z up. Steps 1..N are solid boxes; step i spans
[x_i, x_i + D_i] in local x with tread top at z_i = sum(H_1..H_i).
******************************************************************************/

#pragma once

#include <memory>
#include <vector>

#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_core/reference/TargetTrajectories.h>

#include "humanoid_common_mpc/command/TargetTrajectoriesCalculatorBase.h"
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/common/Types.h"
#include "humanoid_common_mpc/gait/ModeSequenceTemplate.h"

namespace ocs2::humanoid {

/// Semantic-level stair climbing parameters (loaded from stair_climbing.yaml).
struct StairClimbingConfig {
  // ----- staircase ground truth (world frame) -----
  vector3_t stairsBasePosition{vector3_t::Zero()};  ///< center of the 0th step, ground level
  scalar_t stairsYaw{0.0};                          ///< ascent direction about world +z
  scalar_t startOffset{0.0};      ///< local +x offset from the 0th-step center to the first riser
  std::vector<scalar_t> stepHeights;  ///< riser height per step H_i
  std::vector<scalar_t> stepDepths;   ///< tread depth per step D_i

  // ----- gait timing -----
  scalar_t initialStanceDuration{2.0};  ///< settle time before the first lift-off
  scalar_t swingDuration{0.8};          ///< single-support (swing) duration
  scalar_t stanceDuration{0.6};         ///< double-support duration between swings
  scalar_t finalStanceDuration{1.0};    ///< settle time appended after the last touch-down
  bool leftFootFirst{true};             ///< which foot leads
  /// true: step-to gait, both feet land on every tread (2N climb swings).
  /// false: one-tread-one-leg gait, the feet alternate over consecutive treads
  /// (N+1 climb swings; each swing spans two treads, ~2x stride and riser).
  bool bothFeetPerTread{true};

  // ----- foothold generation -----
  scalar_t lateralOffset{0.12};   ///< |y| of each foot from the staircase centerline
  scalar_t treadInset{0.0};       ///< foothold x offset from the tread center
  scalar_t approachStride{0.25};  ///< flat-ground stride length used to approach the stairs
  scalar_t approachStandoff{0.15};  ///< distance the feet stop short of the first riser

  // ----- base reference -----
  scalar_t baseHeightAboveSupport{0.78};  ///< pelvis height above the mean support height

  // ----- swing foot xy tracking -----
  scalar_t footholdTrackingWeight{50.0};  ///< task-space foot cost weight on xy position
};

/// One planned swing of a single foot.
struct PlannedFootstep {
  scalar_t liftOffTime{0.0};
  scalar_t touchDownTime{0.0};
  vector3_t liftOffPosition{vector3_t::Zero()};   ///< world frame
  vector3_t touchDownPosition{vector3_t::Zero()};  ///< world frame
};

/**
 * Immutable, time-anchored stair climbing motion plan.
 * Compiled on the solver thread when the climb is triggered; safe for
 * concurrent read access afterwards.
 */
class StairClimbingPlan {
 public:
  /**
   * @param config      : semantic parameters (staircase GT + gait timing).
   * @param startTime   : absolute time the plan is anchored at (initial stance begins here).
   * @param initialBasePose : [x y z yaw pitch roll] of the base at trigger time; the
   *                          approach footholds start below this pose.
   * @param mpcRobotModel  : provides state/input dimensions and layouts.
   * @param referenceConfig: provides the default joint state for the base reference.
   */
  StairClimbingPlan(const StairClimbingConfig& config,
                    scalar_t startTime,
                    const vector6_t& initialBasePose,
                    const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                    const ReferenceConfig& referenceConfig);

  scalar_t getStartTime() const { return startTime_; }
  scalar_t getFinalTime() const { return finalTime_; }
  bool isActiveAt(scalar_t time) const { return time >= startTime_ && time <= finalTime_; }

  /// Relative-time mode sequence covering the whole climb (insert once into the GaitSchedule).
  const ModeSequenceTemplate& getModeSequenceTemplate() const { return modeSequenceTemplate_; }

  /// Base (pelvis) reference over the whole climb; zero pitch and roll.
  const TargetTrajectories& getBaseTargetTrajectories() const { return baseTargetTrajectories_; }

  /// Planned support height of a foot at a given time (changes at touch-down events).
  scalar_t getPlannedFootHeight(size_t foot, scalar_t time) const;

  /**
   * Swing-foot position reference (world frame), linearly interpolated from the
   * lift-off foothold to the touch-down foothold over the swing phase.
   * @return true when `time` falls inside a planned swing of `foot`.
   */
  bool getSwingFootReference(size_t foot, scalar_t time, vector3_t& positionReference) const;

  /**
   * Builds the per-foot, per-phase lift-off / touch-down height sequences for an
   * externally fetched mode schedule window (SwingTrajectoryPlanner input).
   */
  void getHeightSequences(const ModeSchedule& modeSchedule,
                          feet_array_t<scalar_array_t>& liftOffHeightSequence,
                          feet_array_t<scalar_array_t>& touchDownHeightSequence) const;

  scalar_t getFootholdTrackingWeight() const { return config_.footholdTrackingWeight; }

  const std::vector<PlannedFootstep>& getFootsteps(size_t foot) const { return footsteps_[foot]; }

 private:
  StairClimbingConfig config_;
  scalar_t startTime_{0.0};
  scalar_t finalTime_{0.0};

  ModeSequenceTemplate modeSequenceTemplate_{{0.0, 1.0}, {ModeNumber::STANCE}};
  feet_array_t<std::vector<PlannedFootstep>> footsteps_;
  TargetTrajectories baseTargetTrajectories_;
};

}  // namespace ocs2::humanoid
