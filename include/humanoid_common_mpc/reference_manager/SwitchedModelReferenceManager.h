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

#include <ocs2_core/thread_support/BufferedValue.h>
#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/gait/GaitSchedule.h"
#include "humanoid_common_mpc/gait/MotionPhaseDefinition.h"
#include "humanoid_common_mpc/reference_manager/StairClimbingPlan.h"
#include "humanoid_common_mpc/reference_manager/TerrainFootholdPlanner.h"
#include "humanoid_common_mpc/swing_foot_planner/SwingTrajectoryPlanner.h"

namespace ocs2::humanoid {

/**
 * Manages the ModeSchedule and the TargetTrajectories for switched model.
 */
class SwitchedModelReferenceManager : public ReferenceManager {
 public:
  enum class BaseTrackingMode { RelativeTwist, AbsolutePose };

  SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                const PinocchioInterface& pinocchioInterface,
                                const MpcRobotModelBase<scalar_t>& mpcRobotModel);

  ~SwitchedModelReferenceManager() override = default;

  /** Disable copy / move */
  SwitchedModelReferenceManager& operator=(const SwitchedModelReferenceManager&) = delete;
  SwitchedModelReferenceManager(const SwitchedModelReferenceManager&) = delete;
  SwitchedModelReferenceManager& operator=(SwitchedModelReferenceManager&&) = delete;
  SwitchedModelReferenceManager(SwitchedModelReferenceManager&&) = delete;

  /**
   * Capture-point foot placement for flat ground.
   *
   * On flat ground there is no foothold reference at all: the stair plan and the
   * terrain planner are both inactive, getSwingFootholdReference() returns false,
   * and the foot task-space cost has zero position weight. The landing xy is then
   * whatever the optimizer happens to produce. That is fine while the robot is
   * upright, but it leaves nothing to arrest a lateral fall - and lateral walking
   * dynamics are an inverted pendulum whose only real authority IS where the next
   * foot lands. Measured behaviour matches: step width stayed pinned near 0.18 m
   * while roll grew step over step until the robot went over.
   *
   * This closes that loop the standard way. The swing foot is aimed at the
   * capture point (CoM plus CoM velocity over omega), offset laterally by half
   * the nominal step width so the feet stay apart, with the correction clamped so
   * a transient cannot command an unreachable step.
   */
  struct CaptureFootPlacementSettings {
    bool enabled = false;
    scalar_t gain = 0.6;             //!< scales the capture-point correction; 0 reproduces the nominal foothold
    scalar_t stepWidth = 0.18;       //!< nominal lateral spacing between the feet [m]
    scalar_t maxAdjustment = 0.20;   //!< bound on the correction away from nominal [m]
    //! xy foothold tracking weight handed to the foot cost. The dominant knob,
    //! and it wants to be LARGE: on the vx ladder 30 -> 48.5 s, 100 -> 57.5,
    //! 400 -> 65.0, 1600 -> 78.6, 3200 -> 79.2 (saturating). Too weak and the
    //! optimizer simply ignores the foothold.
    scalar_t trackingWeight = 1600.0;
    //! How far ahead to propagate the capture point about the stance foot, in
    //! seconds, capped at the time remaining until touchdown. The DCM diverges as
    //! exp(omega*t), so in principle projecting to touchdown makes the placement
    //! deadbeat. In practice omega ~ 3.7 turns a 0.4 s projection into a 4.5x
    //! amplification that saturates maxAdjustment on every step and destroys the
    //! feedback it was meant to sharpen - measured worse than no projection.
    //! Default 0 is the instantaneous capture point, which measured best; raise
    //! it together with maxAdjustment if you want to explore the deadbeat end.
    scalar_t projectionHorizon = 0.0;
    //! Feedforward step length, as a fraction of (velocity x step period).
    //!
    //! The capture point alone is the DEADBEAT STOPPING placement: put the foot
    //! there and the CoM comes to rest. Steady walking needs the opposite - the
    //! foot must land ahead of the base by about v * T_step / 2 so the body can
    //! keep passing over it. With omega ~ 3.7 the capture term supplies only
    //! 0.27 * v against the ~0.43 * v that a 0.85 s step needs, leaving the robot
    //! a few centimetres short on every step and slowly pitching forward - which
    //! is worse the faster it walks.
    //!
    //! MEASURED NEUTRAL-TO-HARMFUL here, so it defaults to 0: the nominal already
    //! advances by v * timeToTouchdown (see baseAtTouchdown), which covers most of
    //! the same travel, and adding this term on top double-counts it. On the vx
    //! ladder: 0.0 -> 56.3 s, 0.5 -> 56.1 s, 0.8 -> 55.5 s, 1.2 -> 47.1 s. Kept as
    //! a knob because the right split between the two depends on the gait period.
    scalar_t stepLengthGain = 0.0;
  };

  void setCaptureFootPlacementSettings(const CaptureFootPlacementSettings& settings) {
    captureFootPlacement_ = settings;
  }
  const CaptureFootPlacementSettings& getCaptureFootPlacementSettings() const { return captureFootPlacement_; }

  contact_flag_t getContactFlags(scalar_t time) const;

  bool isInStancePhase(scalar_t time) const { return (getContactFlags(time)[0] && getContactFlags(time)[1]); }

  bool isInContact(scalar_t time, size_t contactIndex) const { return getContactFlags(time)[contactIndex]; };

  void setArmSwingReferenceActive(bool armSwingReferenceActive) { armSwingReferenceActive_ = armSwingReferenceActive; }

  /**
   * Selects how the base tracking cost interprets the target trajectory.
   * RelativeTwist ignores the unobservable world x/y/yaw pose error and tracks
   * horizontal motion in the target heading frame. AbsolutePose retains full
   * world-frame pose tracking for explicit base-pose commands.
   */
  void setBaseTrackingMode(BaseTrackingMode mode) { baseTrackingMode_.setBuffer(mode); }
  BaseTrackingMode getBaseTrackingMode() const { return baseTrackingMode_.get(); }
  bool usesRelativeBaseTwistTracking() const {
    return getBaseTrackingMode() == BaseTrackingMode::RelativeTwist;
  }

  /**
   * External joint-target channel (command_type "joint", mirroring the
   * mpc_controllers reference-manager targets). When a non-empty trajectory is
   * buffered, JointTrackingCost tracks it instead of the internal arm-swing
   * reference; an empty trajectory reverts to the internal reference. States are
   * sized and ordered like the tracked-joint subset. Buffered: setters are
   * thread-safe, consumers read the value swapped in before the solver run.
   * A frame-relation target channel can be added alongside later.
   */
  void setExternalJointTargets(TargetTrajectories jointTargets) { externalJointTargets_.setBuffer(std::move(jointTargets)); }
  const TargetTrajectories& getExternalJointTargets() const { return externalJointTargets_.get(); }
  bool hasExternalJointTargets() const { return !externalJointTargets_.get().empty(); }

  /**
   * External frame-relation target channel (command_type "frame_relation").
   * Convention (matching mpc_controllers): sourceFrames[i] is the reference
   * frame the pose is expressed in (a robot frame such as "pelvis", or a global
   * frame), targetFrames[i] is the tracked leaf frame (e.g. a hand). Each entry
   * carries a pose trajectory (states are [position(3), quaternion x y z w] of
   * target in source) and an optional 6-vector weight [position xyz,
   * orientation xyz]; an empty weight keeps the configured defaults. Same
   * buffering semantics as the joint channel.
   */
  struct FrameRelationTargets {
    std::vector<std::string> sourceFrames;
    std::vector<std::string> targetFrames;
    std::vector<TargetTrajectories> targets;
    std::vector<vector_t> weights;
    bool empty() const { return sourceFrames.empty(); }
  };

  void setExternalFrameRelationTargets(FrameRelationTargets frameRelationTargets) {
    externalFrameRelationTargets_.setBuffer(std::move(frameRelationTargets));
  }
  const FrameRelationTargets& getExternalFrameRelationTargets() const { return externalFrameRelationTargets_.get(); }
  bool hasExternalFrameRelationTargets() const { return !externalFrameRelationTargets_.get().empty(); }

  /**
   * Fixed-sequence stair climbing plan channel. When a plan is set, the swing
   * trajectory planner is fed the plan's per-phase lift-off / touch-down height
   * sequences (instead of flat ground) and the swing-foot cost tracks the
   * planned footholds. Buffered: the setter is thread-safe, consumers read the
   * value swapped in before the solver run. Set nullptr to clear.
   */
  void setStairClimbingPlan(std::shared_ptr<const StairClimbingPlan> plan) { stairClimbingPlan_.setBuffer(std::move(plan)); }
  const std::shared_ptr<const StairClimbingPlan>& getStairClimbingPlan() const { return stairClimbingPlan_.get(); }

  /**
   * Online terrain-aware foothold planning channel (terrain_walk mode).
   * The planner is installed once at configure time; the activation flag is
   * buffered (thread-safe). When active, modifyReferences re-plans the
   * footsteps each cycle from the fetched mode schedule and the desired base
   * trajectory, feeds the resulting height sequences to the swing planner, and
   * terrain-adapts the base height reference.
   */
  void setTerrainFootholdPlanner(std::shared_ptr<TerrainFootholdPlanner> planner) {
    terrainFootholdPlannerPtr_ = std::move(planner);
  }
  void setTerrainWalkActive(bool active) { terrainWalkActive_.setBuffer(active); }
  bool isTerrainWalkActive() const { return terrainWalkActive_.get() && terrainFootholdPlannerPtr_ != nullptr; }

  /**
   * Unified swing-foot foothold reference used by the task-space foot cost:
   * dispatches to the fixed stair climbing plan or the online terrain planner.
   * @return true when a foothold reference is active for `foot` at `time`.
   */
  bool getSwingFootholdReference(size_t foot, scalar_t time, vector3_t& positionReference, scalar_t& trackingWeight) const;

  const std::shared_ptr<GaitSchedule>& getGaitSchedule() const { return gaitSchedulePtr_; }

  const std::shared_ptr<SwingTrajectoryPlanner>& getSwingTrajectoryPlanner() const { return swingTrajectoryPtr_; }

  scalar_t getPhaseVariable(scalar_t time) const;

  vector_t getDesiredState(const TargetTrajectories& targetTrajectories, const vector_t& state, scalar_t time) const;

 protected:
  virtual void modifyReferences(scalar_t initTime,
                                scalar_t finalTime,
                                const vector_t& initState,
                                size_t initMode,
                                TargetTrajectories& targetTrajectories,
                                ModeSchedule& modeSchedule) override;

  // Adjusts the height of the target trajectories to current terrain height and returns that height.
  scalar_t adaptToCurrentGroundHeight(TargetTrajectories& targetTrajectories, const vector_t& initState, size_t initMode);
  scalar_t previousGroundHeightEstimate_{0.0};

  PinocchioInterface pinocchioInterface_;
  const MpcRobotModelBase<scalar_t>* mpcRobotModelPtr_;
  ModeSchedule modeSchedule_;

  bool armSwingReferenceActive_{false};

  BufferedValue<TargetTrajectories> externalJointTargets_{TargetTrajectories()};
  BufferedValue<FrameRelationTargets> externalFrameRelationTargets_{FrameRelationTargets()};
  BufferedValue<std::shared_ptr<const StairClimbingPlan>> stairClimbingPlan_{nullptr};
  BufferedValue<BaseTrackingMode> baseTrackingMode_{BaseTrackingMode::RelativeTwist};

  // Terrain-aware walking: planner mutated only in modifyReferences (solver
  // thread, pre-solve); read-only during the solve, same pattern as the swing planner.
  std::shared_ptr<TerrainFootholdPlanner> terrainFootholdPlannerPtr_;
  BufferedValue<bool> terrainWalkActive_{false};

  /// Measured feet (contact frame) world positions from FK of the current state.
  feet_array_t<vector3_t> computeFeetPositions(const vector_t& initState);

  /// Recompute the capture-point footholds from the current state. Solver thread
  /// only, called from modifyReferences() before the solve reads them.
  void updateCaptureFootholds(scalar_t initTime, const vector_t& initState);

  CaptureFootPlacementSettings captureFootPlacement_;
  feet_array_t<vector3_t> captureFoothold_{vector3_t::Zero(), vector3_t::Zero()};
  feet_array_t<bool> captureFootholdValid_{false, false};

  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
  std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr_;
};

}  // namespace ocs2::humanoid
