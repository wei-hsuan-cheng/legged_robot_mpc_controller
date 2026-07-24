/******************************************************************************
Copyright (c) 2026. All rights reserved.

Online foothold planner over a known (ground-truth) piecewise-planar terrain.

Phase-1 of the perceptive-locomotion roadmap (T-RO 2023, Grandia et al.),
without perception: the terrain model is built from semantic ground-truth
geometry (e.g. the staircase description) instead of segmented elevation maps.

Each solver cycle (called from SwitchedModelReferenceManager::modifyReferences)
the planner walks through the upcoming swing phases of the fetched mode
schedule and, per swing:
  1. computes a nominal foothold below the hip at touch-down, from the desired
     base trajectory (Raibert-style heuristic with a capture-point velocity
     feedback on the first upcoming step, paper eq. (12)),
  2. projects it onto the best terrain region (closest-candidate selection with
     margins and a step-height rejection, simplified paper eq. (13)).
The resulting footstep timeline provides the swing-foot xy references (tracked
by the task-space foot cost), the per-phase lift-off / touch-down heights for
the SwingTrajectoryPlanner, and the support height used to terrain-adapt the
base height reference. Pelvis orientation stays flat (zero pitch/roll): stair
treads are horizontal and a humanoid keeps its torso upright.
******************************************************************************/

#pragma once

#include <vector>

#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_core/reference/TargetTrajectories.h>

#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/common/Types.h"
#include "humanoid_common_mpc/reference_manager/StairClimbingPlan.h"

namespace ocs2::humanoid {

/// One horizontal rectangular planar region (a stair tread), in world frame.
struct PlanarRegion {
  vector2_t center{vector2_t::Zero()};  ///< world xy of the rectangle center
  scalar_t yaw{0.0};                    ///< rotation of the rectangle about +z
  scalar_t halfLengthX{0.15};           ///< half size along the local x (depth/2)
  scalar_t halfLengthY{0.60};           ///< half size along the local y (width/2)
  scalar_t height{0.0};                 ///< world z of the surface
};

/// Piecewise-horizontal ground-truth terrain: a flat ground plane plus elevated
/// rectangular regions. The ground is assumed unavailable inside the footprint
/// of the elevated structure (feet cannot stand under the stairs).
struct GroundTruthTerrainModel {
  scalar_t groundHeight{0.0};
  std::vector<PlanarRegion> regions;
  // Footprint of the elevated structure on the ground plane (world frame,
  // same yaw convention as the regions); ground candidates are pushed out of it.
  vector2_t footprintCenter{vector2_t::Zero()};
  scalar_t footprintYaw{0.0};
  scalar_t footprintHalfLengthX{0.0};
  scalar_t footprintHalfLengthY{0.0};

  /// Surface height at a world xy position (highest region containing it, else ground).
  scalar_t heightAt(const vector2_t& positionWorld) const;
};

/// Builds the terrain model from the staircase ground truth in StairClimbingConfig.
GroundTruthTerrainModel buildTerrainModelFromStairs(const StairClimbingConfig& stairsConfig);

struct TerrainFootholdPlannerSettings {
  scalar_t hipLateralOffset{0.115};   ///< |y| of the nominal foothold from the base centerline
  scalar_t footMarginX{0.10};         ///< region shrink along local x (half foot length + margin)
  scalar_t footMarginY{0.05};         ///< region shrink along local y (half foot width + margin)
  scalar_t maxStepHeight{0.18};       ///< candidate regions further than this in z from the previous foothold are rejected
  scalar_t capturePointFeedbackGain{0.27};  ///< sqrt(h/g) gain on the (measured - desired) base velocity, first step only
  scalar_t maxCapturePointOffset{0.10};     ///< clamp on the capture-point foothold offset [m]
  scalar_t maxBaseHeightAboveSupport{0.72};  ///< cap of the commanded pelvis height above the mean support
  scalar_t engageDistance{0.5};  ///< terrain adaptation engages within this distance of the staircase footprint [m]
  scalar_t maxBaseLead{0.30};  ///< clamp of the base reference's horizontal lead over the planned support midpoint [m]
  scalar_t footholdTrackingWeight{250.0};    ///< task-space foot cost xy weight during swing
  scalar_t swingReferenceArrivalFraction{0.75};  ///< xy reference reaches the foothold at this swing fraction
};

class TerrainFootholdPlanner {
 public:
  TerrainFootholdPlanner(GroundTruthTerrainModel terrainModel, TerrainFootholdPlannerSettings settings)
      : terrainModel_(std::move(terrainModel)), settings_(std::move(settings)) {}

  /**
   * Re-plans the footstep timeline for the current MPC window. Called on the
   * solver thread before the solve; the query methods below are read-only
   * during the solve.
   *
   * @param modeSchedule        : the fetched mode schedule covering the window.
   * @param targetTrajectories  : desired base trajectory (velocity-command targets).
   * @param currentFeetPositions: measured feet positions (world, from FK).
   * @param initState           : current MPC state (for the velocity feedback).
   * @param initTime            : current solver time.
   * @param mpcRobotModel       : state accessors.
   */
  void update(const ModeSchedule& modeSchedule,
              const TargetTrajectories& targetTrajectories,
              const feet_array_t<vector3_t>& currentFeetPositions,
              const vector_t& initState,
              scalar_t initTime,
              const MpcRobotModelBase<scalar_t>& mpcRobotModel);

  /// Planned support height of a foot at a given time (changes at touch-downs).
  scalar_t getPlannedFootHeight(size_t foot, scalar_t time) const;

  /// Mean support height over both feet at a given time (for the base height reference).
  scalar_t getSupportHeight(scalar_t time) const {
    return 0.5 * (getPlannedFootHeight(0, time) + getPlannedFootHeight(1, time));
  }

  /// Swing-foot xy position reference; true when `time` is inside a planned swing.
  bool getSwingFootReference(size_t foot, scalar_t time, vector3_t& positionReference) const;

  /// Planned stance position of a foot at a given time (changes at touch-downs).
  vector3_t getPlannedFootPosition(size_t foot, scalar_t time) const;

  /// Planned support midpoint (mean of the feet positions) at a given time.
  vector3_t getSupportMidpoint(scalar_t time) const {
    return 0.5 * (getPlannedFootPosition(0, time) + getPlannedFootPosition(1, time));
  }

  scalar_t getFootholdTrackingWeight() const { return settings_.footholdTrackingWeight; }
  scalar_t getMaxBaseHeightAboveSupport() const { return settings_.maxBaseHeightAboveSupport; }
  scalar_t getMaxBaseLead() const { return settings_.maxBaseLead; }

  /// True when a world xy is within the engage distance of the staircase
  /// footprint. Away from the stairs the terrain adaptation (base override,
  /// momentum gating, foothold tracking) is disabled so the robot walks
  /// normally on flat ground; it engages only near/on the stairs.
  bool isNearStairs(const vector2_t& positionWorld) const;

  /// Per-foot, per-phase lift-off / touch-down heights for the SwingTrajectoryPlanner.
  void getHeightSequences(const ModeSchedule& modeSchedule,
                          feet_array_t<scalar_array_t>& liftOffHeightSequence,
                          feet_array_t<scalar_array_t>& touchDownHeightSequence) const;

  /**
   * Absolute base (pelvis) reference built from the planned footholds: knots at
   * initTime and every upcoming touch-down, base = mid-feet xy and
   * mean-support+crouch z, terrain yaw, zero pitch/roll. Unlike a
   * velocity-relative reference (which lags with the measured base and stalls
   * the CoM at the riser), this is an absolute forward+up trajectory in sync
   * with the feet, so the CoM commits onto each step. Mirrors the working fixed
   * StairClimbingPlan base generation, but over the online-replanned footholds.
   */
  TargetTrajectories getBaseTargetTrajectories(scalar_t initTime,
                                               scalar_t finalTime,
                                               const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                               const vector_t& defaultJointState) const;

  const feet_array_t<std::vector<PlannedFootstep>>& getFootsteps() const { return footsteps_; }

 private:
  /// Projects a nominal foothold onto the best terrain candidate (region or ground).
  vector3_t projectFoothold(const vector2_t& nominalXY, scalar_t previousFootholdHeight) const;

  GroundTruthTerrainModel terrainModel_;
  TerrainFootholdPlannerSettings settings_;

  feet_array_t<std::vector<PlannedFootstep>> footsteps_;
  feet_array_t<vector3_t> initialFootPosition_;  ///< foot position before the first planned touch-down
};

}  // namespace ocs2::humanoid
