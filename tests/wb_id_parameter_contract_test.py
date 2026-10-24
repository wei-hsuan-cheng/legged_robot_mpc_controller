#!/usr/bin/env python3
"""Static WB-ID YAML/schema regression; no ROS build or simulator required."""

from pathlib import Path
import re
import yaml


ROOT = Path(__file__).resolve().parents[1]
NAME = "humanoid_whole_body_inverse_dynamics_mpc_controller"
WB_ID_LAYOUT_MIGRATIONS = (
    ("model.torqueConstraintNodes", "constraints.inverseDynamics.torqueConstraintNodes"),
    ("model.phaseTransitionStanceTime", "gait.phaseTransitionStanceTime"),
    ("model.useTerrainHeightEstimate", "reference.groundHeight.useEstimate"),
    ("model.maxTerrainHeightStep", "reference.groundHeight.maxStepPerSolve"),
    ("model.verboseCppAd", "codegen.verbose"),
    ("model.recompileLibrariesCppAd", "codegen.recompileLibraries"),
    ("model.armJointNames", "reference.armSwing.jointNames"),
    ("constraints.contact", "model.contactGeometry"),
    ("sqp", "solver.sqp"),
    ("rollout", "solver.rollout"),
    ("constraints.jointLimits", "constraints.jointPositionLimits"),
    ("constraints.footConstraint", "constraints.footMotionEquality"),
    ("constraints.zeroWrench", "constraints.swingFootZeroWrench"),
    ("constraints.footCollision", "constraints.legSelfCollision"),
)


def at_path(values, path):
    for part in path.split("."):
        values = values[part]
    return values


class UniqueLoader(yaml.SafeLoader):
    pass


def unique_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        assert key not in result, f"Duplicate YAML key: {key}"
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)


def check_keys(values, schema, prefix=""):
    for key, value in values.items():
        if not prefix and key == "update_rate":
            continue  # ros2_control controller manager parameter
        entry = schema.get(key)
        if entry is None:
            dynamic = [v for k, v in schema.items() if k.startswith("__map_")]
            assert len(dynamic) == 1, f"Undeclared parameter: {prefix}{key}"
            entry = dynamic[0]
        if isinstance(value, dict):
            check_keys(value, entry, prefix + key + ".")
        else:
            assert "type" in entry, f"Not a parameter leaf: {prefix}{key}"
            if entry["type"] == "bool":
                assert type(value) is bool, f"Expected a YAML boolean: {prefix}{key}"
            size = entry.get("validation", {}).get("fixed_size<>")
            if size:
                assert len(value) == size[0], f"Wrong array length: {prefix}{key}"


def main():
    config = yaml.load((ROOT / "config/g1/ros2_controllers.yaml").read_text(), Loader=UniqueLoader)
    schema = yaml.load((ROOT / f"src/humanoid_whole_body_inverse_dynamics_mpc/{NAME}_parameter.yaml").read_text(), Loader=UniqueLoader)[NAME]
    params = config[NAME]["ros__parameters"]
    check_keys(params, schema)
    assert "wholeBody" not in params and "wholeBody" not in schema
    assert "wbc" not in schema
    costs = params["ocs2"]["costs"]
    count = len(params["robot"]["jointNames"])
    for stage in ("q", "qFinal"):
        assert set(costs[stage]) == {"basePose", "baseVelocity", "jointPositions", "jointVelocities"}
        for key in ("jointPositions", "jointVelocities"):
            assert len(costs[stage][key]) == count
    assert set(costs["r"]) == {"contactWrenches", "baseAccelerations", "jointAccelerations", "jointTorques"}
    for key in ("jointAccelerations", "jointTorques"):
        assert len(costs["r"][key]) == count
    for stage in ("q", "qFinal", "r"):
        assert all(v >= 0 for values in costs[stage].values() for v in values)
    assert "captureFootPlacement" not in costs and "icpErrorWeight" not in costs
    assert "centroidalModelType" not in params["ocs2"]["model"]
    # WB-ID foot stabilization belongs to constraints, not rigid-body model settings.
    assert "footConstraint" not in params["ocs2"]["model"]
    assert "footConstraint" not in schema["ocs2"]["model"]
    foot = params["ocs2"]["constraints"]["footMotionEquality"]
    foot_schema = schema["ocs2"]["constraints"]["footMotionEquality"]
    foot_gains = {
        "positionErrorGain_z", "orientationErrorGain", "linearVelocityErrorGain_z",
        "linearVelocityErrorGain_xy", "angularVelocityErrorGain",
        "linearAccelerationErrorGain_z", "linearAccelerationErrorGain_xy", "angularAccelerationErrorGain",
    }
    assert set(foot) == set(foot_schema) == foot_gains | {"activate"}
    assert all(value >= 0 for value in foot.values())
    for name in ("linearAccelerationErrorGain_z", "linearAccelerationErrorGain_xy", "angularAccelerationErrorGain"):
        assert foot[name] > 0, f"constraints.footMotionEquality.{name} must retain input dependence"
    assert "useInertiaWeightedAngularMomentum" not in params["ocs2"]["reference"]
    for gain in ("kp", "kd"):
        values = params["control"]["jointEffortPd"][gain]
        assert len(values) in (0, count), f"control.jointEffortPd.{gain} needs one entry per joint (or none)"
        assert all(v >= 0 for v in values)
    gait = params["ocs2"]["gait"]
    names = gait["templates"]["names"]
    for name in names:
        assert name in gait["templates"], f"Template not defined: {name}"
        times, modes = gait["templates"][name]["switchingTimes"], gait["templates"][name]["modeSequence"]
        assert len(times) == len(modes) + 1, f"Template {name}: N modes need N+1 switching times"
        assert times == sorted(times), f"Template {name}: switching times not increasing"
    ladder = gait["transitions"]["ladder"]
    assert ladder[0] == "stance" and set(ladder) <= set(names), "Ladder must start at stance and use defined templates"
    for rung in ladder:
        bands = gait["transitions"][rung]
        assert bands["linearVelocity"][0] <= bands["linearVelocity"][1] and bands["yawRate"][0] <= bands["yawRate"][1]

    folder = ROOT / "src/humanoid_whole_body_inverse_dynamics_mpc"
    builder = (folder / "whole_body_inverse_dynamics_mpc_config_builder.cpp").read_text()
    assert "buildCentroidalMpcConfig" not in builder
    assert "humanoid_centroidal_mpc_controller::" not in builder
    assert "p.model.footConstraint" not in builder
    for name in foot_gains:
        assert f"p.constraints.footMotionEquality.{name}" in builder
    for previous, current in WB_ID_LAYOUT_MIGRATIONS:
        for tree in (params["ocs2"], schema["ocs2"]):
            at_path(tree, current)
            try:
                at_path(tree, previous)
            except KeyError:
                pass
            else:
                raise AssertionError(f"Obsolete WB-ID parameter path remains: ocs2.{previous}")
        assert f"p.{current}" in builder, f"Missing WB-ID builder mapping: {current}"
        assert not re.search(rf"\bp\.{re.escape(previous)}\b", builder), f"Obsolete WB-ID builder mapping: {previous}"
    assert set(params["ocs2"]["model"]) == {
        "robotName", "fixedJointNames", "contactNames6DoF", "contactParentJointNames", "contactGeometry",
    }
    assert len(params["ocs2"]["reference"]["armSwing"]["jointNames"]) == 4
    assert set(params["ocs2"]["solver"]) == {"sqp", "rollout"}
    assert "mpc" in params["ocs2"] and "mpc" not in params["ocs2"]["solver"]
    assert "policyTimeOffset" not in params["control"] and "policyTimeOffset" not in schema["control"]
    assert "policyTimeOffset" in params["ocs2"]["solver"]["rollout"]
    controller = (folder / (NAME + ".cpp")).read_text()
    assert controller.count("parameters_.ocs2.solver.rollout.policyTimeOffset") == 2
    assert "parameters_.control.policyTimeOffset" not in controller
    # One WB-ID limit array supplies both OCP soft bounds and feedforward clipping.
    for tree in (params, schema):
        assert "torqueLimit" not in tree["control"]
    at_path(schema, "ocs2.constraints.jointTorqueLimits.limits")
    limits = params["ocs2"]["constraints"]["jointTorqueLimits"].get("limits", [])
    assert len(limits) in (0, count) and all(limit > 0 for limit in limits)
    assert "p.constraints.jointTorqueLimits.limits" in builder
    assert "parameters_.ocs2.constraints.jointTorqueLimits.limits" in controller
    assert "torque_limit_ = mpc_interface_->getTorqueLimits();" in controller
    assert "params.control.torqueLimit" not in builder
    assert "parameters_.control.torqueLimit" not in controller
    assert "clamp_torque(command.feedforward);" in controller
    assert "torqueLimit" in config["humanoid_centroidal_mpc_controller"]["ros__parameters"]["control"]
    # The other controller deliberately retains its existing parameter contract.
    centroidal = config["humanoid_centroidal_mpc_controller"]["ros__parameters"]["ocs2"]
    assert "footConstraint" in centroidal["model"] and "armJointNames" in centroidal["model"]
    assert "contact" in centroidal["constraints"]
    # Every direct p.* access in the builder must exist in the standalone schema.
    for access in re.findall(r"\bp\.([A-Za-z_][\w.]*)", builder):
        node = schema["ocs2"]
        for part in access.split("."):
            if "type" in node:
                break  # vector member functions, e.g. .size()
            if part.endswith("_map"):
                part = "__map_" + part[:-4]
            if part == "at":
                break
            assert part in node, f"Builder accesses undeclared field: p.{access}"
            node = node[part]
    print("WB-ID parameter contract passed: standalone schema, unified Q/Qf/R, valid array sizes, gait templates and ladder.")


if __name__ == "__main__":
    main()
