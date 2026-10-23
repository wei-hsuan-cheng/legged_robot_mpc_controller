#!/usr/bin/env python3
"""Static WB-ID YAML/schema regression; no ROS build or simulator required."""

from pathlib import Path
import re
import yaml


ROOT = Path(__file__).resolve().parents[1]
NAME = "humanoid_whole_body_inverse_dynamics_mpc_controller"


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
