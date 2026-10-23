"""P-005 workcell config와 zero-pose clearance를 Isaac 없이 확인한다."""

import json
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

from core.kinematics.fk import chain_frames
from core.kinematics.transform import multiply, origin_matrix
from core.model.ur10e import SOURCE_COMMIT, load_ur10e_model, source_root
from workcell.layout import (
    load_workcell,
    max_position_difference_m,
    resolve_workcell,
    tool_box_world_matrix,
    tool_collision_matrix_in_wrist3,
)
from workcell.report import judge_workcell, render_test_results, write_evidence

ROOT = Path(__file__).resolve().parents[2]
GAP_M = 0.001


def test_config_is_ur10e_reference_workcell_without_process_state():
    payload = load_workcell()
    resolved_a = resolve_workcell(payload)
    resolved_b = resolve_workcell(payload)
    targets = resolved_a["bolt_targets_world"]

    assert payload["claim"] == "UR10e-based reference workcell"
    assert payload["robot"]["source_commit"] == SOURCE_COMMIT
    assert "FilterBot" not in json.dumps(payload)
    assert [item["id"] for item in targets] == ["B01", "B02", "B03", "B04", "B05", "B06"]
    assert {item["nominal_reach_sphere"] for item in targets} == {
        "inside_nominal_reach_sphere",
        "outside_nominal_reach_sphere",
    }
    assert resolved_a["geometric_sanity"] == "GEOMETRIC SANITY ONLY. NOT IK VALIDATION."
    assert resolved_a["T_world_bolt_panel"]["xyz_m"] == resolved_b["T_world_bolt_panel"]["xyz_m"]
    assert payload["isaac_import"]["override_joint_stiffness_nm_per_rad"] == 200000.0
    assert payload["isaac_import"]["override_joint_damping"] == 1000.0
    assert "torque" not in json.dumps(payload)


def test_tool_collision_frame_matches_tool0_at_zero():
    payload = load_workcell()
    robot = load_ur10e_model()
    frames = {
        frame["child"]: frame["cumulative_matrix"]
        for frame in chain_frames(robot, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    }
    local = tool_collision_matrix_in_wrist3(payload)
    world_from_wrist = multiply(frames["wrist_3_link"], local)
    base = origin_matrix((0.0, 0.0, 0.75), (0.0, 0.0, 0.0))
    world = multiply(base, world_from_wrist)
    expected = tool_box_world_matrix(payload)
    for row in range(4):
        for col in range(4):
            assert abs(world[row][col] - expected[row][col]) < 1e-12


def test_zero_pose_collision_meshes_clear_the_workcell():
    payload = resolve_workcell()
    robot_bounds = _zero_pose_robot_bounds(payload["robot"]["T_world_robot_base"]["xyz_m"])
    bodies = [
        payload["floor_world"]["aabb_m"],
        payload["fixture_world"]["aabb_m"],
        payload["bolt_panel_world"]["aabb_m"],
        payload["tool_box_world_aabb_m"],
    ]
    bodies.extend(post["aabb_m"] for post in payload["posts_world"])
    for body in bodies:
        assert _separated(robot_bounds, body, GAP_M)
    environment = [
        payload["floor_world"]["aabb_m"],
        payload["fixture_world"]["aabb_m"],
        payload["bolt_panel_world"]["aabb_m"],
    ]
    environment.extend(post["aabb_m"] for post in payload["posts_world"])
    for body in environment:
        assert _separated(payload["tool_box_world_aabb_m"], body, GAP_M)


def test_judge_accepts_a_stable_measurement_and_rejects_contact():
    stable = _measurement()
    assert judge_workcell(stable)["result"] == "PASS"
    contact = _measurement()
    contact["unexpected_collision_count"] = 1
    assert judge_workcell(contact)["result"] == "FAIL"
    unmeasured = _measurement()
    unmeasured["initial_penetration_count"] = None
    assert judge_workcell(unmeasured)["result"] == "FAIL"


def test_evidence_writer_creates_the_required_files(tmp_path: Path):
    judgement = judge_workcell(_measurement())
    write_evidence(
        tmp_path,
        {
            "workcell_config": {"claim": "UR10e-based reference workcell"},
            "scene_validation": {"required_prims_present": True},
            "stability_results": {"duration_s": 10.0},
            "reset_results": {"robot_base_difference_m": 0.0},
            "collision_results": {"unexpected_collision_count": 0},
            "environment_md": "isaac\n",
            "summary_md": "PASS\n",
            "test_results_txt": render_test_results(judgement),
        },
    )
    for name in (
        "summary.md",
        "environment.md",
        "workcell_config.json",
        "scene_validation.json",
        "stability_results.json",
        "reset_results.json",
        "collision_results.json",
        "test_results.txt",
    ):
        assert (tmp_path / name).is_file()


def test_p004_isaac_drive_settings_remain():
    text = (ROOT / "isaac" / "scripts" / "run_ur10e_tcp.py").read_text(encoding="utf-8")
    assert "DRIVE_STIFFNESS = 200000.0" in text
    assert "DRIVE_DAMPING = 1000.0" in text
    assert "fix_articulation_root_for_fixed_base" in text


def test_build_snapshots_compare_zero_when_repeated():
    resolved = resolve_workcell()
    left = {item["id"]: item["position_world_m"] for item in resolved["bolt_targets_world"]}
    right = {item["id"]: item["position_world_m"] for item in resolve_workcell()["bolt_targets_world"]}
    assert max_position_difference_m(left, right) == 0.0


def _measurement() -> dict:
    return {
        "acceptance": load_workcell()["acceptance"],
        "build_position_difference_m": 0.0,
        "base_authoring_error_m": 0.0,
        "tool_collision_authoring_error_m": 0.0,
        "bolt_authoring_error_m": 0.0,
        "initial_base_error_m": 0.0,
        "required_prims_present": True,
        "required_frames_present": True,
        "collision_geometry_present": True,
        "panel_collision_query_hit": True,
        "finite_transforms": True,
        "non_finite_count": 0,
        "base_displacement_m": 0.0,
        "max_joint_drift_rad": 0.0,
        "tool0_drift_m": 0.0,
        "floor_displacement_m": 0.0,
        "work_structure_displacement_m": 0.0,
        "bolt_panel_displacement_m": 0.0,
        "pedestal_displacement_m": 0.0,
        "unexpected_collision_count": 0,
        "initial_penetration_count": 0,
        "reset_robot_base_difference_m": 0.0,
        "reset_joint_difference_rad": 0.0,
        "reset_tool0_difference_m": 0.0,
        "reset_bolt_panel_difference_m": 0.0,
        "bolt_targets_build_difference_m": 0.0,
    }


def _zero_pose_robot_bounds(base_xyz: list[float]) -> dict[str, list[float]]:
    robot = load_ur10e_model()
    frames = {
        frame["child"]: frame["cumulative_matrix"]
        for frame in chain_frames(robot, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    }
    urdf = ROOT / "robot_description" / "generated" / SOURCE_COMMIT / "ur10e.urdf"
    root = ET.parse(urdf).getroot()
    lows = [1e300, 1e300, 1e300]
    highs = [-1e300, -1e300, -1e300]
    for link in root.findall("link"):
        collision = link.find("collision")
        if collision is None:
            continue
        mesh = collision.find("geometry/mesh")
        origin = collision.find("origin")
        xyz = _triplet(origin.get("xyz") if origin is not None else "0 0 0")
        rpy = _triplet(origin.get("rpy") if origin is not None else "0 0 0")
        filename = mesh.get("filename").split("meshes/", 1)[1]
        transform = multiply(frames[link.get("name")], origin_matrix(tuple(xyz), tuple(rpy)))
        for vertex in _stl_vertices(source_root() / "meshes" / filename):
            point = [vertex[0], vertex[1], vertex[2], 1.0]
            world = [sum(transform[row][col] * point[col] for col in range(4)) for row in range(3)]
            world[0] += base_xyz[0]
            world[1] += base_xyz[1]
            world[2] += base_xyz[2]
            for axis in range(3):
                lows[axis] = min(lows[axis], world[axis])
                highs[axis] = max(highs[axis], world[axis])
    return {"min_m": lows, "max_m": highs}


def _stl_vertices(path: Path):
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    offset = 84
    for _ in range(count):
        values = struct.unpack_from("<9f", data, offset + 12)
        for index in range(3):
            yield (values[index * 3], values[index * 3 + 1], values[index * 3 + 2])
        offset += 50


def _triplet(text: str) -> list[float]:
    return [float(item) for item in text.split()]


def _separated(left: dict, right: dict, gap: float) -> bool:
    for axis in range(3):
        if left["max_m"][axis] + gap <= right["min_m"][axis]:
            return True
        if right["max_m"][axis] + gap <= left["min_m"][axis]:
            return True
    return False
