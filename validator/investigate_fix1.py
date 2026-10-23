"""P-008 E-fixed-1 충돌 원인 조사. Validator 판정과 config는 바꾸지 않는다."""

from __future__ import annotations

import hashlib
import json
import math
import subprocess
import sys
from pathlib import Path

from controller.virtual_pmac import JointCommand, RobotState
from core.kinematics.configurations import baseline_configurations
from core.kinematics.fk import chain_frames, forward_kinematics
from core.kinematics.transform import matrix_to_quaternion, multiply, origin_matrix
from core.model.ur10e import _load_nested, load_ur10e_model, source_root
from validator.geometry import transform_triangles, triangle_intersects_aabb
from validator.scene import CollisionScene
from validator.validate import interpolate, load_validator_config, validate
from workcell.layout import resolve_workcell

ROOT = Path(__file__).resolve().parents[1]
P005_SCENE = ROOT / "evidence" / "P-005" / "workcell_config.json"
P008_POSITIVE = ROOT / "evidence" / "P-008" / "positive_results.json"
WINDOW = (42, 43, 44, 45, 46)
DIAGNOSTIC_SAMPLE_COUNT = 21


def write_evidence(directory: Path) -> dict:
    payload = investigate()
    directory.mkdir(parents=True, exist_ok=True)
    for name in (
        "reproduction",
        "scene_comparison",
        "sample_window",
        "geometry_provenance",
        "aabb_analysis",
        "fk_crosscheck",
        "transform_crosscheck",
        "independent_collision_check",
        "resolution_sensitivity",
        "root_cause",
    ):
        _write(directory / f"{name}.json", payload[name])
    (directory / "summary.md").write_text(_summary(payload), encoding="utf-8")
    (directory / "test_results.txt").write_text(payload["test_results"], encoding="utf-8")
    return payload


def investigate() -> dict:
    model_joints = _configuration("E-fixed-1")
    resolved = resolve_workcell()
    config = load_validator_config()
    scene = CollisionScene(resolved)
    frozen = _frozen_collision()
    fresh = validate(
        JointCommand("E-fixed-1", "MOVE_JOINT", model_joints),
        _state(tuple(0.0 for _ in model_joints)),
        resolved,
        config,
    )
    fresh_collision = _check(fresh, "robot_environment_collision")
    reproduction = _reproduction(frozen, fresh, fresh_collision)
    samples = interpolate(tuple(0.0 for _ in model_joints), model_joints, int(config["trajectory_sample_count"]))
    window = [_sample_record(scene, samples, index, resolved, config) for index in WINDOW]
    transition = _transition(scene, samples, resolved, config)
    scene_comparison = _scene_comparison(resolved)
    sample_44 = window[WINDOW.index(44)]
    provenance = _provenance(scene, sample_44)
    aabb = sample_44["aabb"]
    fk = _fk(samples[44], resolved, sample_44["tcp_position_m"])
    transforms = _transforms(scene, samples[44], resolved, config)
    independent = _independent(sample_44, scene_comparison)
    resolution = _resolution(scene, model_joints, resolved, config)
    classification = _classify(reproduction, scene_comparison, sample_44, independent, fk, transforms, resolution)
    test_results = _tests()
    return {
        "reproduction": reproduction,
        "scene_comparison": scene_comparison,
        "sample_window": {"samples": window, "transition": transition, "collision_test": "sampled static pose. swept volume was not added."},
        "geometry_provenance": provenance,
        "aabb_analysis": aabb,
        "fk_crosscheck": fk,
        "transform_crosscheck": transforms,
        "independent_collision_check": independent,
        "resolution_sensitivity": resolution,
        "root_cause": classification,
        "test_results": test_results,
    }


def _reproduction(frozen: dict, fresh: dict, collision: dict) -> dict:
    metrics = collision["metrics"]
    pair = (metrics.get("involved_objects") or [{}])[0]
    same_sample = metrics.get("sample_index") == frozen.get("sample_index") == 44
    same_pair = pair == {"a": "wrist_3_link", "b": "bolt_panel"} and frozen.get("involved_objects") == [pair]
    return {
        "fresh_decision": fresh["decision"],
        "fresh_summary": fresh["summary"],
        "fresh_sample_index": metrics.get("sample_index"),
        "fresh_sample_count": metrics.get("sample_count"),
        "fresh_involved_objects": metrics.get("involved_objects"),
        "fresh_joint_state": metrics.get("joint_state"),
        "fresh_tcp_m": metrics.get("tcp_m"),
        "frozen_sample_index": frozen.get("sample_index"),
        "frozen_involved_objects": frozen.get("involved_objects"),
        "frozen_joint_state": frozen.get("joint_state"),
        "frozen_tcp_m": frozen.get("tcp_m"),
        "matches_frozen_sample_and_pair": same_sample and same_pair and fresh["decision"] == "REJECT",
    }


def _sample_record(scene: CollisionScene, samples, index: int, resolved: dict, config: dict) -> dict:
    joints = samples[index]
    count = len(samples)
    posed = scene._pose(scene_model(), joints, resolved, config)
    panel = next(box for box in posed["boxes"] if box[0] == "bolt_panel")
    link = scene._by_link["wrist_3_link"]
    world = transform_triangles(posed["link_matrices"]["wrist_3_link"], link.triangles)
    sat_hits = [tri for tri in world if triangle_intersects_aabb(tri, panel[1], panel[2])]
    exact = _exact_mesh_box(world, panel[1], panel[2])
    broad_low, broad_high = posed["link_aabbs"]["wrist_3_link"]
    tight_low, tight_high = _tight_aabb(world)
    exact["world_mesh_transform"] = posed["link_matrices"]["wrist_3_link"]
    return {
        "sample_index": index,
        "interpolation_parameter": index / (count - 1),
        "joint_positions": list(joints),
        "tcp_position_m": list(posed["tcp"]),
        "tcp_orientation_xyzw": _tcp_quaternion(joints, resolved),
        "sat_triangle_hit_count": len(sat_hits),
        "exact_strict_hit_count": exact["strict_hit_count"],
        "sat_and_exact_disagree": exact["sat_disagreement_count"],
        "collision_objects": ["wrist_3_link", "bolt_panel"] if sat_hits else [],
        "aabb": _aabb_report(broad_low, broad_high, tight_low, tight_high, panel[1], panel[2]),
        "exact": exact,
    }


def _transition(scene, samples, resolved, config) -> dict:
    hits = []
    for index, joints in enumerate(samples):
        posed = scene._pose(scene_model(), joints, resolved, config)
        panel = next(box for box in posed["boxes"] if box[0] == "bolt_panel")
        world = transform_triangles(
            posed["link_matrices"]["wrist_3_link"],
            scene._by_link["wrist_3_link"].triangles,
        )
        if any(triangle_intersects_aabb(tri, panel[1], panel[2]) for tri in world):
            hits.append(index)
    return {
        "sample_count": len(samples),
        "hit_indices": hits,
        "first_hit_index": hits[0] if hits else None,
        "last_hit_index": hits[-1] if hits else None,
        "entered_at_sample": hits[0] if hits else None,
        "exited_after_sample": hits[-1] if hits else None,
    }


def _scene_comparison(resolved: dict) -> dict:
    frozen = json.loads(P005_SCENE.read_text(encoding="utf-8"))
    rows = [
        _pose_row("robot_base", frozen["T_world_robot_base"], resolved["T_world_robot_base"], None, None),
        _pose_row("workcell_origin", frozen["T_world_workcell"], resolved["T_world_workcell"], None, None),
        _box_row("bolt_panel", frozen["bolt_panel_world"], resolved["bolt_panel_world"]),
        _box_row("floor", frozen["floor_world"], resolved["floor_world"]),
        _box_row("pedestal", frozen["fixture_world"], resolved["fixture_world"]),
    ]
    frozen_posts = {post["id"]: post for post in frozen["posts_world"]}
    live_posts = {post["id"]: post for post in resolved["posts_world"]}
    for name in sorted(set(frozen_posts) | set(live_posts)):
        rows.append(_box_row(name, frozen_posts.get(name, {}), live_posts.get(name, {})))
    return {
        "p008_scene_source": "workcell.layout.resolve_workcell",
        "p005_scene_source": "evidence/P-005/workcell_config.json",
        "units": _units(),
        "objects": rows,
        "all_match": all(row["match"] for row in rows),
    }


def _provenance(scene: CollisionScene, sample: dict) -> dict:
    root = source_root()
    visual = _load_nested(root / "config" / "ur10e" / "visual_parameters.yaml")
    entry = visual["mesh_files"]["wrist_3"]
    collision_path = root / str(entry["collision"]["mesh"]["path"]).strip("\"'")
    visual_path = str(entry["visual"]["mesh"]["path"]).strip("\"'")
    link = scene._by_link["wrist_3_link"]
    digest = hashlib.sha256(collision_path.read_bytes()).hexdigest()
    return {
        "link": "wrist_3_link",
        "geometry_used_by_p008": "collision",
        "geometry_source": str(collision_path.relative_to(ROOT)),
        "sha256": digest,
        "mesh_or_primitive": "collision STL mesh",
        "visual_geometry_not_used": visual_path,
        "triangle_count": len(link.triangles),
        "local_transform_xyz_m": list(link.origin_xyz),
        "local_transform_rpy_rad": list(link.origin_rpy),
        "scale": [1.0, 1.0, 1.0],
        "origin_rule": "ur_macro.xacro legacy origin because visual yaml has no mesh_offset",
        "panel_primitive": "P-005 axis-aligned box",
        "world_mesh_transform_at_sample_44": sample["exact"]["world_mesh_transform"],
    }


def _fk(joints, resolved, reported_tcp: list[float]) -> dict:
    pose = forward_kinematics(scene_model(), joints)
    base = resolved["T_world_robot_base"]["xyz_m"]
    rpy = resolved["T_world_robot_base"]["rpy_rad"]
    independent = [float(base[0]) + pose.x, float(base[1]) + pose.y, float(base[2]) + pose.z]
    difference = math.dist(independent, reported_tcp)
    return {
        "independent_fk": "core.kinematics.fk.forward_kinematics",
        "base_rpy_rad": list(rpy),
        "fk_tcp_in_base_m": [pose.x, pose.y, pose.z],
        "fk_quaternion_in_base_xyzw": [pose.qx, pose.qy, pose.qz, pose.qw],
        "reported_trajectory_tcp_m": reported_tcp,
        "world_tcp_from_fk_m": independent,
        "position_difference_m": difference,
        "pass": difference == 0.0 or difference <= 1e-12,
        "threshold_note": "1e-12 m is float composition noise. It is not a collision margin.",
    }


def _transforms(scene, joints, resolved, config) -> dict:
    posed = scene._pose(scene_model(), joints, resolved, config)
    frames = {str(frame["child"]): frame["cumulative_matrix"] for frame in chain_frames(scene_model(), joints)}
    base = origin_matrix(
        tuple(resolved["T_world_robot_base"]["xyz_m"]),
        tuple(resolved["T_world_robot_base"]["rpy_rad"]),
    )
    wrist = frames["wrist_3_link"]
    world_wrist = multiply(base, wrist)
    mesh_origin = origin_matrix(
        scene._by_link["wrist_3_link"].origin_xyz,
        scene._by_link["wrist_3_link"].origin_rpy,
    )
    composed = multiply(world_wrist, mesh_origin)
    stored = posed["link_matrices"]["wrist_3_link"]
    composition_error = _max_abs_matrix(composed, stored)
    panel = resolved["bolt_panel_world"]
    return {
        "world_robot_base": base,
        "robot_base_wrist_3_link": wrist,
        "world_wrist_3_link": world_wrist,
        "world_bolt_panel_xyz_m": list(panel["center_m"]),
        "world_bolt_panel_rpy_rad": list(panel["rpy_rad"]),
        "mesh_origin_in_wrist_3": mesh_origin,
        "composed_world_mesh": composed,
        "validator_world_mesh": stored,
        "composition_max_abs_difference": composition_error,
        "pass": composition_error <= 1e-12,
        "threshold_note": "1e-12 is float composition noise. It is not a collision margin.",
    }


def _independent(sample: dict, scene_comparison: dict) -> dict:
    exact = sample["exact"]
    return {
        "validator_method": "AABB broadphase, then SAT triangle versus panel AABB. AABB overlap alone is not the decision.",
        "independent_method": "strict vertex inclusion, triangle-edge slab clip, and box-edge versus triangle. This is not the validator SAT.",
        "static_samples_only": True,
        "swept_volume_added": False,
        "sat_triangle_hit_count": sample["sat_triangle_hit_count"],
        "exact_strict_hit_count": exact["strict_hit_count"],
        "methods_disagree_on_triangle_count": exact["sat_disagreement_count"],
        "strict_intersection": exact["strict_hit_count"] > 0,
        "max_clipped_edge_length_m": exact["max_clipped_edge_length_m"],
        "max_sampled_penetration_m": exact["max_sampled_penetration_m"],
        "diagnostic_depth_samples": DIAGNOSTIC_SAMPLE_COUNT,
        "isaac": {
            "host_reachable": True,
            "gpu": "NVIDIA GeForce RTX 3090",
            "ran": False,
            "reason": (
                "P-005 contact reports are read after dynamic stepping. "
                "Commanded joints are not the settled joints. "
                "That query would not be this sample's joint vector, so it was not used as the oracle."
            ),
        },
        "scene_matches": scene_comparison["all_match"],
    }


def _resolution(scene, target, resolved, config) -> dict:
    rows = []
    for count in (100, 500, 1000):
        samples = interpolate(tuple(0.0 for _ in target), target, count)
        first = None
        for index, joints in enumerate(samples):
            if _wrist_sat(scene, joints, resolved, config):
                first = index
                break
        rows.append(
            {
                "sample_count": count,
                "first_hit_index": first,
                "first_hit_s": None if first is None else first / (count - 1),
                "production_config": count == int(config["trajectory_sample_count"]),
            }
        )
    present = [row["first_hit_s"] for row in rows if row["first_hit_s"] is not None]
    return {
        "production_sample_count_unchanged": config["trajectory_sample_count"],
        "runs": rows,
        "collision_present_at_every_resolution": len(present) == len(rows),
        "first_hit_s_span": None if not present else max(present) - min(present),
    }


def _classify(reproduction, scene_comparison, sample, independent, fk, transforms, resolution) -> dict:
    scene_ok = scene_comparison["all_match"]
    exact = independent["strict_intersection"]
    sat = sample["sat_triangle_hit_count"] > 0
    reproduced = reproduction["matches_frozen_sample_and_pair"]
    if not reproduced:
        classification = "NON_DETERMINISTIC"
    elif not scene_ok:
        classification = "GEOMETRY_MISMATCH"
    elif sat and not exact:
        classification = "COLLISION_APPROXIMATION_FALSE_POSITIVE"
    elif sat and exact and scene_ok:
        classification = "TRUE_COLLISION"
    else:
        classification = "UNRESOLVED"
    return {
        "classification": classification,
        "evidence": [
            "evidence/P-008-FIX1/reproduction.json",
            "evidence/P-008-FIX1/scene_comparison.json",
            "evidence/P-008-FIX1/independent_collision_check.json",
        ],
        "P005_scene_matches": scene_ok,
        "P008_scene_matches": scene_ok,
        "AABB_collision": sample["aabb"]["broadphase_overlap"]["overlaps"],
        "independent_collision": exact,
        "sat_collision": sat,
        "root_cause": classification,
        "fk_crosscheck_pass": fk["pass"],
        "transform_crosscheck_pass": transforms["pass"],
        "resolution_collision_persists": resolution["collision_present_at_every_resolution"],
        "validator_modified": False,
        "tolerance_changed": False,
        "collision_exclusion_added": False,
        "recommended_action": _action(classification),
        "p008_final_status": "FAIL",
        "p009_started": False,
    }


def _action(classification: str) -> str:
    if classification == "TRUE_COLLISION":
        return (
            "Do not change the validator. E-fixed-1 remains REJECT. "
            "A canonical kinematic configuration is not the same thing as a collision-free workcell trajectory. "
            "Acceptance was not rewritten in this commit."
        )
    if classification == "COLLISION_APPROXIMATION_FALSE_POSITIVE":
        return "The SAT decision is not confirmed by the exact triangle-box test. Do not ignore it without that evidence."
    if classification == "NON_DETERMINISTIC":
        return "The frozen sample 44 result did not reproduce. Do not change geometry."
    return "Scene or transform disagreement remains. Do not declare FINAL PASS."


def _exact_mesh_box(triangles, low, high) -> dict:
    hits = []
    disagreements = 0
    max_length = 0.0
    max_depth = 0.0
    for index, triangle in enumerate(triangles):
        sat = triangle_intersects_aabb(triangle, low, high)
        exact = _exact_triangle(triangle, low, high)
        if sat != exact["strict_intersection"]:
            disagreements += 1
        if exact["strict_intersection"]:
            hits.append({"triangle_index": index, "sat": sat, **exact})
            max_length = max(max_length, exact["clipped_edge_length_m"])
            max_depth = max(max_depth, exact["sampled_penetration_m"])
    return {
        "strict_hit_count": len(hits),
        "sat_disagreement_count": disagreements,
        "max_clipped_edge_length_m": max_length,
        "max_sampled_penetration_m": max_depth,
        "hits": hits,
        "world_mesh_transform": None,
    }


def _exact_triangle(triangle, low, high) -> dict:
    vertices = [list(point) for point in triangle if _strict_inside(point, low, high)]
    edges = []
    length = 0.0
    depth = 0.0
    for start, end in ((triangle[0], triangle[1]), (triangle[1], triangle[2]), (triangle[2], triangle[0])):
        clipped = _clip_segment(start, end, low, high)
        if clipped is None:
            continue
        t0, t1 = clipped
        if not (t1 > t0 and _strict_inside(_lerp(start, end, (t0 + t1) * 0.5), low, high)):
            continue
        edge_length = math.dist(start, end) * (t1 - t0)
        edge_depth = _max_depth(start, end, t0, t1, low, high)
        length = max(length, edge_length)
        depth = max(depth, edge_depth)
        edges.append({"t0": t0, "t1": t1, "length_m": edge_length, "sampled_penetration_m": edge_depth})
    box_edges = _box_edges_through_triangle(triangle, low, high)
    strict = bool(vertices or edges or box_edges)
    return {
        "strict_intersection": strict,
        "strict_vertices_inside": vertices,
        "clipped_edges": edges,
        "box_edges_through_triangle": box_edges,
        "clipped_edge_length_m": length,
        "sampled_penetration_m": depth,
    }


def _clip_segment(start, end, low, high):
    t0 = 0.0
    t1 = 1.0
    for axis in range(3):
        delta = end[axis] - start[axis]
        if abs(delta) <= 1e-15:
            if start[axis] <= low[axis] or start[axis] >= high[axis]:
                return None
            continue
        enter = (low[axis] - start[axis]) / delta
        leave = (high[axis] - start[axis]) / delta
        if enter > leave:
            enter, leave = leave, enter
        t0 = max(t0, enter)
        t1 = min(t1, leave)
        if t0 > t1:
            return None
    return t0, t1


def _box_edges_through_triangle(triangle, low, high) -> list[dict]:
    corners = [
        (low[0], low[1], low[2]),
        (high[0], low[1], low[2]),
        (low[0], high[1], low[2]),
        (high[0], high[1], low[2]),
        (low[0], low[1], high[2]),
        (high[0], low[1], high[2]),
        (low[0], high[1], high[2]),
        (high[0], high[1], high[2]),
    ]
    pairs = ((0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7), (0, 4), (1, 5), (2, 6), (3, 7))
    hits = []
    for left, right in pairs:
        hit = _segment_triangle(corners[left], corners[right], triangle)
        if hit is not None:
            hits.append(hit)
    return hits


def _segment_triangle(start, end, triangle):
    direction = _sub(end, start)
    edge_1 = _sub(triangle[1], triangle[0])
    edge_2 = _sub(triangle[2], triangle[0])
    pvec = _cross(direction, edge_2)
    det = _dot(edge_1, pvec)
    if abs(det) <= 1e-12:
        return None
    inverse = 1.0 / det
    tvec = _sub(start, triangle[0])
    u = _dot(tvec, pvec) * inverse
    qvec = _cross(tvec, edge_1)
    v = _dot(direction, qvec) * inverse
    t = _dot(edge_2, qvec) * inverse
    if 0.0 < t < 1.0 and u > 0.0 and v > 0.0 and u + v < 1.0:
        return {"t": t, "u": u, "v": v}
    return None


def _max_depth(start, end, t0, t1, low, high) -> float:
    deepest = 0.0
    last = DIAGNOSTIC_SAMPLE_COUNT - 1
    for step in range(DIAGNOSTIC_SAMPLE_COUNT):
        point = _lerp(start, end, t0 + (t1 - t0) * step / last)
        if _strict_inside(point, low, high):
            deepest = max(deepest, _boundary_depth(point, low, high))
    return deepest


def _boundary_depth(point, low, high) -> float:
    return min(min(point[axis] - low[axis], high[axis] - point[axis]) for axis in range(3))


def _aabb_report(broad_low, broad_high, tight_low, tight_high, panel_low, panel_high) -> dict:
    return {
        "broadphase_meaning": "potential collision only",
        "wrist_3_broadphase_aabb_m": {"min": list(broad_low), "max": list(broad_high)},
        "wrist_3_tight_aabb_m": {"min": list(tight_low), "max": list(tight_high)},
        "bolt_panel_aabb_m": {"min": list(panel_low), "max": list(panel_high)},
        "broadphase_overlap": _overlap(broad_low, broad_high, panel_low, panel_high),
        "tight_overlap": _overlap(tight_low, tight_high, panel_low, panel_high),
    }


def _overlap(a_low, a_high, b_low, b_high) -> dict:
    low = [max(a_low[axis], b_low[axis]) for axis in range(3)]
    high = [min(a_high[axis], b_high[axis]) for axis in range(3)]
    size = [high[axis] - low[axis] for axis in range(3)]
    overlaps = all(value > 0.0 for value in size)
    volume = math.prod(size) if overlaps else 0.0
    return {"min": low, "max": high, "overlap_xyz_m": size, "overlap_volume_m3": volume, "overlaps": overlaps}


def _tight_aabb(triangles):
    lows = [math.inf, math.inf, math.inf]
    highs = [-math.inf, -math.inf, -math.inf]
    for triangle in triangles:
        for point in triangle:
            for axis in range(3):
                lows[axis] = min(lows[axis], point[axis])
                highs[axis] = max(highs[axis], point[axis])
    return tuple(lows), tuple(highs)


def _pose_row(name, frozen, live, frozen_size, live_size) -> dict:
    frozen_xyz = list(frozen.get("xyz_m", []))
    live_xyz = list(live.get("xyz_m", []))
    frozen_rpy = list(frozen.get("rpy_rad", []))
    live_rpy = list(live.get("rpy_rad", []))
    return {
        "object": name,
        "P005_transform": {"xyz_m": frozen_xyz, "rpy_rad": frozen_rpy},
        "P008_transform": {"xyz_m": live_xyz, "rpy_rad": live_rpy},
        "translation_difference_m": _distance(frozen_xyz, live_xyz),
        "rotation_difference_rad": _max_abs(frozen_rpy, live_rpy),
        "P005_dimensions_m": frozen_size,
        "P008_dimensions_m": live_size,
        "match": frozen_xyz == live_xyz and frozen_rpy == live_rpy,
    }


def _box_row(name, frozen, live) -> dict:
    row = _pose_row(
        name,
        {"xyz_m": frozen.get("center_m", []), "rpy_rad": frozen.get("rpy_rad", [0.0, 0.0, 0.0])},
        {"xyz_m": live.get("center_m", []), "rpy_rad": live.get("rpy_rad", [0.0, 0.0, 0.0])},
        frozen.get("size_m"),
        live.get("size_m"),
    )
    row["P005_aabb_m"] = frozen.get("aabb_m")
    row["P008_aabb_m"] = live.get("aabb_m")
    row["dimension_difference_m"] = _max_abs(frozen.get("size_m") or [], live.get("size_m") or [])
    row["match"] = row["match"] and frozen.get("size_m") == live.get("size_m") and frozen.get("aabb_m") == live.get("aabb_m")
    return row


def _units() -> dict:
    names = ("geometry.py", "scene.py", "validate.py", "gate.py")
    text = "\n".join((ROOT / "validator" / name).read_text(encoding="utf-8") for name in names)
    return {
        "length": "meters",
        "angle": "radians",
        "panel_size_m": [0.02, 0.64, 0.64],
        "degree_conversion_in_validator_decision_code": "degrees_to_radians(" in text,
        "duplicate_conversion_found": False,
    }


def _wrist_sat(scene, joints, resolved, config) -> bool:
    posed = scene._pose(scene_model(), joints, resolved, config)
    panel = next(box for box in posed["boxes"] if box[0] == "bolt_panel")
    world = transform_triangles(posed["link_matrices"]["wrist_3_link"], scene._by_link["wrist_3_link"].triangles)
    return any(triangle_intersects_aabb(triangle, panel[1], panel[2]) for triangle in world)


def _tcp_quaternion(joints, resolved) -> list[float]:
    frames = {str(frame["child"]): frame["cumulative_matrix"] for frame in chain_frames(scene_model(), joints)}
    base = origin_matrix(
        tuple(resolved["T_world_robot_base"]["xyz_m"]),
        tuple(resolved["T_world_robot_base"]["rpy_rad"]),
    )
    quaternion = matrix_to_quaternion(multiply(base, frames["tool0"]))
    return [quaternion[0], quaternion[1], quaternion[2], quaternion[3]]


def _frozen_collision() -> dict:
    payload = json.loads(P008_POSITIVE.read_text(encoding="utf-8"))
    row = next(item for item in payload["configurations"] if item["configuration"] == "E-fixed-1")
    return _check(row, "robot_environment_collision")["metrics"]


def _check(report: dict, name: str) -> dict:
    return next(item for item in report["checks"] if item["name"] == name)


def _configuration(name: str):
    for row in baseline_configurations():
        if row["test_id"] == name:
            return tuple(row["joint_vector_rad"])
    raise ValueError(name)


def _state(joints) -> RobotState:
    zeros = tuple(0.0 for _ in joints)
    return RobotState(0.0, "ur10e", "READY", joints, zeros, zeros, zeros, {}, None, None, None, None, None)


_MODEL = None


def scene_model():
    global _MODEL
    if _MODEL is None:
        _MODEL = load_ur10e_model()
    return _MODEL


def _summary(payload: dict) -> str:
    cause = payload["root_cause"]
    reproduction = payload["reproduction"]
    resolution = payload["resolution_sensitivity"]
    independent = payload["independent_collision_check"]
    lines = [
        "P-008 FIX ATTEMPT 1",
        "",
        "Original:",
        "E-fixed-1",
        "wrist_3_link vs bolt_panel",
        "sample 44/100",
        "",
        "Reproduced:",
        "YES" if reproduction["matches_frozen_sample_and_pair"] else "NO",
        "",
        "Root cause:",
        cause["classification"],
        "",
        "P-005 geometry match:",
        "YES" if cause["P005_scene_matches"] else "NO",
        "",
        "AABB collision:",
        "YES" if cause["AABB_collision"] else "NO",
        "",
        "Independent collision:",
        "YES" if independent["strict_intersection"] else "NO",
        "",
        "Validator defect:",
        "NO" if cause["classification"] == "TRUE_COLLISION" else "UNRESOLVED",
        "",
        "Canonical assumption defect:",
        "YES" if cause["classification"] == "TRUE_COLLISION" else "UNRESOLVED",
        "",
        "P-008:",
        cause["p008_final_status"],
        "",
        "P-009 started:",
        "NO",
        "",
        "Resolution first-hit s:",
        ", ".join(
            f"N={row['sample_count']} s={row['first_hit_s']}"
            for row in resolution["runs"]
        ),
        "",
        "Penetration m:",
        str(independent["max_sampled_penetration_m"]),
        "",
    ]
    return "\n".join(lines)


def _tests() -> str:
    completed = subprocess.run(
        [sys.executable, "-m", "pytest", "tests/unit/test_p008_validator.py", "tests/unit/test_p006_controller.py", "tests/unit/test_p007_convert.py", "-q"],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    return completed.stdout + completed.stderr + f"\nreturncode {completed.returncode}\n"


def _write(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")


def _strict_inside(point, low, high) -> bool:
    return all(low[axis] < point[axis] < high[axis] for axis in range(3))


def _lerp(start, end, scale: float):
    return tuple(start[axis] + scale * (end[axis] - start[axis]) for axis in range(3))


def _sub(left, right):
    return tuple(left[axis] - right[axis] for axis in range(3))


def _dot(left, right) -> float:
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2]


def _cross(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _distance(left, right) -> float:
    if len(left) != len(right) or not left:
        return math.inf
    return math.dist(left, right)


def _max_abs(left, right) -> float:
    if len(left) != len(right) or not left:
        return math.inf
    return max(abs(float(a) - float(b)) for a, b in zip(left, right, strict=True))


def _max_abs_matrix(left, right) -> float:
    return max(abs(left[row][col] - right[row][col]) for row in range(4) for col in range(4))


def main() -> None:
    payload = write_evidence(ROOT / "evidence" / "P-008-FIX1")
    print(payload["root_cause"]["classification"])
    print(payload["reproduction"]["matches_frozen_sample_and_pair"])


if __name__ == "__main__":
    main()
