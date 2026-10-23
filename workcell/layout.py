"""Machine-readable UR10e reference workcell.

회전체와 bolt 공정 상태는 없다. 길이는 meter, 각은 radian이다.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

from core.kinematics.fk import chain_frames
from core.kinematics.transform import identity, multiply, origin_matrix
from core.model.ur10e import SOURCE_COMMIT, load_ur10e_model

CLAIM = "UR10e-based reference workcell"
FORBIDDEN_KEYS = frozenset({"torque", "process_state", "fastening_result", "fastening_success"})
REQUIRED_FRAMES = ("world", "robot_base", "workcell", "bolt_panel", "tool0")
REQUIRED_TARGETS = ("B01", "B02", "B03", "B04", "B05", "B06")


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def default_config_path() -> Path:
    return repo_root() / "workcell" / "config" / "workcell.json"


def load_workcell(path: Path | None = None) -> dict:
    source = default_config_path() if path is None else path
    payload = json.loads(source.read_text(encoding="utf-8"))
    validate_workcell(payload)
    return payload


def validate_workcell(payload: dict) -> None:
    if payload.get("claim") != CLAIM:
        raise ValueError("workcell claim must be UR10e-based reference workcell")
    blob = json.dumps(payload)
    if "FilterBot" in blob or "Core Robotics" in blob:
        raise ValueError("workcell config names a forbidden robot")
    _reject_forbidden_keys(payload)
    if payload["units"] != {"length": "meter", "angle": "radian", "time": "second"}:
        raise ValueError("canonical units changed")
    if tuple(payload["frames"]) != REQUIRED_FRAMES:
        raise ValueError("required frames are missing")
    robot = payload["robot"]
    if robot["source_commit"] != SOURCE_COMMIT:
        raise ValueError("robot source commit does not match the pinned UR10e model")
    if robot["model"] != "ur10e":
        raise ValueError("P-005 reference manipulator is the pinned UR10e")
    floor = payload["floor"]["size_m"]
    if floor[0] < 2.5 or floor[1] < 2.5:
        raise ValueError("floor is smaller than 2.5 m by 2.5 m")
    targets = payload["bolt_targets"]
    if tuple(item["id"] for item in targets) != REQUIRED_TARGETS:
        raise ValueError("bolt targets must be B01 through B06")
    for item in targets:
        if item["frame"] != "bolt_panel":
            raise ValueError(item["id"] + " is not in the bolt_panel frame")
        axis = item["approach_axis"]
        norm = math.sqrt(sum(float(value) * float(value) for value in axis))
        if abs(norm - 1.0) > 1e-12:
            raise ValueError(item["id"] + " approach_axis is not a unit vector")
        _require_finite(item["position_m"])
    tool = payload["tool_placeholder"]
    if tool["label"] != "PROJECT TEST GEOMETRY":
        raise ValueError("tool placeholder label changed")
    if tool["disclaimer"] != "NOT ACTUAL INDUSTRIAL TOOL":
        raise ValueError("tool placeholder disclaimer changed")
    panel = payload["bolt_panel"]
    if panel["size_m"][0] >= panel["size_m"][1] or panel["size_m"][0] >= panel["size_m"][2]:
        raise ValueError("bolt panel thin axis is not X")
    if payload["physics"]["gravity_enabled"] is not True:
        raise ValueError("gravity is off")
    importer = payload["isaac_import"]
    if importer["override_joint_stiffness_nm_per_rad"] != 200000.0:
        raise ValueError("Isaac stiffness is not the P-004 value")
    if importer["override_joint_damping"] != 1000.0:
        raise ValueError("Isaac damping is not the P-004 value")
    if importer["relocate_articulation_root"] is not True:
        raise ValueError("articulation root relocation is off")
    if importer["collision_from_visuals"] is not False:
        raise ValueError("collision_from_visuals must stay false")
    _require_finite(robot["T_world_robot_base"]["xyz_m"])
    _require_finite(payload["workcell"]["T_world_workcell"]["xyz_m"])


def resolve_workcell(payload: dict | None = None) -> dict:
    """config의 identity 회전을 world 위치로 푼다. 두 번 호출해도 같다."""
    source = load_workcell() if payload is None else payload
    validate_workcell(source)
    workcell_xyz = _xyz(source["workcell"]["T_world_workcell"]["xyz_m"])
    panel_local = _xyz(source["bolt_panel"]["center_in_workcell_m"])
    panel_world = _add(workcell_xyz, panel_local)
    posts = []
    for post in source["work_structure"]["posts"]:
        center = _add(workcell_xyz, _xyz(post["center_in_workcell_m"]))
        posts.append({**post, "center_m": center, "aabb_m": _aabb(center, post["size_m"])})
    targets = []
    for item in source["bolt_targets"]:
        world = _add(panel_world, _xyz(item["position_m"]))
        distance = _distance(_xyz(source["robot"]["T_world_robot_base"]["xyz_m"]), world)
        targets.append(
            {
                **item,
                "position_world_m": world,
                "distance_from_robot_base_m": distance,
                "nominal_reach_sphere": _reach_class(distance, float(source["robot"]["nominal_reach_m"])),
            }
        )
    resolved = {
        **source,
        "T_world_robot_base": source["robot"]["T_world_robot_base"],
        "T_world_workcell": source["workcell"]["T_world_workcell"],
        "T_world_bolt_panel": {"xyz_m": panel_world, "rpy_rad": list(source["bolt_panel"]["rpy_rad"])},
        "floor_world": {**source["floor"], "aabb_m": _aabb(source["floor"]["center_m"], source["floor"]["size_m"])},
        "fixture_world": {
            **source["fixture"],
            "aabb_m": _aabb(source["fixture"]["center_m"], source["fixture"]["size_m"]),
        },
        "posts_world": posts,
        "bolt_panel_world": {
            **source["bolt_panel"],
            "center_m": panel_world,
            "aabb_m": _aabb(panel_world, source["bolt_panel"]["size_m"]),
        },
        "bolt_targets_world": targets,
        "tool_box_world_aabb_m": _oriented_aabb(tool_box_world_matrix(source), source["tool_placeholder"]["size_m"]),
        "geometric_sanity": "GEOMETRIC SANITY ONLY. NOT IK VALIDATION.",
    }
    return resolved


def tool_collision_matrix_in_wrist3(payload: dict) -> list[list[float]]:
    """q=0에서 wrist_3_link 기준 tool cube center. 고정 조인트라 관절각에 따라 변하지 않는다."""
    frames = _zero_frames()
    center = origin_matrix(tuple(payload["tool_placeholder"]["center_in_tool0_m"]), (0.0, 0.0, 0.0))
    base_center = multiply(frames["tool0"], center)
    return multiply(_invert_rigid(frames["wrist_3_link"]), base_center)


def tool_box_world_matrix(payload: dict) -> list[list[float]]:
    frames = _zero_frames()
    base = origin_matrix(
        tuple(payload["robot"]["T_world_robot_base"]["xyz_m"]),
        tuple(payload["robot"]["T_world_robot_base"]["rpy_rad"]),
    )
    center = origin_matrix(tuple(payload["tool_placeholder"]["center_in_tool0_m"]), (0.0, 0.0, 0.0))
    return multiply(multiply(base, frames["tool0"]), center)


def max_position_difference_m(left: dict[str, list[float]], right: dict[str, list[float]]) -> float:
    if set(left) != set(right):
        raise ValueError("transform snapshots name different frames")
    worst = 0.0
    for name in left:
        worst = max(worst, _distance(left[name], right[name]))
    return worst


def _zero_frames() -> dict[str, list[list[float]]]:
    robot = load_ur10e_model()
    frames = {}
    for frame in chain_frames(robot, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)):
        frames[str(frame["child"])] = frame["cumulative_matrix"]
    return frames


def _invert_rigid(matrix: list[list[float]]) -> list[list[float]]:
    rotation = [[matrix[row][col] for col in range(3)] for row in range(3)]
    translation = [matrix[0][3], matrix[1][3], matrix[2][3]]
    transposed = [[rotation[col][row] for col in range(3)] for row in range(3)]
    inverse_translation = [
        -sum(transposed[row][col] * translation[col] for col in range(3)) for row in range(3)
    ]
    inverse = identity()
    for row in range(3):
        for col in range(3):
            inverse[row][col] = transposed[row][col]
        inverse[row][3] = inverse_translation[row]
    return inverse


def _reach_class(distance_m: float, nominal_reach_m: float) -> str:
    if distance_m <= nominal_reach_m:
        return "inside_nominal_reach_sphere"
    return "outside_nominal_reach_sphere"


def _aabb(center: list[float], size: list[float]) -> dict[str, list[float]]:
    half = [float(value) * 0.5 for value in size]
    return {
        "min_m": [float(center[index]) - half[index] for index in range(3)],
        "max_m": [float(center[index]) + half[index] for index in range(3)],
    }


def _oriented_aabb(matrix: list[list[float]], size: list[float]) -> dict[str, list[float]]:
    half = [float(value) * 0.5 for value in size]
    lows = [math.inf, math.inf, math.inf]
    highs = [-math.inf, -math.inf, -math.inf]
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                point = [sx * half[0], sy * half[1], sz * half[2], 1.0]
                world = [sum(matrix[row][col] * point[col] for col in range(4)) for row in range(3)]
                for axis in range(3):
                    lows[axis] = min(lows[axis], world[axis])
                    highs[axis] = max(highs[axis], world[axis])
    return {"min_m": lows, "max_m": highs}


def _reject_forbidden_keys(value: object) -> None:
    if isinstance(value, dict):
        for key, child in value.items():
            if key in FORBIDDEN_KEYS:
                raise ValueError("forbidden field " + key)
            _reject_forbidden_keys(child)
    elif isinstance(value, list):
        for child in value:
            _reject_forbidden_keys(child)


def _require_finite(values: list[float]) -> None:
    for value in values:
        if not math.isfinite(float(value)):
            raise ValueError("non-finite geometry")


def _xyz(values: list[float]) -> list[float]:
    return [float(value) for value in values]


def _add(left: list[float], right: list[float]) -> list[float]:
    return [left[index] + right[index] for index in range(3)]


def _distance(left: list[float], right: list[float]) -> float:
    return math.dist(left, right)
