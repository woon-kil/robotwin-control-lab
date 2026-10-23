"""MOVE_JOINT pre-execution checks. SAFE_TO_ATTEMPT는 검사 통과만 뜻한다."""

from __future__ import annotations

import json
import math
from pathlib import Path

from controller.virtual_pmac import JointCommand, RobotState
from core.kinematics.fk import forward_kinematics
from core.kinematics.limits import limit_violations
from core.model.types import RobotModel
from core.model.ur10e import load_ur10e_model
from validator.scene import CollisionScene, prohibited_zone
from workcell.layout import resolve_workcell

CONFIG_PATH = Path(__file__).resolve().parent / "config" / "default.json"
MOVE_JOINT = "MOVE_JOINT"

REQUIRED_CHECKS = (
    "finite_input",
    "command_support",
    "target_validity",
    "joint_limit",
    "reachability",
    "trajectory",
    "fk_trajectory",
    "robot_environment_collision",
    "tool_environment_collision",
    "tool_robot_collision",
    "self_collision",
    "safety_zone",
)
OPTIONAL_CHECKS = ("tcp_alignment", "ik_feasibility")
_GEOMETRY_CHECKS = (
    "fk_trajectory",
    "robot_environment_collision",
    "tool_environment_collision",
    "tool_robot_collision",
    "self_collision",
    "safety_zone",
)
CHECK_RESULTS = ("PASS", "FAIL", "NOT_APPLICABLE", "NOT_EVALUATED")
DECISIONS = ("SAFE_TO_ATTEMPT", "REJECT")

_SCENE: CollisionScene | None = None
_SCENE_KEY: int | None = None


def load_validator_config(path: Path | None = None) -> dict:
    source = CONFIG_PATH if path is None else path
    payload = json.loads(source.read_text(encoding="utf-8"))
    count = payload["trajectory_sample_count"]
    if isinstance(count, bool) or not isinstance(count, int) or count < 2:
        raise ValueError("trajectory_sample_count must be an integer >= 2")
    return payload


def validate(
    command: JointCommand,
    state: RobotState,
    workcell: dict,
    config: dict | None = None,
    model: RobotModel | None = None,
) -> dict:
    try:
        active = load_validator_config() if config is None else config
        robot = load_ur10e_model() if model is None else model
        resolved = workcell if "bolt_panel_world" in workcell else resolve_workcell(workcell)
        report = _validate(command, state, resolved, active, robot)
        _assert_finite_tree(report)
        return report
    except Exception as exc:
        return _internal_error(command, state, exc)


def joint_limit_rows(model: RobotModel, joints: tuple[float, ...]) -> tuple[dict, ...]:
    rows = []
    for name, value in zip(model.actuated_joint_order, joints, strict=True):
        limits = model.joint_by_name(name).limits
        if limits is None:
            raise ValueError(name + " has no position limit")
        rows.append(
            {
                "joint": name,
                "lower_limit": limits.lower,
                "upper_limit": limits.upper,
                "commanded_value": value,
                "margin_to_lower": value - limits.lower,
                "margin_to_upper": limits.upper - value,
                "inside": value >= limits.lower and value <= limits.upper,
            }
        )
    return tuple(rows)


def interpolate(start: tuple[float, ...], target: tuple[float, ...], count: int) -> tuple[tuple[float, ...], ...]:
    if count < 2:
        raise ValueError("trajectory_sample_count must include both endpoints")
    last = count - 1
    samples = []
    for index in range(count):
        scale = index / last
        samples.append(tuple(start[axis] + scale * (target[axis] - start[axis]) for axis in range(len(start))))
    return tuple(samples)


def trajectory_limit_metrics(model: RobotModel, samples: tuple[tuple[float, ...], ...]) -> dict:
    first = None
    violating = 0
    for index, sample in enumerate(samples):
        if limit_violations(model, sample):
            violating += 1
            if first is None:
                first = index
    return {
        "sample_count": len(samples),
        "violating_sample_count": violating,
        "first_violation_index": first,
    }


def _validate(command: JointCommand, state: RobotState, resolved: dict, config: dict, model: RobotModel) -> dict:
    count = int(config["trajectory_sample_count"])
    joint_count = len(model.actuated_joint_order)
    start, start_code = _read_vector(getattr(state, "joint_position", None), joint_count)
    target, target_code = _read_vector(getattr(command, "joint_targets", None), joint_count)
    command_type = getattr(command, "command_type", None)
    checks: list[dict] = []
    non_finite = start_code == "NON_FINITE" or target_code == "NON_FINITE"
    invalid = start_code == "TARGET_INVALID" or target_code == "TARGET_INVALID"
    checks.append(
        _check(
            "finite_input",
            "FAIL" if non_finite else "NOT_EVALUATED" if invalid else "PASS",
            "NON_FINITE" if non_finite else None,
            {"start": start_code, "target": target_code},
        )
    )
    supported = command_type == MOVE_JOINT
    checks.append(
        _check(
            "command_support",
            "PASS" if supported else "FAIL",
            None if supported else "UNSUPPORTED_COMMAND",
            {"command_type": command_type, "supported": [MOVE_JOINT]},
        )
    )
    if invalid or non_finite or start is None or target is None:
        checks.append(
            _check(
                "target_validity",
                "FAIL" if invalid else "NOT_EVALUATED",
                "TARGET_INVALID" if invalid else None,
                {},
            )
        )
        checks.extend(_not_evaluated(("joint_limit", "reachability", "trajectory", *_GEOMETRY_CHECKS)))
    else:
        checks.append(_check("target_validity", "PASS", None, {"joint_set": list(model.actuated_joint_order)}))
        rows = joint_limit_rows(model, target)
        violated = [row["joint"] for row in rows if row["inside"] is False]
        checks.append(
            _check(
                "joint_limit",
                "FAIL" if violated else "PASS",
                "JOINT_LIMIT" if violated else None,
                {"joints": list(rows), "violating_joints": violated},
            )
        )
        checks.append(
            _check(
                "reachability",
                "FAIL" if violated else "PASS",
                "UNREACHABLE" if violated else None,
                {**_reachability_definition(), "within_joint_limits": not violated, "ik": "NOT_APPLICABLE"},
            )
        )
        if not supported:
            checks.extend(_not_evaluated(("trajectory", *_GEOMETRY_CHECKS)))
        else:
            samples = interpolate(start, target, count)
            limit_metrics = trajectory_limit_metrics(model, samples)
            trajectory_fail = limit_metrics["violating_sample_count"] > 0
            checks.append(
                _check(
                    "trajectory",
                    "FAIL" if trajectory_fail else "PASS",
                    "TRAJECTORY_INVALID" if trajectory_fail else None,
                    {
                        **limit_metrics,
                        "model": config["trajectory_model"],
                        "formula": config["trajectory_formula"],
                        "parameter": config["trajectory_parameter"],
                        "joint_limit_set_is_convex": True,
                    },
                )
            )
            if trajectory_fail:
                checks.extend(_not_evaluated(_GEOMETRY_CHECKS))
            else:
                checks.extend(_geometry_checks(model, samples, resolved, config))
    checks.extend(_optional_checks(resolved))
    return _finish(command, state, checks, config)


def _geometry_checks(model: RobotModel, samples: tuple[tuple[float, ...], ...], resolved: dict, config: dict) -> list[dict]:
    scene = _scene_for(resolved)
    tcp_path = []
    totals = {"aabb_separated_pairs": 0, "narrow_phase_pairs": 0, "triangle_tests": 0}
    robot_hit = None
    tool_hit = None
    tool_robot_hit = None
    self_hit = None
    zone_hit = None
    zone = None
    for index, sample in enumerate(samples):
        posed = scene.evaluate(model, sample, resolved, config)
        tcp_path.append(posed["tcp_m"])
        zone = posed["prohibited_zone_m"]
        for key in totals:
            totals[key] += int(posed[key])
        if robot_hit is None and posed["robot_hits"]:
            robot_hit = _collision_metrics(index, len(samples), sample, posed["tcp_m"], posed["robot_hits"], totals)
        if tool_hit is None and posed["tool_hits"]:
            tool_hit = _collision_metrics(index, len(samples), sample, posed["tcp_m"], posed["tool_hits"], totals)
        if tool_robot_hit is None and posed["tool_robot_hits"]:
            tool_robot_hit = _collision_metrics(index, len(samples), sample, posed["tcp_m"], posed["tool_robot_hits"], totals)
        if self_hit is None and posed["self_hits"]:
            self_hit = _collision_metrics(index, len(samples), sample, posed["tcp_m"], posed["self_hits"], totals)
        if zone_hit is None and posed["in_prohibited_zone"]:
            zone_hit = {
                "sample_index": index,
                "sample_count": len(samples),
                "tcp_m": posed["tcp_m"],
                "joint_state": list(sample),
            }
    reference = forward_kinematics(model, samples[0])
    fk_fail = _distance(tcp_path[0], _tcp_in_base(reference, resolved)) > 1e-9
    checks = [
        _check(
            "fk_trajectory",
            "FAIL" if fk_fail else "PASS",
            "VALIDATION_INTERNAL_ERROR" if fk_fail else None,
            {
                "sample_count": len(samples),
                "tcp_path_m": tcp_path,
                "source": "core.kinematics.fk.chain_frames",
                "tcp_matches_forward_kinematics_at_first_sample": not fk_fail,
            },
        )
    ]
    checks.append(_collision_check("robot_environment_collision", robot_hit, totals, len(samples)))
    checks.append(_collision_check("tool_environment_collision", tool_hit, totals, len(samples)))
    checks.append(_collision_check("tool_robot_collision", tool_robot_hit, totals, len(samples)))
    self_metrics = dict(self_hit or _clear_collision(len(samples), totals))
    self_metrics["adjacent_links_excluded"] = True
    self_metrics["label"] = "EVALUATED"
    checks.append(
        _check(
            "self_collision",
            "FAIL" if self_hit else "PASS",
            "COLLISION" if self_hit else None,
            self_metrics,
        )
    )
    zone_low, zone_high = zone if zone is not None else prohibited_zone(resolved, config)
    checks.append(
        _check(
            "safety_zone",
            "FAIL" if zone_hit else "PASS",
            "SAFETY_ZONE" if zone_hit else None,
            {
                "class": config["safety_zone"]["class"],
                "mode": config["safety_zone"]["mode"],
                "boundary": config["safety_zone"]["boundary"],
                "prohibited_min_m": list(zone_low),
                "prohibited_max_m": list(zone_high),
                "sample_count": len(samples),
                "violation": zone_hit,
            },
        )
    )
    return checks


def _collision_check(name: str, hit: dict | None, totals: dict, sample_count: int) -> dict:
    metrics = dict(hit or _clear_collision(sample_count, totals))
    metrics["method"] = (
        "AABB broadphase rejects separated pairs. Overlap uses triangle or OBB tests. "
        "AABB overlap alone is not a collision."
    )
    return _check(name, "FAIL" if hit else "PASS", "COLLISION" if hit else None, metrics)


def _collision_metrics(index: int, count: int, sample: tuple[float, ...], tcp: list[float], hits: list[dict], totals: dict) -> dict:
    return {
        "collision": True,
        "sample_index": index,
        "sample_count": count,
        "involved_objects": hits,
        "joint_state": list(sample),
        "tcp_m": tcp,
        **totals,
    }


def _clear_collision(count: int, totals: dict) -> dict:
    return {
        "collision": False,
        "sample_index": None,
        "sample_count": count,
        "involved_objects": [],
        "joint_state": None,
        "tcp_m": None,
        **totals,
    }


def _optional_checks(resolved: dict) -> list[dict]:
    targets = [item["id"] for item in resolved.get("bolt_targets_world", resolved.get("bolt_targets", []))]
    return [
        _check(
            "tcp_alignment",
            "NOT_APPLICABLE",
            None,
            {
                "alignment_angle_deg": None,
                "allowed_angle_deg": None,
                "parameter_class": "PROJECT TEST PARAMETER",
                "reason": "MOVE_JOINT has no bolt target pose. No joint mapping is defined for B01-B06.",
                "bolt_target_ids": targets,
            },
        ),
        _check(
            "ik_feasibility",
            "NOT_APPLICABLE",
            None,
            {"reason": "Cartesian IK is not implemented. IK success is not used as reachability."},
        ),
    ]


def _not_evaluated(names: tuple[str, ...]) -> list[dict]:
    return [_check(name, "NOT_EVALUATED", None, {}) for name in names]


def _finish(command: JointCommand, state: RobotState, checks: list[dict], config: dict) -> dict:
    by_name = {check["name"]: check for check in checks}
    ordered = [by_name[name] for name in REQUIRED_CHECKS + OPTIONAL_CHECKS]
    blocking: list[str] = []
    for check in ordered:
        if check["name"] in OPTIONAL_CHECKS and check["result"] == "NOT_APPLICABLE":
            continue
        if check["result"] == "FAIL" and check["reason_code"] is not None:
            blocking.append(str(check["reason_code"]))
        elif check["result"] != "PASS":
            blocking.append("NOT_EVALUATED" if check["result"] == "NOT_EVALUATED" else "VALIDATION_INTERNAL_ERROR")
    decision = "SAFE_TO_ATTEMPT" if not blocking else "REJECT"
    report = {
        "validation_id": "validation-" + str(getattr(command, "command_id", "missing")),
        "command_id": getattr(command, "command_id", None),
        "timestamp_s": float(getattr(state, "timestamp_s", 0.0) or 0.0),
        "decision": decision,
        "checks": ordered,
        "blocking_reason_codes": blocking,
        "summary": _summary(decision, ordered),
        "metrics": {
            "trajectory_sample_count": config["trajectory_sample_count"],
            "claim": config["claim"],
        },
        "claim_boundary": config["claim"],
    }
    return report


def _summary(decision: str, checks: list[dict]) -> str:
    if decision == "SAFE_TO_ATTEMPT":
        return "SAFE_TO_ATTEMPT — pre-execution checks passed"
    for check in checks:
        if check["result"] != "FAIL":
            continue
        metrics = check["metrics"]
        if check["name"] == "joint_limit":
            rows = [row for row in metrics.get("joints", []) if row.get("inside") is False]
            if rows:
                row = rows[0]
                if row["commanded_value"] > row["upper_limit"]:
                    excess = row["commanded_value"] - row["upper_limit"]
                    side = "upper"
                else:
                    excess = row["lower_limit"] - row["commanded_value"]
                    side = "lower"
                return f"REJECT — {row['joint']} exceeds {side} limit by {excess} rad"
        if check["reason_code"] == "COLLISION":
            involved = metrics.get("involved_objects") or []
            if involved:
                pair = involved[0]
                index = metrics.get("sample_index")
                count = metrics.get("sample_count")
                return f"REJECT — {pair['a']} intersects {pair['b']} at sample {index}/{count}"
        if check["name"] == "safety_zone" and metrics.get("violation"):
            violation = metrics["violation"]
            return (
                "REJECT — TCP is inside the project test safety zone at sample "
                f"{violation['sample_index']}/{violation['sample_count']}"
            )
        if check["reason_code"] == "NON_FINITE":
            return "REJECT — non-finite joint value"
        if check["reason_code"] == "UNSUPPORTED_COMMAND":
            return "REJECT — unsupported command"
        if check["reason_code"] == "TRAJECTORY_INVALID":
            return (
                "REJECT — trajectory leaves joint limits at sample "
                f"{metrics.get('first_violation_index')}/{metrics.get('sample_count')}"
            )
        if check["reason_code"]:
            return "REJECT — " + str(check["reason_code"])
    return "REJECT — required check was not evaluated"


def _check(name: str, result: str, reason_code: str | None, metrics: dict) -> dict:
    if result not in CHECK_RESULTS:
        raise ValueError("unknown check result")
    return {"name": name, "result": result, "reason_code": reason_code, "metrics": metrics}


def _reachability_definition() -> dict:
    return {
        "definition": "finite joint target, known joint set, within modeled joint limits",
        "ik_success_is_reachability": False,
    }


def _read_vector(values: object, expected: int) -> tuple[tuple[float, ...] | None, str | None]:
    if not isinstance(values, (list, tuple)) or isinstance(values, str):
        return None, "TARGET_INVALID"
    if len(values) != expected:
        return None, "TARGET_INVALID"
    parsed: list[float] = []
    for value in values:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return None, "TARGET_INVALID"
        number = float(value)
        if not math.isfinite(number):
            return None, "NON_FINITE"
        parsed.append(number)
    return tuple(parsed), None


def _tcp_in_base(pose: object, resolved: dict) -> list[float]:
    base = resolved["T_world_robot_base"]["xyz_m"]
    rpy = resolved["T_world_robot_base"]["rpy_rad"]
    if list(rpy) != [0.0, 0.0, 0.0]:
        raise ValueError("TCP comparison expects the pinned workcell base orientation")
    return [float(base[0]) + float(pose.x), float(base[1]) + float(pose.y), float(base[2]) + float(pose.z)]


def _distance(left: list[float], right: list[float]) -> float:
    return math.dist(left, right)


def _scene_for(resolved: dict) -> CollisionScene:
    global _SCENE, _SCENE_KEY
    key = id(resolved)
    if _SCENE is None or _SCENE_KEY != key:
        _SCENE = CollisionScene(resolved)
        _SCENE_KEY = key
    return _SCENE


def _internal_error(command: object, state: object, exc: Exception) -> dict:
    command_id = getattr(command, "command_id", None)
    return {
        "validation_id": "validation-" + str(command_id),
        "command_id": command_id,
        "timestamp_s": float(getattr(state, "timestamp_s", 0.0) or 0.0),
        "decision": "REJECT",
        "checks": [
            _check(
                "validation_internal",
                "FAIL",
                "VALIDATION_INTERNAL_ERROR",
                {"error_type": type(exc).__name__},
            )
        ],
        "blocking_reason_codes": ["VALIDATION_INTERNAL_ERROR"],
        "summary": "REJECT — validation internal error",
        "metrics": {"claim": "PROJECT TEST VALIDATOR. SAFE_TO_ATTEMPT means pre-execution checks passed."},
        "claim_boundary": "PROJECT TEST VALIDATOR. SAFE_TO_ATTEMPT means pre-execution checks passed.",
    }


def _assert_finite_tree(value: object) -> None:
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("non-finite validator output")
        return
    if isinstance(value, dict):
        for child in value.values():
            _assert_finite_tree(child)
        return
    if isinstance(value, (list, tuple)):
        for child in value:
            _assert_finite_tree(child)
