"""P-008 evidence. 기존 evidence 디렉터리는 쓰지 않는다."""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
from pathlib import Path

from controller.evidence import DT_S, MAX_STEPS
from controller.virtual_pmac import COMPLETION_TOLERANCE_RAD, JointCommand, RobotState, VirtualPmacController
from core.kinematics.configurations import baseline_configurations
from core.model.ur10e import SOURCE_COMMIT, load_ur10e_model
from validator.gate import ExecutionGate
from validator.scene import CollisionScene
from validator.validate import (
    OPTIONAL_CHECKS,
    REQUIRED_CHECKS,
    joint_limit_rows,
    load_validator_config,
    validate,
)
from workcell.layout import resolve_workcell

SELECTED = ("A-zero", "B-positive-1", "C-mixed-1", "E-fixed-1", "E-fixed-3")
CLAIM = "PROJECT TEST VALIDATOR. SAFE_TO_ATTEMPT means pre-execution checks passed."


def write_evidence(directory: Path, run_pytest: bool = False) -> dict:
    payload = build_payload()
    if run_pytest:
        payload["regression_results"] = _pytest()
        payload["result"] = _overall(payload)
    directory.mkdir(parents=True, exist_ok=True)
    _write_json(directory / "validator_contract.json", payload["validator_contract"])
    _write_json(directory / "validator_config.json", payload["validator_config"])
    _write_json(directory / "positive_results.json", payload["positive_results"])
    _write_json(directory / "negative_results.json", payload["negative_results"])
    _write_json(directory / "joint_limit_results.json", payload["joint_limit_results"])
    _write_json(directory / "trajectory_results.json", payload["trajectory_results"])
    _write_json(directory / "collision_results.json", payload["collision_results"])
    _write_json(directory / "safety_zone_results.json", payload["safety_zone_results"])
    _write_json(directory / "alignment_results.json", payload["alignment_results"])
    _write_json(directory / "execution_gate_results.json", payload["execution_gate_results"])
    _write_json(directory / "determinism_results.json", payload["determinism_results"])
    _write_json(directory / "regression_results.json", payload["regression_results"])
    (directory / "summary.md").write_text(_summary_md(payload), encoding="utf-8")
    (directory / "test_results.txt").write_text(_test_text(payload), encoding="utf-8")
    return payload


def build_payload() -> dict:
    model = load_ur10e_model()
    resolved = resolve_workcell()
    config = load_validator_config()
    scene = CollisionScene(resolved)
    positives = _positives(model, resolved, config)
    negatives = _negatives(model, resolved, config, scene)
    limits = _joint_limits(model)
    determinism = _determinism(model, resolved, config, negatives)
    gate = _execution_gate(model, resolved, config, positives, negatives)
    payload = {
        "result": "PENDING_REGRESSION",
        "validator_contract": _contract(config),
        "validator_config": config,
        "positive_results": {"configurations": positives, "safe_count": _safe_count(positives)},
        "negative_results": {"cases": negatives},
        "joint_limit_results": limits,
        "trajectory_results": _named(positives, negatives, "trajectory"),
        "collision_results": _collision_section(positives, negatives),
        "safety_zone_results": _named(positives, negatives, "safety_zone"),
        "alignment_results": _named(positives, negatives, "tcp_alignment"),
        "execution_gate_results": gate,
        "determinism_results": determinism,
        "regression_results": {
            "pytest_ran": False,
            "isaac_rerun": False,
            "unity_rerun": False,
            "ros_evidence_rerun": False,
            "p003_stiffness_unchanged": 100000,
            "p004_stiffness_unchanged": 200000,
            "p004_damping_unchanged": 1000,
        },
        "non_finite_count": 0,
        "timeout_count": _timeouts(positives),
    }
    payload["non_finite_count"] = _count_non_finite(payload)
    payload["result"] = _overall(payload)
    return payload


def _positives(model, resolved, config) -> list[dict]:
    rows = {row["test_id"]: row["joint_vector_rad"] for row in baseline_configurations()}
    results = []
    for name in SELECTED:
        controller = _ready(model)
        before = controller.get_state()
        gate = ExecutionGate(controller)
        command = JointCommand(name, "MOVE_JOINT", rows[name])
        submit_before = gate.submit_count
        report = gate.consider(command, before, resolved, config)
        _finish_motion(controller, command, report)
        report["configuration"] = name
        report["start_q"] = list(before.joint_position)
        report["target_q"] = list(rows[name])
        report["submit_count_before"] = submit_before
        report["submit_count_after"] = gate.submit_count
        results.append(report)
    return results


def _negatives(model, resolved, config, scene: CollisionScene) -> list[dict]:
    zero = tuple(0.0 for _ in model.actuated_joint_order)
    lower = _outside_lower(model)
    upper = _outside_upper(model)
    tool_pose, tool_screen = _first_match(scene, model, resolved, config, _tool_panel)
    zone_pose, zone_screen = _first_match(scene, model, resolved, config, lambda screen: bool(screen["in_prohibited_zone"]))
    cases = [
        _case(model, resolved, config, "joint_lower_limit", zero, lower, "JOINT_LIMIT"),
        _case(model, resolved, config, "joint_upper_limit", zero, upper, "JOINT_LIMIT"),
        _case(model, resolved, config, "nan", zero, _replace(zero, 2, math.nan), "NON_FINITE"),
        _case(model, resolved, config, "positive_inf", zero, _replace(zero, 2, math.inf), "NON_FINITE"),
        _case(model, resolved, config, "negative_inf", zero, _replace(zero, 2, -math.inf), "NON_FINITE"),
        _case(model, resolved, config, "unsupported_command", zero, zero, "UNSUPPORTED_COMMAND", command_type="MOVE_TCP"),
        _case(model, resolved, config, "unknown_command", zero, zero, "UNSUPPORTED_COMMAND", command_type="UNKNOWN"),
        _case(model, resolved, config, "trajectory_joint_limit", zero, lower, "TRAJECTORY_INVALID"),
    ]
    if tool_pose is None:
        cases.append({"case": "tool_panel_collision", "found": False, "decision": "NOT_FOUND"})
    else:
        cases.append(
            _case(
                model,
                resolved,
                config,
                "tool_panel_collision",
                tool_pose,
                tool_pose,
                "COLLISION",
                screen=tool_screen,
            )
        )
    cases.append(_case(model, resolved, config, "canonical_path_collision", zero, _vector("E-fixed-1"), "COLLISION"))
    if zone_pose is None:
        cases.append({"case": "safety_zone", "found": False, "decision": "NOT_FOUND"})
    else:
        cases.append(
            _case(
                model,
                resolved,
                config,
                "safety_zone",
                zone_pose,
                zone_pose,
                "SAFETY_ZONE",
                screen=zone_screen,
            )
        )
    return cases


def _case(model, resolved, config, name, start, target, expected, command_type="MOVE_JOINT", screen=None) -> dict:
    controller = _ready(model)
    gate = ExecutionGate(controller)
    before = gate.submit_count
    state = _state(start)
    command = JointCommand(name, command_type, target)
    report = gate.consider(command, state, resolved, config)
    check_reason = _reason_for_expected(report, expected)
    return {
        "case": name,
        "input": {
            "start_q": None if any(not math.isfinite(value) for value in start) else list(start),
            "target_q": None if any(not math.isfinite(value) for value in target) else list(target),
            "command_type": command_type,
            "screen": screen,
        },
        "expected_reason": expected,
        "actual_reason": check_reason,
        "blocking_reason_codes": report["blocking_reason_codes"],
        "decision": report["decision"],
        "summary": report["summary"],
        "controller_submitted": report["controller_submitted"],
        "submit_count_before": before,
        "submit_count_after": gate.submit_count,
        "controller_joint_position_after": list(controller.get_state().joint_position),
        "matched_expected_reason": expected in report["blocking_reason_codes"],
    }


def _joint_limits(model) -> dict:
    rows = []
    order = model.actuated_joint_order
    for index, name in enumerate(order):
        limits = model.joint_by_name(name).limits
        base = [0.0 for _ in order]
        probes = {
            "inside_lower": limits.lower + 1e-4,
            "at_lower": limits.lower,
            "outside_lower": limits.lower - 1e-4,
            "inside_upper": limits.upper - 1e-4,
            "at_upper": limits.upper,
            "outside_upper": limits.upper + 1e-4,
        }
        for label, value in probes.items():
            joints = tuple(value if joint_index == index else item for joint_index, item in enumerate(base))
            measured = joint_limit_rows(model, joints)[index]
            rows.append(
                {
                    "joint": name,
                    "probe": label,
                    "commanded_value": value,
                    "lower_limit": measured["lower_limit"],
                    "upper_limit": measured["upper_limit"],
                    "margin_to_lower": measured["margin_to_lower"],
                    "margin_to_upper": measured["margin_to_upper"],
                    "inside": measured["inside"],
                    "inclusion": "value < lower or value > upper is outside. At-limit is inside.",
                }
            )
    return {"source_commit": SOURCE_COMMIT, "rows": rows}


def _determinism(model, resolved, config, negatives) -> dict:
    zero = tuple(0.0 for _ in model.actuated_joint_order)
    target = _vector("A-zero")
    first = validate(JointCommand("det-safe", "MOVE_JOINT", target), _state(zero), resolved, config, model)
    second = validate(JointCommand("det-safe", "MOVE_JOINT", target), _state(zero), resolved, config, model)
    collision = next(row for row in negatives if row["case"] == "canonical_path_collision")
    collision_target = _vector("E-fixed-1")
    third = validate(JointCommand("det-hit", "MOVE_JOINT", collision_target), _state(zero), resolved, config, model)
    fourth = validate(JointCommand("det-hit", "MOVE_JOINT", collision_target), _state(zero), resolved, config, model)
    return {
        "runs": 2,
        "safe_equal": _same(first, second),
        "collision_equal": _same(third, fourth),
        "compared": ["decision", "blocking_reason_codes", "checks", "summary", "metrics"],
        "collision_case": collision["case"],
    }


def _execution_gate(model, resolved, config, positives, negatives) -> dict:
    safe = next(row for row in positives if row["decision"] == "SAFE_TO_ATTEMPT")
    rejected = next(row for row in negatives if row["case"] == "canonical_path_collision")
    controller = _ready(model)
    gate = ExecutionGate(controller)
    safe_before = gate.submit_count
    safe_command = JointCommand("gate-safe", "MOVE_JOINT", tuple(safe["target_q"]))
    safe_report = gate.consider(safe_command, controller.get_state(), resolved, config)
    _finish_motion(controller, safe_command, safe_report)
    rejected_before = gate.submit_count
    rejected_command = JointCommand("gate-reject", "MOVE_JOINT", _vector("E-fixed-1"))
    rejected_report = gate.consider(rejected_command, controller.get_state(), resolved, config)
    return {
        "safe_case": {
            "configuration": safe["configuration"],
            "decision": safe_report["decision"],
            "submit_count_before": safe_before,
            "submit_count_after": rejected_before,
            "execution_result": safe_report.get("execution"),
        },
        "rejected_case": {
            "case": rejected["case"],
            "decision": rejected_report["decision"],
            "submit_count_before": rejected_before,
            "submit_count_after": gate.submit_count,
            "summary": rejected_report["summary"],
        },
    }


def _first_match(scene, model, resolved, config, predicate):
    grid = config["negative_search"]
    axes = [_axis(grid[name]) for name in (
        "shoulder_pan_rad",
        "shoulder_lift_rad",
        "elbow_rad",
        "wrist_1_rad",
        "wrist_2_rad",
        "wrist_3_rad",
    )]
    for pan in axes[0]:
        for lift in axes[1]:
            for elbow in axes[2]:
                for wrist_1 in axes[3]:
                    for wrist_2 in axes[4]:
                        for wrist_3 in axes[5]:
                            joints = (pan, lift, elbow, wrist_1, wrist_2, wrist_3)
                            screen = scene.screen(model, joints, resolved, config)
                            if predicate(screen):
                                return joints, screen
    return None, None


def _tool_panel(screen) -> bool:
    return any(item["b"] == "bolt_panel" for item in screen["tool_hits"])


def _finish_motion(controller: VirtualPmacController, command: JointCommand, report: dict) -> None:
    if report["decision"] != "SAFE_TO_ATTEMPT":
        report["execution"] = None
        return
    steps = 0
    state = controller.get_state()
    while steps < MAX_STEPS:
        state = controller.step(DT_S)
        steps += 1
        if state.execution_state == "COMPLETED":
            break
    report["execution"] = {
        "steps": steps,
        "completed": state.execution_state == "COMPLETED",
        "execution_state": state.execution_state,
        "controller_state": state.controller_state,
        "max_joint_error_rad": max(abs(value) for value in state.joint_error),
        "trace": list(controller.trace(command.command_id)),
        "timeout": state.execution_state != "COMPLETED",
    }


def _ready(model) -> VirtualPmacController:
    controller = VirtualPmacController(model)
    controller.connect()
    controller.servo_on()
    return controller


def _state(joints: tuple[float, ...]) -> RobotState:
    zeros = tuple(0.0 for _ in joints)
    return RobotState(0.0, "ur10e", "READY", joints, zeros, zeros, zeros, {}, None, None, None, None, None)


def _outside_lower(model) -> tuple[float, ...]:
    limits = model.joint_by_name(model.actuated_joint_order[2]).limits
    return _replace(tuple(0.0 for _ in model.actuated_joint_order), 2, limits.lower - 0.2)


def _outside_upper(model) -> tuple[float, ...]:
    limits = model.joint_by_name(model.actuated_joint_order[0]).limits
    return _replace(tuple(0.0 for _ in model.actuated_joint_order), 0, limits.upper + 0.2)


def _replace(values: tuple[float, ...], index: int, value: float) -> tuple[float, ...]:
    row = list(values)
    row[index] = value
    return tuple(row)


def _vector(name: str) -> tuple[float, ...]:
    for row in baseline_configurations():
        if row["test_id"] == name:
            return tuple(row["joint_vector_rad"])
    raise ValueError(name)


def _axis(spec: dict) -> tuple[float, ...]:
    count = int(round((float(spec["stop"]) - float(spec["start"])) / float(spec["step"])))
    return tuple(float(spec["start"]) + index * float(spec["step"]) for index in range(count + 1))


def _reason_for_expected(report: dict, expected: str) -> str | None:
    if expected in report["blocking_reason_codes"]:
        return expected
    codes = report["blocking_reason_codes"]
    return codes[0] if codes else None


def _named(positives, negatives, check_name: str) -> dict:
    return {
        "positive": [_check_view(row, check_name) for row in positives],
        "negative": [_negative_view(row, check_name) for row in negatives],
    }


def _collision_section(positives, negatives) -> dict:
    names = (
        "robot_environment_collision",
        "tool_environment_collision",
        "tool_robot_collision",
        "self_collision",
    )
    return {"checks": {name: _named(positives, negatives, name) for name in names}}


def _check_view(row: dict, check_name: str) -> dict:
    checks = {item["name"]: item for item in row.get("checks", [])}
    check = checks.get(check_name)
    return {
        "configuration": row.get("configuration") or row.get("case"),
        "decision": row.get("decision"),
        "check": _shrink(check),
    }


def _negative_view(row: dict, check_name: str) -> dict:
    return {
        "case": row.get("case"),
        "decision": row.get("decision"),
        "actual_reason": row.get("actual_reason"),
        "summary": row.get("summary"),
        "matched_expected_reason": row.get("matched_expected_reason"),
    }


def _shrink(check: dict | None) -> dict | None:
    if check is None:
        return None
    metrics = dict(check["metrics"])
    path = metrics.get("tcp_path_m")
    if isinstance(path, list) and len(path) > 2:
        metrics["tcp_path_m"] = {"first": path[0], "last": path[-1], "count": len(path)}
    return {
        "name": check["name"],
        "result": check["result"],
        "reason_code": check["reason_code"],
        "metrics": metrics,
    }


def _same(left: dict, right: dict) -> bool:
    keys = ("decision", "blocking_reason_codes", "checks", "summary", "metrics")
    return json.dumps({key: left[key] for key in keys}, sort_keys=True) == json.dumps(
        {key: right[key] for key in keys}, sort_keys=True
    )


def _safe_count(rows: list[dict]) -> int:
    return sum(1 for row in rows if row["decision"] == "SAFE_TO_ATTEMPT" and _completed(row))


def _completed(row: dict) -> bool:
    execution = row.get("execution") or {}
    return (
        row.get("controller_submitted") is True
        and execution.get("completed") is True
        and execution.get("trace") == ["ACCEPTED", "EXECUTING", "COMPLETED"]
        and execution.get("max_joint_error_rad") <= COMPLETION_TOLERANCE_RAD
    )


def _timeouts(rows: list[dict]) -> int:
    return sum(1 for row in rows if (row.get("execution") or {}).get("timeout") is True)


def _overall(payload: dict) -> str:
    positives = payload["positive_results"]["configurations"]
    negatives = payload["negative_results"]["cases"]
    if payload["non_finite_count"] != 0 or payload["timeout_count"] != 0:
        return "FAIL"
    if _safe_count(positives) != len(SELECTED):
        return "FAIL"
    if any(row.get("controller_submitted") for row in negatives):
        return "FAIL"
    if any(row.get("decision") != "REJECT" for row in negatives):
        return "FAIL"
    if any(row.get("matched_expected_reason") is not True for row in negatives):
        return "FAIL"
    if not payload["determinism_results"]["safe_equal"] or not payload["determinism_results"]["collision_equal"]:
        return "FAIL"
    regression = payload["regression_results"]
    if regression.get("pytest_ran") and regression.get("returncode") != 0:
        return "FAIL"
    if not regression.get("pytest_ran"):
        return "PENDING_REGRESSION"
    gate = payload["execution_gate_results"]
    if gate["safe_case"]["submit_count_after"] != gate["safe_case"]["submit_count_before"] + 1:
        return "FAIL"
    if gate["rejected_case"]["submit_count_after"] != gate["rejected_case"]["submit_count_before"]:
        return "FAIL"
    return "PASS"


def _contract(config: dict) -> dict:
    return {
        "implementation": "validator.validate.PreExecutionValidator function validate; validator.gate.ExecutionGate",
        "input_contract": ["JointCommand", "RobotState", "WorkcellState", "ValidatorConfig"],
        "output_contract": ["decision", "checks", "blocking_reason_codes", "metrics", "summary"],
        "decision_vocabulary": ["SAFE_TO_ATTEMPT", "REJECT"],
        "check_vocabulary": ["PASS", "FAIL", "NOT_APPLICABLE", "NOT_EVALUATED"],
        "reason_codes": [
            "TARGET_INVALID",
            "NON_FINITE",
            "UNSUPPORTED_COMMAND",
            "JOINT_LIMIT",
            "UNREACHABLE",
            "TCP_ALIGNMENT",
            "TRAJECTORY_INVALID",
            "COLLISION",
            "SAFETY_ZONE",
            "VALIDATION_INTERNAL_ERROR",
            "NOT_EVALUATED",
        ],
        "required_checks": list(REQUIRED_CHECKS),
        "optional_checks": list(OPTIONAL_CHECKS),
        "fail_closed_rule": "unknown, invalid, non-finite, missing data, unsupported, required NOT_EVALUATED, and internal errors REJECT. Unknown results are not PASS.",
        "decision_rule": "Any required FAIL or NOT_EVALUATED rejects. Optional NOT_APPLICABLE is allowed. All required PASS means SAFE_TO_ATTEMPT.",
        "reachability": "MOVE_JOINT baseline: finite joint target, known joint set, within modeled joint limits. IK success is not reachability.",
        "trajectory": config["trajectory_model"],
        "joint_limit_rule": config["joint_limit_rule"],
        "safety_zone_class": config["safety_zone"]["class"],
        "claim_boundary": CLAIM,
        "not_claims": config["not_claims"],
        "architecture_notes": [
            "REQ-026 bolt vocabulary uses REJECTED. This MOVE_JOINT gate uses REJECT. REQUIREMENTS.md was not edited.",
            "IK feasibility and TCP alignment are NOT_APPLICABLE. NOT_APPLICABLE is not PASS.",
            "B01-B06 have no joint configuration mapping.",
            "Self-collision is evaluated. Adjacent collision links are excluded. A separated AABB is a non-intersection, not a stub PASS.",
            "Geometry checks are NOT_EVALUATED when the joint command is already non-finite or outside limits. That still REJECTS.",
            "The ROS 2 adapter was not modified and does not contain this validator.",
            "ExecutionGate is outside VirtualPmacController.",
        ],
    }


def _pytest() -> dict:
    root = Path(__file__).resolve().parents[1]
    env = os.environ.copy()
    env["PYTHONPATH"] = str(root)
    completed = subprocess.run(
        [sys.executable, "-m", "pytest", "tests/unit", "-q"],
        cwd=root,
        env=env,
        text=True,
        capture_output=True,
        check=False,
    )
    return {
        "pytest_ran": True,
        "command": "PYTHONPATH=. python3 -m pytest tests/unit -q",
        "returncode": completed.returncode,
        "stdout": completed.stdout,
        "stderr": completed.stderr,
        "isaac_rerun": False,
        "unity_rerun": False,
        "ros_evidence_rerun": False,
    }


def _count_non_finite(value: object) -> int:
    if isinstance(value, float):
        return 0 if math.isfinite(value) else 1
    if isinstance(value, dict):
        return sum(_count_non_finite(item) for item in value.values())
    if isinstance(value, (list, tuple)):
        return sum(_count_non_finite(item) for item in value)
    return 0


def _summary_md(payload: dict) -> str:
    positives = payload["positive_results"]["configurations"]
    lines = [
        "# P-008",
        "",
        CLAIM,
        "이 결과는 체결 성공, 산업 안전 인증, ISO safety zone, human protection zone이 아니다.",
        "",
        "Result",
        payload["result"],
        "",
        "Canonical",
        f"{payload['positive_results']['safe_count']}/{len(SELECTED)}",
        "",
    ]
    for row in positives:
        lines.append(f"{row['configuration']}: {row['decision']} — {row['summary']}")
    lines.extend(
        [
            "",
            "Timeouts",
            str(payload["timeout_count"]),
            "",
            "Non-finite",
            str(payload["non_finite_count"]),
            "",
            "P-009 started",
            "NO",
            "",
        ]
    )
    return "\n".join(lines)


def _test_text(payload: dict) -> str:
    regression = payload["regression_results"]
    stdout = regression.get("stdout", "")
    return "\n".join(
        [
            f"RESULT {payload['result']}",
            f"safe {payload['positive_results']['safe_count']}/{len(SELECTED)}",
            f"timeouts {payload['timeout_count']}",
            f"non_finite {payload['non_finite_count']}",
            "pytest",
            stdout,
            regression.get("stderr", ""),
        ]
    )


def _write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    payload = write_evidence(root / "evidence" / "P-008", run_pytest=True)
    print("RESULT " + payload["result"])
    print(payload["positive_results"]["safe_count"])


if __name__ == "__main__":
    main()
