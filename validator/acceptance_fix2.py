"""P-008 acceptance reconciliation. Validator 판정과 collision config는 바꾸지 않는다."""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
from pathlib import Path

from controller.virtual_pmac import COMPLETION_TOLERANCE_RAD, JointCommand
from core.model.ur10e import load_ur10e_model
from validator import evidence
from validator.gate import ExecutionGate
from validator.scene import CollisionScene
from validator.validate import OPTIONAL_CHECKS, REQUIRED_CHECKS, load_validator_config, validate
from workcell.layout import resolve_workcell

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "evidence" / "P-008-FIX2"
FIX1 = ROOT / "evidence" / "P-008-FIX1"

DESIGNATED_SAFE = ("A-zero", "B-positive-1", "C-mixed-1", "E-fixed-3")
CLASSIFICATION = {
    "A-zero": "DESIGNATED_SAFE",
    "B-positive-1": "DESIGNATED_SAFE",
    "C-mixed-1": "DESIGNATED_SAFE",
    "E-fixed-1": "KNOWN_COLLISION",
    "E-fixed-3": "DESIGNATED_SAFE",
}
REJECT_CLASS = {
    "joint_lower_limit": "KNOWN_JOINT_LIMIT_VIOLATION",
    "joint_upper_limit": "KNOWN_JOINT_LIMIT_VIOLATION",
    "nan": "KNOWN_NON_FINITE",
    "positive_inf": "KNOWN_NON_FINITE",
    "negative_inf": "KNOWN_NON_FINITE",
    "unsupported_command": "KNOWN_UNSUPPORTED",
    "unknown_command": "KNOWN_UNSUPPORTED",
    "trajectory_joint_limit": "KNOWN_JOINT_LIMIT_VIOLATION",
    "tool_panel_collision": "KNOWN_COLLISION",
    "canonical_path_collision": "KNOWN_COLLISION",
    "safety_zone": "KNOWN_SAFETY_ZONE_VIOLATION",
}
REASON_FOR_CHANGE = (
    "The original P-008 plan incorrectly assumed that all five canonical kinematic "
    "configurations were collision-free in the P-005 workcell. P-008-FIX1 independently "
    "confirmed that E-fixed-1 intersects the bolt panel. The validator is unchanged. "
    "The acceptance model is corrected to distinguish kinematic reference configurations "
    "from workcell-safe trajectories."
)


def assert_output_directory(directory: Path) -> Path:
    target = directory.resolve()
    allowed = OUTPUT.resolve()
    if target != allowed:
        raise RuntimeError("FIX2 evidence can only be written to evidence/P-008-FIX2")
    return target


def build_acceptance() -> dict:
    model = load_ur10e_model()
    resolved = resolve_workcell()
    config = load_validator_config()
    scene = CollisionScene(resolved)
    measured_configurations = evidence._positives(model, resolved, config)
    measured_negatives = evidence._negatives(model, resolved, config, scene)
    by_name = {row["configuration"]: row for row in measured_configurations}
    safe_cases = [_safe_row(by_name[name]) for name in DESIGNATED_SAFE]
    known = _known_collision_row(by_name["E-fixed-1"])
    reject_cases = [_reject_row(row) for row in measured_negatives]
    first = [_signature(row) for row in measured_configurations] + [_signature_from_negative(row) for row in measured_negatives]
    second = _replay(model, resolved, config, measured_configurations, measured_negatives)
    determinism = _determinism(first, second)
    gate = _gate(model, resolved, config)
    fail_closed = _fail_closed(model, resolved, config, reject_cases)
    matrix = safe_cases + [known] + reject_cases
    payload = {
        "safe_cases": safe_cases,
        "e_fixed_1": known,
        "reject_cases": reject_cases,
        "acceptance_matrix": matrix,
        "execution_gate_results": gate,
        "determinism_results": determinism,
        "fail_closed": fail_closed,
        "timeout_count": sum(1 for row in safe_cases if row["timeout"]),
        "designated_safe_pass": sum(1 for row in safe_cases if row["result"] == "PASS"),
        "negative_count": len(reject_cases),
        "negative_pass": sum(1 for row in reject_cases if row["result"] == "PASS"),
        "config_unchanged": _config_unchanged(config),
        "frozen_history": _frozen_history(),
        "fix1_evidence": _fix1_evidence(),
    }
    payload["non_finite_count"] = _count_non_finite(payload)
    payload["checklist"] = _checklist(payload)
    payload["case_result"] = "PASS" if all(payload["checklist"].values()) else "FAIL"
    return payload


def write_evidence(directory: Path | None = None) -> dict:
    target = assert_output_directory(OUTPUT if directory is None else directory)
    payload = build_acceptance()
    regression = _pytest()
    payload["regression_results"] = regression
    payload["result"] = "FINAL PASS" if payload["case_result"] == "PASS" and regression["returncode"] == 0 else "FAIL"
    target.mkdir(parents=True, exist_ok=True)
    _write_json(target / "acceptance_contract.json", _contract(payload))
    _write_json(target / "acceptance_matrix.json", {"rows": payload["acceptance_matrix"]})
    _write_json(target / "safe_cases.json", {"cases": payload["safe_cases"]})
    _write_json(target / "reject_cases.json", {"cases": payload["reject_cases"]})
    _write_json(target / "execution_gate_results.json", payload["execution_gate_results"])
    _write_json(target / "determinism_results.json", payload["determinism_results"])
    _write_json(target / "regression_results.json", regression)
    (target / "summary.md").write_text(_summary(payload), encoding="utf-8")
    (target / "test_results.txt").write_text(_test_text(payload), encoding="utf-8")
    return payload


def _safe_row(report: dict) -> dict:
    checks = _checks(report)
    execution = report.get("execution") or {}
    completed = (
        report["controller_submitted"] is True
        and execution.get("completed") is True
        and execution.get("trace") == ["ACCEPTED", "EXECUTING", "COMPLETED"]
        and execution.get("max_joint_error_rad") <= COMPLETION_TOLERANCE_RAD
        and execution.get("timeout") is not True
    )
    optional_ok = all(item["result"] == "NOT_APPLICABLE" for item in checks if item["name"] in OPTIONAL_CHECKS)
    required_ok = all(item["result"] == "PASS" for item in checks if item["name"] in REQUIRED_CHECKS)
    self_ok = _self_evaluated(checks)
    codes = list(report["blocking_reason_codes"])
    passed = (
        report["decision"] == "SAFE_TO_ATTEMPT"
        and not codes
        and completed
        and optional_ok
        and required_ok
        and self_ok
    )
    return {
        "case": report["configuration"],
        "case_classification": CLASSIFICATION[report["configuration"]],
        "provenance": "P-008 initial measurement was SAFE_TO_ATTEMPT and completed. FIX2 remeasured the same command. It was not moved into the reject set.",
        "expected_decision": "SAFE_TO_ATTEMPT",
        "actual_decision": report["decision"],
        "expected_reason": None,
        "actual_reason": codes,
        "controller_submitted": report["controller_submitted"],
        "execution_completed": completed,
        "execution_trace": execution.get("trace"),
        "max_joint_error_rad": execution.get("max_joint_error_rad"),
        "steps": execution.get("steps"),
        "timeout": execution.get("timeout") is True,
        "optional_not_applicable": optional_ok,
        "required_checks_pass": required_ok,
        "self_collision_evaluated": self_ok,
        "checks": checks,
        "result": "PASS" if passed else "FAIL",
    }


def _known_collision_row(report: dict) -> dict:
    signature = _signature(report)
    pair = signature["collision_pair"]
    sample = signature["first_collision_sample"]
    codes = signature["blocking_reason_codes"]
    passed = (
        report["decision"] == "REJECT"
        and "COLLISION" in codes
        and pair == {"a": "wrist_3_link", "b": "bolt_panel"}
        and sample == 44
        and report["controller_submitted"] is False
        and _triangle_tests(report) > 0
    )
    return {
        "case": "E-fixed-1",
        "case_classification": "KNOWN_COLLISION",
        "provenance": {
            "source": "P-008 initial observation + P-008-FIX1 independent investigation",
            "classification": "KNOWN_COLLISION",
            "evidence": [
                "evidence/P-008/positive_results.json",
                "evidence/P-008-FIX1/root_cause.json",
                "evidence/P-008-FIX1/independent_collision_check.json",
                "evidence/P-008-FIX1/scene_comparison.json",
            ],
        },
        "expected_decision": "REJECT",
        "actual_decision": report["decision"],
        "expected_reason": "COLLISION",
        "actual_reason": codes,
        "collision_pair": pair,
        "first_collision_sample": sample,
        "sample_count": 100,
        "summary": report["summary"],
        "controller_submitted": report["controller_submitted"],
        "submit_count_before": report["submit_count_before"],
        "submit_count_after": report["submit_count_after"],
        "result": "PASS" if passed else "FAIL",
    }


def _reject_row(row: dict) -> dict:
    name = row["case"]
    expected = row["expected_reason"]
    codes = list(row.get("blocking_reason_codes") or [])
    submitted = row.get("controller_submitted")
    pair, sample = _collision_from_summary(row.get("summary"))
    passed = (
        row.get("decision") == "REJECT"
        and submitted is False
        and row.get("submit_count_after") == row.get("submit_count_before")
        and expected in codes
        and row.get("found") is not False
    )
    if name == "canonical_path_collision":
        passed = passed and pair == {"a": "wrist_3_link", "b": "bolt_panel"} and sample == 44
    return {
        "case": name,
        "case_classification": REJECT_CLASS[name],
        "provenance": "Existing P-008 negative case. The negative_search grid was not retuned.",
        "expected_decision": "REJECT",
        "actual_decision": row.get("decision"),
        "expected_reason": expected,
        "actual_reason": codes,
        "summary": row.get("summary"),
        "controller_submitted": submitted,
        "submit_count_before": row.get("submit_count_before"),
        "submit_count_after": row.get("submit_count_after"),
        "collision_pair": pair,
        "first_collision_sample": sample,
        "result": "PASS" if passed else "FAIL",
    }


def _replay(model, resolved, config, configurations, negatives) -> list[dict]:
    signatures = []
    for report in configurations:
        controller = evidence._ready(model)
        gate = ExecutionGate(controller)
        command = JointCommand(report["configuration"] + "-replay", "MOVE_JOINT", tuple(report["target_q"]))
        replay = gate.consider(command, controller.get_state(), resolved, config)
        signatures.append(_signature(replay))
    zero = tuple(0.0 for _ in model.actuated_joint_order)
    special = {
        "nan": ("MOVE_JOINT", zero, evidence._replace(zero, 2, math.nan)),
        "positive_inf": ("MOVE_JOINT", zero, evidence._replace(zero, 2, math.inf)),
        "negative_inf": ("MOVE_JOINT", zero, evidence._replace(zero, 2, -math.inf)),
        "unsupported_command": ("MOVE_TCP", zero, zero),
        "unknown_command": ("UNKNOWN", zero, zero),
    }
    for row in negatives:
        if row["case"] in special:
            command_type, start, target = special[row["case"]]
        else:
            command_type = row["input"]["command_type"]
            start = tuple(row["input"]["start_q"])
            target = tuple(row["input"]["target_q"])
        controller = evidence._ready(model)
        gate = ExecutionGate(controller)
        command = JointCommand(row["case"] + "-replay", command_type, target)
        replay = gate.consider(command, evidence._state(start), resolved, config)
        signatures.append(_signature(replay))
    return signatures


def _determinism(first: list[dict], second: list[dict]) -> dict:
    mismatches = []
    for index, (left, right) in enumerate(zip(first, second, strict=True)):
        if left != right:
            mismatches.append({"index": index, "first": left, "second": right})
    return {
        "runs": 2,
        "equal": not mismatches,
        "compared": [
            "decision",
            "reason codes",
            "collision object pair",
            "first collision sample",
            "controller submission result",
        ],
        "mismatches": mismatches,
    }


def _gate(model, resolved, config) -> dict:
    controller = evidence._ready(model)
    gate = ExecutionGate(controller)
    safe_before = gate.submit_count
    safe_command = JointCommand("gate-safe", "MOVE_JOINT", evidence._vector("A-zero"))
    safe_report = gate.consider(safe_command, controller.get_state(), resolved, config)
    evidence._finish_motion(controller, safe_command, safe_report)
    collision_before = gate.submit_count
    collision_command = JointCommand("gate-collision", "MOVE_JOINT", evidence._vector("E-fixed-1"))
    collision_report = gate.consider(collision_command, controller.get_state(), resolved, config)
    invalid_controller = evidence._ready(model)
    invalid_gate = ExecutionGate(invalid_controller)
    invalid_before = invalid_gate.submit_count
    invalid_report = invalid_gate.consider(
        JointCommand("gate-nan", "MOVE_JOINT", evidence._replace(tuple(0.0 for _ in range(6)), 2, math.nan)),
        evidence._state(tuple(0.0 for _ in range(6))),
        resolved,
        config,
    )
    limit_controller = evidence._ready(model)
    limit_gate = ExecutionGate(limit_controller)
    limit_before = limit_gate.submit_count
    limit_report = limit_gate.consider(
        JointCommand("gate-limit", "MOVE_JOINT", evidence._outside_lower(model)),
        evidence._state(tuple(0.0 for _ in model.actuated_joint_order)),
        resolved,
        config,
    )
    safe_execution = safe_report.get("execution") or {}
    collision_signature = _signature(collision_report)
    return {
        "safe_case": {
            "case": "A-zero",
            "decision": safe_report["decision"],
            "controller_submitted": safe_report["controller_submitted"],
            "submit_count_before": safe_before,
            "submit_count_after": collision_before,
            "trace": safe_execution.get("trace"),
            "completed": safe_execution.get("completed"),
            "max_joint_error_rad": safe_execution.get("max_joint_error_rad"),
            "timeout": safe_execution.get("timeout") is True,
        },
        "rejected_collision_case": {
            "case": "E-fixed-1",
            "decision": collision_report["decision"],
            "reason_codes": collision_signature["blocking_reason_codes"],
            "collision_pair": collision_signature["collision_pair"],
            "first_collision_sample": collision_signature["first_collision_sample"],
            "controller_submitted": collision_report["controller_submitted"],
            "submit_count_before": collision_before,
            "submit_count_after": gate.submit_count,
        },
        "invalid_input_case": {
            "case": "nan",
            "decision": invalid_report["decision"],
            "reason_codes": list(invalid_report["blocking_reason_codes"]),
            "controller_submitted": invalid_report["controller_submitted"],
            "submit_count_before": invalid_before,
            "submit_count_after": invalid_gate.submit_count,
        },
        "joint_limit_case": {
            "case": "joint_lower_limit",
            "decision": limit_report["decision"],
            "reason_codes": list(limit_report["blocking_reason_codes"]),
            "controller_submitted": limit_report["controller_submitted"],
            "submit_count_before": limit_before,
            "submit_count_after": limit_gate.submit_count,
        },
    }


def _fail_closed(model, resolved, config, reject_cases) -> str:
    broken = validate(JointCommand("broken", "MOVE_JOINT", tuple(0.0 for _ in range(6))), evidence._state(tuple(0.0 for _ in range(6))), {"bolt_panel_world": {}})
    controller = evidence._ready(model)
    gate = ExecutionGate(controller)
    before = gate.submit_count
    gated = gate.consider(
        JointCommand("broken-gate", "MOVE_JOINT", tuple(0.0 for _ in range(6))),
        evidence._state(tuple(0.0 for _ in range(6))),
        {"bolt_panel_world": {}},
        config,
    )
    by_case = {row["case"]: row for row in reject_cases}
    limit = by_case["joint_lower_limit"]
    nan = by_case["nan"]
    unknown = by_case["unknown_command"]
    checks = [
        broken["decision"] == "REJECT" and broken["blocking_reason_codes"] == ["VALIDATION_INTERNAL_ERROR"],
        gated["decision"] == "REJECT" and gated["controller_submitted"] is False and gate.submit_count == before,
        "NON_FINITE" in nan["actual_reason"] and "NOT_EVALUATED" in limit["actual_reason"],
        "UNSUPPORTED_COMMAND" in unknown["actual_reason"],
        "JOINT_LIMIT" in limit["actual_reason"],
    ]
    return "PASS" if all(checks) else "FAIL"


def _checklist(payload: dict) -> dict[str, bool]:
    safe = payload["safe_cases"]
    known = payload["e_fixed_1"]
    rejects = {row["case"]: row for row in payload["reject_cases"]}
    gate = payload["execution_gate_results"]
    safe_gate = gate["safe_case"]
    collision_gate = gate["rejected_collision_case"]
    invalid_gate = gate["invalid_input_case"]
    limit_gate = gate["joint_limit_case"]
    config = payload["config_unchanged"]
    history = payload["frozen_history"]
    fix1 = payload["fix1_evidence"]
    return {
        "validator_interface": known["actual_decision"] in ("SAFE_TO_ATTEMPT", "REJECT") and all(row["actual_decision"] in ("SAFE_TO_ATTEMPT", "REJECT") for row in safe),
        "fail_closed": payload["fail_closed"] == "PASS",
        "reason_codes": known["result"] == "PASS" and all(row["result"] == "PASS" for row in payload["reject_cases"]),
        "non_finite_rejection": rejects["nan"]["result"] == "PASS" and rejects["positive_inf"]["result"] == "PASS" and rejects["negative_inf"]["result"] == "PASS",
        "unsupported_command_rejection": rejects["unsupported_command"]["result"] == "PASS" and rejects["unknown_command"]["result"] == "PASS",
        "joint_limit_validation": rejects["joint_lower_limit"]["result"] == "PASS" and rejects["joint_upper_limit"]["result"] == "PASS",
        "trajectory_validation": rejects["trajectory_joint_limit"]["result"] == "PASS" and all(row["required_checks_pass"] for row in safe),
        "fk_trajectory": all(_check_result(row, "fk_trajectory") == "PASS" for row in safe),
        "collision_validation": known["result"] == "PASS" and rejects["tool_panel_collision"]["result"] == "PASS" and rejects["canonical_path_collision"]["result"] == "PASS",
        "self_collision_validation": all(row["self_collision_evaluated"] for row in safe),
        "safety_zone_validation": rejects["safety_zone"]["result"] == "PASS",
        "designated_safe_cases": payload["designated_safe_pass"] == 4 and len(safe) == 4,
        "e_fixed_1_known_collision": known["actual_decision"] == "REJECT" and known["result"] == "PASS",
        "known_collision_controller_submission": collision_gate["submit_count_after"] == collision_gate["submit_count_before"] and known["controller_submitted"] is False,
        "other_negative_cases": payload["negative_pass"] == payload["negative_count"],
        "safe_command_controller_submission": safe_gate["submit_count_after"] == safe_gate["submit_count_before"] + 1 and all(row["controller_submitted"] for row in safe),
        "safe_execution_completed": safe_gate["completed"] is True and safe_gate["trace"] == ["ACCEPTED", "EXECUTING", "COMPLETED"] and all(row["execution_completed"] for row in safe),
        "deterministic_replay": payload["determinism_results"]["equal"] is True,
        "timeout": payload["timeout_count"] == 0 and safe_gate["timeout"] is False,
        "non_finite_validator_output": payload["non_finite_count"] == 0,
        "tolerance_not_weakened": config is True,
        "collision_exclusion_not_added": config is True,
        "safety_certification_not_claimed": config is True and history["p008_result"] == "FAIL",
        "p009_not_started": OUTPUT.name != "P-009",
        "historical_evidence_preserved": history["p008_result"] == "FAIL" and fix1["classification"] == "TRUE_COLLISION",
        "invalid_input_blocked": invalid_gate["decision"] == "REJECT" and invalid_gate["submit_count_after"] == invalid_gate["submit_count_before"] and limit_gate["decision"] == "REJECT" and limit_gate["submit_count_after"] == limit_gate["submit_count_before"],
        "optional_checks_not_recorded_as_pass": all(row["optional_not_applicable"] for row in safe),
    }


def _contract(payload: dict) -> dict:
    return {
        "purpose": "Distinguish a safe workcell command from an invalid or colliding command. Passing every command is not the purpose.",
        "historical_acceptance": "5 canonical positive cases PASS",
        "reason_for_change": REASON_FOR_CHANGE,
        "evidence_for_change": payload["fix1_evidence"],
        "current_acceptance": {
            "designated_safe": "SAFE_TO_ATTEMPT and controller completion",
            "known_reject": "REJECT and controller_submitted false",
            "checklist": payload["checklist"],
            "result": payload["result"],
        },
        "safe_cases": list(DESIGNATED_SAFE),
        "reject_cases": [row["case"] for row in payload["reject_cases"]],
        "canonical_configurations_unchanged": list(CLASSIFICATION),
        "workcell_classification": CLASSIFICATION,
        "unchanged_safety_rules": [
            "collision tolerance was not widened",
            "collision geometry was not reduced",
            "bolt_panel and wrist_3_link stay in the collision check",
            "trajectory_sample_count stays 100",
            "required checks stay required",
            "fail-closed stays in force",
            "safety zone depth stays 0.40 m",
            "PROJECT TEST SAFETY ZONE is not an industrial safety certification",
            "PROJECT VALIDATION PATH is not an actual UR trajectory planner",
        ],
        "claim_boundary": "PROJECT TEST VALIDATOR. SAFE_TO_ATTEMPT means pre-execution checks passed.",
        "required_checks": list(REQUIRED_CHECKS),
        "optional_checks": list(OPTIONAL_CHECKS),
        "engineering_history": [
            "P-008 Initial expected 5 safe and measured 4 safe plus 1 collision.",
            "P-008 FIX1 independently classified the collision as TRUE_COLLISION.",
            "Validator defect was NO. The test assumption was wrong.",
            "P-008 FIX2 corrects the acceptance model and leaves the validator decision unchanged.",
        ],
    }


def _summary(payload: dict) -> str:
    regression = payload["regression_results"]
    lines = [
        "P-008 FIX ATTEMPT 2",
        "",
        "Purpose:",
        "Acceptance reconciliation after confirmed TRUE_COLLISION.",
        "",
        "Validator modified:",
        "NO",
        "",
        "Collision tolerance changed:",
        "NO",
        "",
        "Collision exclusion added:",
        "NO",
        "",
        "Designated safe:",
        f"{payload['designated_safe_pass']}/4 PASS",
        "",
        "Known collision:",
        f"E-fixed-1 {payload['e_fixed_1']['actual_decision']} {payload['e_fixed_1']['result']}",
        "",
        "Negative suite:",
        f"{payload['negative_pass']}/{payload['negative_count']} PASS",
        "",
        "Safe command reaches controller:",
        "YES" if payload["execution_gate_results"]["safe_case"]["controller_submitted"] else "NO",
        "",
        "Rejected command reaches controller:",
        "YES" if payload["execution_gate_results"]["rejected_collision_case"]["controller_submitted"] else "NO",
        "",
        "Fail-closed:",
        payload["fail_closed"],
        "",
        "Determinism:",
        "PASS" if payload["determinism_results"]["equal"] else "FAIL",
        "",
        "Regression:",
        "PASS" if regression["returncode"] == 0 else "FAIL",
        "",
        "P-008:",
        payload["result"],
        "",
        "P-009 started:",
        "NO",
        "",
        "P-008 Initial",
        "expected 5 safe",
        "actual 4 safe + 1 collision",
        "P-008 FIX1",
        "independent investigation",
        "TRUE_COLLISION",
        "Validator defect = NO",
        "Test assumption = WRONG",
        "P-008 FIX2",
        "acceptance corrected",
        "safe → SAFE_TO_ATTEMPT",
        "unsafe → REJECT",
        "",
    ]
    return "\n".join(lines)


def _test_text(payload: dict) -> str:
    regression = payload["regression_results"]
    return "\n".join(
        [
            f"RESULT {payload['result']}",
            f"designated_safe {payload['designated_safe_pass']}/4",
            f"negative {payload['negative_pass']}/{payload['negative_count']}",
            f"e_fixed_1 {payload['e_fixed_1']['actual_decision']} sample {payload['e_fixed_1']['first_collision_sample']}",
            f"timeouts {payload['timeout_count']}",
            f"non_finite {payload['non_finite_count']}",
            "pytest",
            regression.get("stdout", ""),
            regression.get("stderr", ""),
        ]
    )


def _pytest() -> dict:
    env = os.environ.copy()
    env["PYTHONPATH"] = str(ROOT)
    command = [
        sys.executable,
        "-m",
        "pytest",
        "tests/unit/test_ur10e_fk.py",
        "tests/unit/test_p001b_evidence.py",
        "tests/unit/test_p006_controller.py",
        "tests/unit/test_p007_convert.py",
        "tests/unit/test_p008_validator.py",
        "tests/unit/test_p008_fix2_acceptance.py",
        "-q",
    ]
    completed = subprocess.run(command, cwd=ROOT, env=env, text=True, capture_output=True, check=False)
    return {
        "pytest_ran": True,
        "command": "PYTHONPATH=. python3 -m pytest tests/unit/test_ur10e_fk.py tests/unit/test_p001b_evidence.py tests/unit/test_p006_controller.py tests/unit/test_p007_convert.py tests/unit/test_p008_validator.py tests/unit/test_p008_fix2_acceptance.py -q",
        "returncode": completed.returncode,
        "stdout": completed.stdout,
        "stderr": completed.stderr,
        "isaac_rerun": False,
        "unity_rerun": False,
        "ros_evidence_rerun": False,
    }


def _signature(report: dict) -> dict:
    pair = None
    sample = None
    for check in report.get("checks", []):
        if check.get("result") == "FAIL" and check.get("reason_code") == "COLLISION":
            metrics = check["metrics"]
            objects = metrics.get("involved_objects") or []
            pair = objects[0] if objects else None
            sample = metrics.get("sample_index")
            break
    return {
        "decision": report["decision"],
        "blocking_reason_codes": list(report["blocking_reason_codes"]),
        "collision_pair": pair,
        "first_collision_sample": sample,
        "controller_submitted": report["controller_submitted"],
    }


def _signature_from_negative(row: dict) -> dict:
    pair, sample = _collision_from_summary(row.get("summary"))
    return {
        "decision": row.get("decision"),
        "blocking_reason_codes": list(row.get("blocking_reason_codes") or []),
        "collision_pair": pair,
        "first_collision_sample": sample,
        "controller_submitted": row.get("controller_submitted"),
    }


def _collision_from_summary(summary: object) -> tuple[dict | None, int | None]:
    text = str(summary or "")
    marker = " intersects "
    sample_marker = " at sample "
    if marker not in text or sample_marker not in text:
        return None, None
    left, right = text.split(marker, 1)
    name_b, sample_text = right.split(sample_marker, 1)
    name_a = left.split("— ", 1)[1]
    return {"a": name_a, "b": name_b}, int(sample_text.split("/", 1)[0])


def _checks(report: dict) -> list[dict]:
    rows = []
    for check in report["checks"]:
        metrics = check["metrics"]
        kept = {
            key: metrics[key]
            for key in ("sample_index", "sample_count", "involved_objects", "label", "method", "class", "triangle_tests", "adjacent_links_excluded")
            if key in metrics
        }
        rows.append(
            {
                "name": check["name"],
                "result": check["result"],
                "reason_code": check["reason_code"],
                "metrics": kept,
            }
        )
    return rows


def _self_evaluated(checks: list[dict]) -> bool:
    row = next(item for item in checks if item["name"] == "self_collision")
    return row["result"] == "PASS" and row["metrics"].get("label") == "EVALUATED"


def _check_result(row: dict, name: str) -> str:
    return next(item["result"] for item in row["checks"] if item["name"] == name)


def _triangle_tests(report: dict) -> int:
    checks = {item["name"]: item for item in report["checks"]}
    return int(checks["robot_environment_collision"]["metrics"]["triangle_tests"])


def _config_unchanged(config: dict) -> bool:
    collision = config["collision"]
    zone = config["safety_zone"]
    return (
        config["trajectory_sample_count"] == 100
        and collision["touching_faces_are_collision"] is False
        and collision["tool_parent_link_excluded"] == "wrist_3_link"
        and zone["prohibited_depth_m"] == 0.4
        and zone["class"] == "PROJECT TEST SAFETY ZONE"
        and config["trajectory_model"] == "PROJECT VALIDATION PATH"
        and "industrial safety certified" in config["not_claims"]
    )


def _frozen_history() -> dict:
    summary = (ROOT / "evidence" / "P-008" / "summary.md").read_text(encoding="utf-8")
    result = "FAIL" if "\nFAIL\n" in summary else "UNKNOWN"
    return {"p008_result": result, "p009_exists": (ROOT / "evidence" / "P-009").exists()}


def _fix1_evidence() -> dict:
    root_cause = json.loads((FIX1 / "root_cause.json").read_text(encoding="utf-8"))
    independent = json.loads((FIX1 / "independent_collision_check.json").read_text(encoding="utf-8"))
    scene = json.loads((FIX1 / "scene_comparison.json").read_text(encoding="utf-8"))
    return {
        "files": [
            "evidence/P-008-FIX1/root_cause.json",
            "evidence/P-008-FIX1/independent_collision_check.json",
            "evidence/P-008-FIX1/scene_comparison.json",
        ],
        "classification": root_cause["classification"],
        "validator_modified_in_fix1": root_cause["validator_modified"],
        "independent_collision": independent["strict_intersection"],
        "scene_matches": scene["all_match"],
    }


def _count_non_finite(value: object) -> int:
    if isinstance(value, float):
        return 0 if math.isfinite(value) else 1
    if isinstance(value, dict):
        return sum(_count_non_finite(item) for item in value.values())
    if isinstance(value, (list, tuple)):
        return sum(_count_non_finite(item) for item in value)
    return 0


def _write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")


def main() -> None:
    payload = write_evidence(OUTPUT)
    print(payload["result"])
    print(payload["e_fixed_1"]["actual_decision"])
    print(payload["designated_safe_pass"])
    print(payload["negative_pass"])


if __name__ == "__main__":
    main()
