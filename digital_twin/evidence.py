"""P-009 evidence. 기존 evidence 디렉터리는 쓰지 않는다."""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
from pathlib import Path

from core.model.ur10e import load_ur10e_model
from digital_twin.monitor import load_monitor_config
from digital_twin.session import DESIGNATED_SAFE, KNOWN_COLLISION, run_case
from validator.validate import load_validator_config
from workcell.layout import resolve_workcell

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "evidence" / "P-009"


def assert_output_directory(directory: Path) -> Path:
    if directory.resolve() != OUTPUT.resolve():
        raise RuntimeError("P-009 evidence can only be written to evidence/P-009")
    return directory.resolve()


def build_payload() -> dict:
    model = load_ur10e_model()
    resolved = resolve_workcell()
    validator_config = load_validator_config()
    monitor_config = load_monitor_config()
    first = [run_case(name, model, resolved, validator_config, monitor_config) for name in DESIGNATED_SAFE]
    second = [run_case(name, model, resolved, validator_config, monitor_config) for name in DESIGNATED_SAFE]
    rejection = run_case(KNOWN_COLLISION, model, resolved, validator_config, monitor_config)
    fault = run_case("B-positive-1", model, resolved, validator_config, monitor_config, fault_after_steps=1)
    determinism = _determinism(first, second)
    payload = {
        "tracking": first,
        "rejection": rejection,
        "fault": fault,
        "determinism_results": determinism,
        "monitor_config": monitor_config,
        "timeout_count": sum(1 for row in first if row["timed_out"]),
        "p010_started": (ROOT / "evidence" / "P-010").exists(),
        "historical_p008_fail_preserved": "\nFAIL\n" in (ROOT / "evidence" / "P-008" / "summary.md").read_text(encoding="utf-8"),
    }
    payload["non_finite_count"] = _count_non_finite(payload)
    payload["checklist"] = _checklist(payload)
    payload["case_result"] = "PASS" if all(payload["checklist"].values()) else "FAIL"
    return payload


def write_evidence(directory: Path | None = None) -> dict:
    target = assert_output_directory(OUTPUT if directory is None else directory)
    payload = build_payload()
    regression = _pytest()
    payload["regression_results"] = regression
    payload["result"] = "PASS" if payload["case_result"] == "PASS" and regression["returncode"] == 0 else "FAIL"
    target.mkdir(parents=True, exist_ok=True)
    traces = target / "traces"
    traces.mkdir(parents=True, exist_ok=True)
    _write_json(target / "monitor_contract.json", _contract(payload))
    _write_json(target / "monitor_config.json", payload["monitor_config"])
    _write_json(target / "tracking_results.json", {"cases": [_public(row) for row in payload["tracking"]]})
    _write_json(target / "tracking_metrics.json", {"cases": [_metrics_row(row) for row in payload["tracking"]]})
    _write_json(target / "rejection_results.json", _rejection(payload["rejection"]))
    _write_json(target / "fault_results.json", _fault(payload["fault"]))
    _write_json(target / "determinism_results.json", payload["determinism_results"])
    _write_json(target / "regression_results.json", regression)
    for row in payload["tracking"]:
        _write_jsonl(traces / (row["case"] + ".jsonl"), row["samples"])
    (target / "summary.md").write_text(_summary(payload), encoding="utf-8")
    (target / "test_results.txt").write_text(_test_text(payload), encoding="utf-8")
    return payload


def _checklist(payload: dict) -> dict[str, bool]:
    rows = payload["tracking"]
    by_name = {row["case"]: row for row in rows}
    rejection = payload["rejection"]
    fault = payload["fault"]
    config = payload["monitor_config"]
    separated = [
        name
        for name in ("B-positive-1", "C-mixed-1", "E-fixed-3")
        if any(sample["target_q"] != sample["actual_q"] for sample in by_name[name]["samples"])
    ]
    joint_ok = all(
        sample["joint_error_rad"][index] == sample["target_q"][index] - sample["actual_q"][index]
        for row in rows
        for sample in row["samples"]
        for index in range(6)
    )
    tcp_consistent = all(
        math.dist(sample["target_tcp"]["position_m"], sample["actual_tcp"]["position_m"]) == sample["tcp_position_error_m"]
        for row in rows
        for sample in row["samples"]
    )
    opened = by_name["B-positive-1"]["samples"][0]
    fk_reused = opened["target_tcp"]["position_m"] != opened["actual_tcp"]["position_m"]
    final_ok = all(row["metrics"]["final_max_joint_error_rad"] <= config["joint_settling_tolerance_rad"] for row in rows)
    settling_distinct = all(
        row["metrics"]["completion_time_s"] != row["metrics"]["settled_time_s"] and row["completed"] and row["settled"]
        for row in rows
    )
    return {
        "monitor_interface": all(row["status"] == "SETTLED" for row in rows),
        "target_actual_independent": len(separated) == 3 and all(sample["target_q"] == sample["actual_q"] for sample in by_name["A-zero"]["samples"]),
        "joint_error": joint_ok,
        "tcp_position_error": tcp_consistent and all(row["metrics"]["final_tcp_position_error_m"] == 0.0 for row in rows),
        "tcp_orientation_error": all(
            row["samples"][-1]["target_tcp"]["quaternion_xyzw"] == row["samples"][-1]["actual_tcp"]["quaternion_xyzw"]
            for row in rows
        ),
        "p001b_fk_reused": fk_reused,
        "designated_safe_cases": [row["case"] for row in rows] == list(DESIGNATED_SAFE) and all(row["validation_decision"] == "SAFE_TO_ATTEMPT" for row in rows),
        "safe_commands_tracked": all(row["tracking_started"] for row in rows),
        "safe_executions_completed": all(row["completed"] for row in rows),
        "nonzero_target_actual_separation": len(separated) == 3,
        "final_convergence": final_ok,
        "settling_measured": all(row["settling_time_s"] is not None for row in rows),
        "completion_settling_separate": settling_distinct,
        "e_fixed_1_blocked": rejection["validation_decision"] == "REJECT" and rejection["controller_submitted"] is False,
        "e_fixed_1_tracking_not_started": rejection["tracking_started"] is False and rejection["execution_trace_created"] is False,
        "fault_not_completed": fault["fault_occurred"] is True and fault["completed"] is False,
        "fault_not_settled": fault["settled"] is False and fault["tracking_started"] is True,
        "non_finite_output": payload["non_finite_count"] == 0,
        "deterministic_replay": payload["determinism_results"]["equal"] is True,
        "timeout": payload["timeout_count"] == 0,
        "tolerance_not_retuned": config["joint_settling_tolerance_rad"] == 1e-9 and config["settling_consecutive_samples"] == 3 and config["sample_period_sec"] == 0.01,
        "real_robot_accuracy_not_claimed": "actual UR10e positioning accuracy" in config["not_claims"],
        "hard_realtime_not_claimed": "hard real-time certification" in config["not_claims"],
        "p010_not_started": payload["p010_started"] is False,
        "historical_evidence_preserved": payload["historical_p008_fail_preserved"] is True,
    }


def _contract(payload: dict) -> dict:
    return {
        "target_definition": "target_q is the commanded joint position accepted by VirtualPmacController. The monitor does not copy it from actual_q.",
        "actual_definition": "actual_q is the controller plant joint position at the sample. The monitor does not copy it from target_q.",
        "sample_schema": [
            "sequence",
            "timestamp_s",
            "dt_s",
            "controller_state",
            "execution_state",
            "target_q",
            "actual_q",
            "joint_error_rad",
            "abs_joint_error_rad",
            "target_tcp",
            "actual_tcp",
            "tcp_position_error_m",
            "tcp_orientation_error_rad",
        ],
        "joint_error_formula": "error_i = target_i - actual_i, radian",
        "tcp_position_formula": "||p_target - p_actual||, meter, from P-001B forward_kinematics in the robot base frame",
        "tcp_orientation_formula": "shortest angle of R_target^-1 * R_actual via core.validation.tcp_compare.orientation_error_rad. trace(R) = trace(R^-1), so this is the same angle as R_target * R_actual^-1. Euler component subtraction is not used. Identical quaternions can still produce a sub-1e-7 rad acos result; the stored quaternions are the pose comparison.",
        "fk_source": "core.kinematics.fk.forward_kinematics",
        "settling_rule": "PROJECT TEST PARAMETER. max abs joint error <= joint_settling_tolerance_rad for settling_consecutive_samples consecutive samples after execution start. N is 3. A single COMPLETED sample is not SETTLED.",
        "completion_vs_settling": "COMPLETED is the controller trace. SETTLED is the monitor rule. They are recorded as separate times and are not assigned from each other.",
        "time_source": payload["monitor_config"]["time_source"],
        "sample_period_sec": payload["monitor_config"]["sample_period_sec"],
        "start_state": payload["monitor_config"]["start_state"],
        "claim_boundary": payload["monitor_config"]["claim"],
        "not_claims": payload["monitor_config"]["not_claims"],
        "tracking_error_note": "global max joint and TCP errors include the initial target-actual difference. They are not controller accuracy and not UR10e accuracy.",
        "p007_relation": "P-007 latency is transport time. This gate measures motion-state tracking and does not rerun ROS.",
    }


def _public(row: dict) -> dict:
    return {
        "case": row["case"],
        "validation_decision": row["validation_decision"],
        "controller_submitted": row["controller_submitted"],
        "tracking_started": row["tracking_started"],
        "completed": row["completed"],
        "settled": row["settled"],
        "sample_count": row["sample_count"],
        "metrics": row["metrics"],
    }


def _metrics_row(row: dict) -> dict:
    return {"case": row["case"], "metrics": row["metrics"]}


def _rejection(row: dict) -> dict:
    return {
        "case": row["case"],
        "validation_decision": row["validation_decision"],
        "controller_submitted": row["controller_submitted"],
        "tracking_started": row["tracking_started"],
        "execution_trace_created": row["execution_trace_created"],
        "submit_count_before": row["submit_count_before"],
        "submit_count_after": row["submit_count_after"],
        "blocking_reason_codes": row["blocking_reason_codes"],
    }


def _fault(row: dict) -> dict:
    observation = row["fault_observation"] or {}
    return {
        "fault_case": row["case"],
        "tracking_started": row["tracking_started"],
        "fault_step": row["fault_step"],
        "fault_occurred": row["fault_occurred"],
        "execution_completed": row["completed"],
        "settled": row["settled"],
        "status": row["status"],
        "actual_q_at_fault": observation.get("actual_q"),
        "target_q_at_fault": observation.get("target_q"),
        "controller_trace": row["controller_trace"],
    }


def _determinism(first: list[dict], second: list[dict]) -> dict:
    mismatches = []
    for left, right in zip(first, second, strict=True):
        if _signature(left) != _signature(right):
            mismatches.append(left["case"])
    return {
        "runs": 2,
        "equal": not mismatches,
        "numeric_tolerance": 0,
        "comparison": "exact JSON equality of samples, metrics, traces, settling time, and completion",
        "compared": [
            "sample_count",
            "state sequence",
            "target trace",
            "actual trace",
            "joint errors",
            "TCP errors",
            "settling time",
            "final metrics",
        ],
        "mismatches": mismatches,
    }


def _signature(row: dict) -> str:
    payload = {
        "samples": row["samples"],
        "metrics": row["metrics"],
        "settled": row["settled"],
        "settling_time_s": row["settling_time_s"],
        "completed": row["completed"],
        "controller_trace": row["controller_trace"],
    }
    return json.dumps(payload, sort_keys=True, allow_nan=False)


def _summary(payload: dict) -> str:
    rows = payload["tracking"]
    peak_joint = max(row["metrics"]["global_max_joint_error_rad"] for row in rows)
    peak_position_mm = max(row["metrics"]["max_tcp_position_error_mm"] for row in rows)
    peak_orientation_deg = max(row["metrics"]["max_tcp_orientation_error_deg"] for row in rows)
    final_joint = max(row["metrics"]["final_max_joint_error_rad"] for row in rows)
    completed = sum(1 for row in rows if row["completed"])
    regression = payload["regression_results"]
    lines = [
        "P-009",
        "",
        "PROJECT DIGITAL-TWIN STATE TRACKING",
        "This is not UR10e positioning accuracy, industrial trajectory tracking, or hard real-time.",
        "",
        "Result",
        payload["result"],
        "",
        "Designated safe tracked",
        str(sum(1 for row in rows if row["tracking_started"])) + "/4",
        "",
        "Executions completed",
        str(completed) + "/4",
        "",
        "Global max joint tracking error rad",
        repr(peak_joint),
        "",
        "Max TCP position tracking error mm",
        repr(peak_position_mm),
        "",
        "Max TCP orientation tracking error deg",
        repr(peak_orientation_deg),
        "",
        "Final max joint error rad",
        repr(final_joint),
        "",
        "Settling rule",
        "max abs joint error <= 1e-9 rad for 3 consecutive samples after execution start",
        "",
        "E-fixed-1 tracking started",
        "YES" if payload["rejection"]["tracking_started"] else "NO",
        "",
        "Fault settled",
        "YES" if payload["fault"]["settled"] else "NO",
        "",
        "Determinism",
        "PASS" if payload["determinism_results"]["equal"] else "FAIL",
        "",
        "Timeout",
        str(payload["timeout_count"]),
        "",
        "Non-finite",
        str(payload["non_finite_count"]),
        "",
        "Regression",
        "PASS" if regression["returncode"] == 0 else "FAIL",
        "",
        "P-010 started",
        "NO",
        "",
    ]
    return "\n".join(lines)


def _test_text(payload: dict) -> str:
    regression = payload["regression_results"]
    return "\n".join(
        [
            "RESULT " + payload["result"],
            "timeouts " + str(payload["timeout_count"]),
            "non_finite " + str(payload["non_finite_count"]),
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
        "tests/unit/test_p009_monitor.py",
        "-q",
    ]
    completed = subprocess.run(command, cwd=ROOT, env=env, text=True, capture_output=True, check=False)
    return {
        "pytest_ran": True,
        "command": "PYTHONPATH=. python3 -m pytest tests/unit/test_ur10e_fk.py tests/unit/test_p001b_evidence.py tests/unit/test_p006_controller.py tests/unit/test_p007_convert.py tests/unit/test_p008_validator.py tests/unit/test_p008_fix2_acceptance.py tests/unit/test_p009_monitor.py -q",
        "returncode": completed.returncode,
        "stdout": completed.stdout,
        "stderr": completed.stderr,
        "ros_evidence_rerun": False,
        "isaac_rerun": False,
        "unity_rerun": False,
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


def _write_jsonl(path: Path, samples: list[dict]) -> None:
    lines = [json.dumps(sample, sort_keys=True, allow_nan=False) for sample in samples]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    payload = write_evidence(OUTPUT)
    peak_joint = max(row["metrics"]["global_max_joint_error_rad"] for row in payload["tracking"])
    peak_mm = max(row["metrics"]["max_tcp_position_error_mm"] for row in payload["tracking"])
    peak_deg = max(row["metrics"]["max_tcp_orientation_error_deg"] for row in payload["tracking"])
    final_joint = max(row["metrics"]["final_max_joint_error_rad"] for row in payload["tracking"])
    print(payload["result"])
    print(peak_joint)
    print(peak_mm)
    print(peak_deg)
    print(final_joint)
    print(payload["timeout_count"])
    print(payload["non_finite_count"])


if __name__ == "__main__":
    main()
