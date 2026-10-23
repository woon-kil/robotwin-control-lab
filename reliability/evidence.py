"""P-011 evidence. 다른 Gate 디렉터리는 쓰지 않는다."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

from reliability.metrics import case_summary, drift_matches, drift_results, summarize, unbounded_rss_growth, validation_matrix
from reliability.model import CLAIM, OUTPUT, load_reliability_config
from reliability.runner import ReliabilitySession

ROOT = Path(__file__).resolve().parents[1]


def assert_output_directory(directory: Path) -> Path:
    if directory.resolve() != OUTPUT.resolve():
        raise RuntimeError("P-011 evidence can only be written to evidence/P-011")
    return directory.resolve()


def main() -> None:
    directory = assert_output_directory(OUTPUT)
    directory.mkdir(parents=True, exist_ok=True)
    session = ReliabilitySession()
    session.run_primary(directory / "cycle_results.jsonl")
    session.run_recovery()
    code, text = _regression()
    result = write_evidence(directory, session, code, text)
    print(result["result"])
    print(f"cycles {len(session.rows)}/500")
    if result["result"] != "PASS":
        raise SystemExit(1)


def write_evidence(directory: Path, session: ReliabilitySession, regression_code: int, regression_text: str) -> dict:
    directory = assert_output_directory(directory)
    rows = session.rows
    safe_rows = [row for row in rows if row["expected_decision"] == "SAFE_TO_ATTEMPT"]
    reject_rows = [row for row in rows if row["expected_decision"] == "REJECT"]
    durations = [float(row["duration_sec"]) for row in safe_rows if isinstance(row["duration_sec"], (int, float))]
    settling = [float(row["settling_time_sec"]) for row in safe_rows if isinstance(row["settling_time_sec"], (int, float))]
    joints = [float(row["final_joint_error_rad"]) for row in safe_rows if isinstance(row["final_joint_error_rad"], (int, float))]
    position = [float(row["final_tcp_position_error_m"]) for row in safe_rows if isinstance(row["final_tcp_position_error_m"], (int, float))]
    orientation = [
        float(row["final_tcp_orientation_error_rad"])
        for row in safe_rows
        if isinstance(row["final_tcp_orientation_error_rad"], (int, float))
    ]
    samples = [float(row["sample_count"]) for row in safe_rows]
    timing = {"execution_duration": summarize(durations), "settling_time": summarize(settling)}
    tracking = {
        "final_joint_error": summarize(joints),
        "final_tcp_position_error": summarize(position),
        "final_tcp_orientation_error": summarize(orientation),
        "sample_count": summarize(samples),
    }
    drift = drift_results(rows)
    matrix = validation_matrix(rows)
    cases = case_summary(rows)
    rss = [point["rss_bytes"] for point in session.checkpoints if isinstance(point.get("rss_bytes"), int)]
    resources = {
        "samples": session.checkpoints,
        "rss_start_bytes": rss[0] if rss else None,
        "rss_end_bytes": rss[-1] if rss else None,
        "rss_delta_bytes": None if len(rss) < 2 else rss[-1] - rss[0],
        "unbounded_monotonic_growth": unbounded_rss_growth(rss),
        "process_failed": False,
        "oom": False,
        "note": "observed process RSS. Growth is not by itself a memory leak.",
    }
    recovery = {
        "attempted": len(session.recovery),
        "passed": sum(1 for row in session.recovery if row.get("result") == "PASS"),
        "failed": sum(1 for row in session.recovery if row.get("result") != "PASS"),
        "per_run": session.recovery,
    }
    counts = _counts(rows, session)
    checklist = _checklist(counts, drift, resources, recovery, regression_code, rows)
    result = "PASS" if all(checklist.values()) else "FAIL"
    _write_json(directory / "reliability_contract.json", _contract())
    _write_json(directory / "run_config.json", load_reliability_config())
    _write_json(directory / "case_summary.json", cases)
    _write_json(directory / "validation_matrix.json", matrix)
    _write_json(directory / "timing_metrics.json", timing)
    _write_json(directory / "tracking_metrics.json", tracking)
    _write_json(directory / "drift_results.json", drift)
    _write_json(directory / "checkpoint_results.json", {"checkpoints": session.checkpoints})
    _write_json(directory / "resource_results.json", resources)
    _write_json(directory / "recovery_results.json", recovery)
    _write_json(
        directory / "regression_results.json",
        {
            "command": _regression_command(),
            "returncode": regression_code,
            "result": "PASS" if regression_code == 0 else "FAIL",
            "historical_core_modified": _core_dirty(),
            "p012_started": (ROOT / "evidence" / "P-012").exists(),
        },
    )
    (directory / "test_results.txt").write_text(regression_text, encoding="utf-8")
    (directory / "summary.md").write_text(
        _summary(result, counts, timing, tracking, resources, recovery, session, drift),
        encoding="utf-8",
    )
    return {"result": result, "checklist": checklist, "counts": counts}


def _counts(rows: list[dict], session: ReliabilitySession) -> dict:
    safe = [row for row in rows if row["expected_decision"] == "SAFE_TO_ATTEMPT"]
    reject = [row for row in rows if row["expected_decision"] == "REJECT"]
    return {
        "cycles": len(rows),
        "correct": sum(1 for row in rows if row["result"] == "PASS"),
        "safe_correct": sum(1 for row in safe if row["actual_decision"] == "SAFE_TO_ATTEMPT" and row["result"] == "PASS"),
        "reject_correct": sum(1 for row in reject if row["actual_decision"] == "REJECT" and not row["controller_submitted"]),
        "unsafe_submissions": sum(1 for row in rows if row.get("failure_category") == "UNSAFE_SUBMISSION" or (row["expected_decision"] == "REJECT" and row["controller_submitted"])),
        "false_rejects": sum(1 for row in safe if row["actual_decision"] == "REJECT"),
        "completed": sum(1 for row in safe if row["completed"]),
        "settled": sum(1 for row in safe if row["settled"]),
        "rejected_tracking_started": sum(1 for row in reject if row["tracking_started"]),
        "timeouts": sum(1 for row in rows if row["timeout"]),
        "non_finite": sum(1 for row in rows if row["non_finite"]),
        "state_leakage": sum(1 for row in rows if row.get("failure_category") == "STATE_LEAK" or row.get("state_leak")),
        "unexpected_faults": sum(1 for row in rows if row["fault"]),
        "stopped": session.stop_reason,
        "first_failure_cycle": session.first_failure_cycle,
        "first_failure_category": session.first_failure_category,
        "first_failure_details": session.first_failure_details,
        "wall_runtime_sec": getattr(session, "wall_runtime_sec", None),
    }


def _checklist(counts: dict, drift: dict, resources: dict, recovery: dict, regression_code: int, rows: list[dict]) -> dict:
    drift_ok = bool(drift) and all(drift_matches(row) for row in drift.values())
    recovery_ok = recovery["attempted"] >= 10 and recovery["passed"] == recovery["attempted"] and recovery["failed"] == 0
    return {
        "cycles_500": counts["cycles"] == 500,
        "correct_500": counts["correct"] == 500,
        "safe_400": counts["safe_correct"] == 400,
        "reject_100": counts["reject_correct"] == 100,
        "unsafe_0": counts["unsafe_submissions"] == 0,
        "false_reject_0": counts["false_rejects"] == 0,
        "completed_400": counts["completed"] == 400,
        "settled_400": counts["settled"] == 400,
        "rejected_tracking_0": counts["rejected_tracking_started"] == 0,
        "timeout_0": counts["timeouts"] == 0,
        "non_finite_0": counts["non_finite"] == 0,
        "state_leak_0": counts["state_leakage"] == 0,
        "unexpected_fault_0": counts["unexpected_faults"] == 0,
        "metrics_present": _metrics_present(rows),
        "drift": drift_ok,
        "resource": not resources["unbounded_monotonic_growth"] and not resources["process_failed"] and not resources["oom"],
        "recovery": recovery_ok,
        "regression": regression_code == 0,
        "core_untouched": not _core_dirty(),
        "p012_not_started": not (ROOT / "evidence" / "P-012").exists(),
    }


def _metrics_present(rows: list[dict]) -> bool:
    safe = [row for row in rows if row["expected_decision"] == "SAFE_TO_ATTEMPT" and row["settled"]]
    if len(safe) != 400:
        return False
    keys = ("duration_sec", "settling_time_sec", "final_joint_error_rad", "final_tcp_position_error_m", "final_tcp_orientation_error_rad", "sample_count")
    return all(isinstance(row[key], (int, float)) and not isinstance(row[key], bool) for row in safe for key in keys)


def _delta_clear(row: dict) -> bool:
    return drift_matches(row)


def _summary(result: str, counts: dict, timing: dict, tracking: dict, resources: dict, recovery: dict, session: ReliabilitySession, drift: dict) -> str:
    duration = timing["execution_duration"]
    settling = timing["settling_time"]
    joint = tracking["final_joint_error"]
    position = tracking["final_tcp_position_error"]
    orientation = tracking["final_tcp_orientation_error"]
    lines = [
        "P-011",
        "",
        "500-CYCLE DETERMINISTIC PROJECT WORKLOAD",
        "",
        "Result:",
        result,
        "",
        "Cycles:",
        f"{counts['cycles']}/500",
        "",
        "Correct:",
        f"{counts['correct']}/500",
        "",
        "Safe:",
        f"{counts['safe_correct']}/400",
        "",
        "Reject:",
        f"{counts['reject_correct']}/100",
        "",
        "Unsafe submissions:",
        str(counts["unsafe_submissions"]),
        "",
        "False rejects:",
        str(counts["false_rejects"]),
        "",
        "Completed:",
        f"{counts['completed']}/400",
        "",
        "Settled:",
        f"{counts['settled']}/400",
        "",
        "Timeouts:",
        str(counts["timeouts"]),
        "",
        "Non-finite:",
        str(counts["non_finite"]),
        "",
        "State leakage:",
        str(counts["state_leakage"]),
        "",
        "Unexpected faults:",
        str(counts["unexpected_faults"]),
        "",
        "Recovery:",
        f"{recovery['passed']}/{recovery['attempted']}",
        "",
        "Execution duration:",
        _spread(duration),
        "",
        "Settling:",
        _spread(settling),
        "",
        "Final joint error:",
        _spread(joint),
        "",
        "Final TCP position error:",
        _spread(position),
        "",
        "Final TCP orientation error:",
        _spread(orientation),
        "",
        "Observed RSS:",
        f"{resources['rss_start_bytes']} / {resources['rss_end_bytes']} / {resources['rss_delta_bytes']}",
        "",
        "Wall runtime:",
        str(counts["wall_runtime_sec"]),
        "",
        "Settling drift max sec:",
        str(_max_settling_drift(drift)),
        "",
        "Drift comparison:",
        "sample count and final errors exact. settling subtraction bound 1e-9 s.",
        "",
        "Historical core modified:",
        "YES" if _core_dirty() else "NO",
        "",
        "P-012 started:",
        "NO",
        "",
        "Claim:",
        CLAIM,
        "",
        "First failure:",
        str(counts["first_failure_cycle"]),
        str(counts["first_failure_category"]),
        "",
        "Employment-ready MVP:",
        "YES" if result == "PASS" else "NO",
    ]
    if session.stop_reason:
        lines.extend(["", "Stopped:", session.stop_reason])
    return "\n".join(lines) + "\n"


def _max_settling_drift(drift: dict) -> float | None:
    values = [row.get("settling_time_delta") for row in drift.values() if isinstance(row.get("settling_time_delta"), (int, float))]
    if not values:
        return None
    return max(abs(float(value)) for value in values)


def _spread(summary: dict) -> str:
    return f"mean {summary['mean']} / P50 {summary['p50']} / P95 {summary['p95']} / P99 {summary['p99']} / max {summary['max']}"


def _contract() -> dict:
    config = load_reliability_config()
    return {
        "purpose": "Repeat the existing validate, execute, reject, and track path on one controller session.",
        "cycle_definition": {
            "safe": "known zero start, command, SAFE_TO_ATTEMPT, controller submission, tracking, completion, settling, record",
            "reject": "command, REJECT, controller not called, tracking not started, record",
        },
        "workload": config["sequence"],
        "iterations": config["iterations"],
        "expected_outcomes": {"SAFE_TO_ATTEMPT": 400, "REJECT": 100},
        "pass_rule": "500/500 correct cycles on this deterministic project workload, with 400 settled executions and 100 collision rejects.",
        "failure_categories": [
            "VALIDATION_MISMATCH",
            "UNSAFE_SUBMISSION",
            "SAFE_NOT_SUBMITTED",
            "EXECUTION_FAILURE",
            "NOT_SETTLED",
            "TIMEOUT",
            "NON_FINITE",
            "STATE_LEAK",
            "RESET_FAILURE",
            "UNEXPECTED_FAULT",
            "METRIC_FAILURE",
            "INTERNAL_ERROR",
        ],
        "critical_fail_rule": "UNSAFE_SUBMISSION stops the run. A REJECT that is submitted or tracked is UNSAFE_SUBMISSION.",
        "reset_semantics": config["reset"],
        "known_start": config["known_start"],
        "timing_semantics": config["timing_semantics"],
        "resource_semantics": config["resource_semantics"],
        "claim_boundary": CLAIM,
        "not_claims": [
            "industrial reliability certified",
            "production proven",
            "24/7 validated",
            "MTBF proven",
            "field reliability proven",
            "real robot reliability",
        ],
        "percentile_method": "linear interpolation between the nearest sorted ranks",
        "drift_comparison": "Final joint error, TCP error, and sample count deltas are exact. Settling time is a project-clock subtraction and may differ by about 1e-13 s; the comparison bound is 1e-9 s, below the 0.01 s sample period.",
        "screenshots": "P-010 visual evidence is not repeated. This gate records quantitative results.",
    }


def _regression() -> tuple[int, str]:
    env = os.environ.copy()
    env["PYTHONPATH"] = str(ROOT)
    completed = subprocess.run(
        [sys.executable, "-m", "pytest", *_regression_targets(), "-q"],
        cwd=ROOT,
        env=env,
        text=True,
        capture_output=True,
        check=False,
    )
    return completed.returncode, completed.stdout + "\n" + completed.stderr


def _regression_command() -> str:
    return "PYTHONPATH=. python3 -m pytest " + " ".join(_regression_targets()) + " -q"


def _regression_targets() -> list[str]:
    return [
        "tests/unit/test_p006_controller.py",
        "tests/unit/test_p008_validator.py",
        "tests/unit/test_p009_monitor.py",
        "tests/unit/test_p010_hmi.py",
        "tests/unit/test_p011_reliability.py",
    ]


def _core_dirty() -> bool:
    completed = subprocess.run(
        ["git", "status", "--short", "--", "controller", "validator", "digital_twin", "hmi", "evidence/P-010"],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    return bool(completed.stdout.strip())


def _write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
