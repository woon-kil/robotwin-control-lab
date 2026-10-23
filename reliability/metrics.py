"""500-cycle 집계. 새 tracking 식은 만들지 않는다."""

from __future__ import annotations

import math

from reliability.model import DRIFT_SETTLING_ABS_SEC, EXPECTED, FAILURE_CATEGORIES, NONZERO_SAFE, SEQUENCE


def percentile(values: list[float], percent: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * (percent / 100.0)
    low = math.floor(rank)
    high = math.ceil(rank)
    if low == high:
        return ordered[low]
    weight = rank - low
    return ordered[low] * (1.0 - weight) + ordered[high] * weight


def summarize(values: list[float]) -> dict:
    if not values:
        return {"count": 0, "min": None, "mean": None, "p50": None, "p95": None, "p99": None, "max": None}
    return {
        "count": len(values),
        "min": min(values),
        "mean": sum(values) / len(values),
        "p50": percentile(values, 50),
        "p95": percentile(values, 95),
        "p99": percentile(values, 99),
        "max": max(values),
    }


def classify_cycle(row: dict) -> str | None:
    expected = row["expected_decision"]
    actual = row["actual_decision"]
    submitted = bool(row["controller_submitted"])
    tracked = bool(row["tracking_started"])
    if expected == "REJECT" and (submitted or tracked):
        return "UNSAFE_SUBMISSION"
    if row.get("internal_error"):
        return "INTERNAL_ERROR"
    if row.get("state_leak"):
        return "STATE_LEAK"
    if row.get("reset_result") == "FAILED":
        return "RESET_FAILURE"
    if row.get("timeout"):
        return "TIMEOUT"
    if row.get("non_finite"):
        return "NON_FINITE"
    if row.get("fault"):
        return "UNEXPECTED_FAULT"
    if actual != expected:
        return "VALIDATION_MISMATCH"
    if expected == "SAFE_TO_ATTEMPT" and not submitted:
        return "SAFE_NOT_SUBMITTED"
    if expected == "SAFE_TO_ATTEMPT" and not row.get("completed"):
        return "EXECUTION_FAILURE"
    if expected == "SAFE_TO_ATTEMPT" and not row.get("settled"):
        return "NOT_SETTLED"
    if expected == "SAFE_TO_ATTEMPT" and row["case_name"] in NONZERO_SAFE and not row.get("separation_observed"):
        return "METRIC_FAILURE"
    if expected == "REJECT" and "COLLISION" not in (row.get("validation_reason") or []):
        return "VALIDATION_MISMATCH"
    if row.get("category_hint") in FAILURE_CATEGORIES:
        return row["category_hint"]
    return None


def validation_matrix(rows: list[dict]) -> dict:
    matrix = {
        "expected_safe_actual_safe": 0,
        "expected_safe_actual_reject": 0,
        "expected_reject_actual_safe": 0,
        "expected_reject_actual_reject": 0,
    }
    for row in rows:
        expected_safe = row["expected_decision"] == "SAFE_TO_ATTEMPT"
        actual_safe = row["actual_decision"] == "SAFE_TO_ATTEMPT"
        if expected_safe and actual_safe:
            matrix["expected_safe_actual_safe"] += 1
        elif expected_safe and not actual_safe:
            matrix["expected_safe_actual_reject"] += 1
        elif not expected_safe and actual_safe:
            matrix["expected_reject_actual_safe"] += 1
        else:
            matrix["expected_reject_actual_reject"] += 1
    return matrix


def case_summary(rows: list[dict]) -> dict:
    summary = {}
    for name in SEQUENCE:
        selected = [row for row in rows if row["case_name"] == name]
        summary[name] = {
            "expected_count": 100,
            "actual_count": len(selected),
            "correct_count": sum(1 for row in selected if row["result"] == "PASS"),
            "safe_count": sum(1 for row in selected if row["actual_decision"] == "SAFE_TO_ATTEMPT"),
            "reject_count": sum(1 for row in selected if row["actual_decision"] == "REJECT"),
            "submitted_count": sum(1 for row in selected if row["controller_submitted"]),
            "completed_count": sum(1 for row in selected if row["completed"]),
            "settled_count": sum(1 for row in selected if row["settled"]),
            "timeout_count": sum(1 for row in selected if row["timeout"]),
            "non_finite_count": sum(1 for row in selected if row["non_finite"]),
            "failure_count": sum(1 for row in selected if row["result"] != "PASS"),
        }
    return summary


def drift_results(rows: list[dict]) -> dict:
    payload = {}
    for name in SEQUENCE:
        if EXPECTED[name] != "SAFE_TO_ATTEMPT":
            continue
        selected = [row for row in rows if row["case_name"] == name and row["result"] == "PASS"]
        first = next((row for row in selected if row["iteration"] == 1), None)
        last = next((row for row in selected if row["iteration"] == 100), None)
        payload[name] = _drift_pair(first, last)
    return payload


def drift_matches(row: dict) -> bool:
    exact = (
        "final_joint_error_delta",
        "final_tcp_position_error_delta",
        "final_tcp_orientation_error_delta",
        "sample_count_delta",
    )
    if not all(row.get(key) == 0 or row.get(key) == 0.0 for key in exact):
        return False
    settling = row.get("settling_time_delta")
    return isinstance(settling, (int, float)) and not isinstance(settling, bool) and abs(float(settling)) <= DRIFT_SETTLING_ABS_SEC


def unbounded_rss_growth(samples: list[int]) -> bool:
    if len(samples) < 3:
        return False
    deltas = [after - before for before, after in zip(samples, samples[1:])]
    if any(delta <= 0 for delta in deltas):
        return False
    return deltas[-1] >= deltas[0]


def _drift_pair(first: dict | None, last: dict | None) -> dict:
    if first is None or last is None:
        return {
            "first_iteration": None if first is None else first["iteration"],
            "last_iteration": None if last is None else last["iteration"],
            "final_joint_error_delta": None,
            "final_tcp_position_error_delta": None,
            "final_tcp_orientation_error_delta": None,
            "settling_time_delta": None,
            "sample_count_delta": None,
        }
    return {
        "first_iteration": first["iteration"],
        "last_iteration": last["iteration"],
        "final_joint_error_delta": _abs_delta(first["final_joint_error_rad"], last["final_joint_error_rad"]),
        "final_tcp_position_error_delta": _abs_delta(first["final_tcp_position_error_m"], last["final_tcp_position_error_m"]),
        "final_tcp_orientation_error_delta": _abs_delta(
            first["final_tcp_orientation_error_rad"],
            last["final_tcp_orientation_error_rad"],
        ),
        "settling_time_delta": _abs_delta(first["settling_time_sec"], last["settling_time_sec"]),
        "sample_count_delta": _abs_delta(first["sample_count"], last["sample_count"]),
    }


def _abs_delta(first: object, last: object) -> float | None:
    if not isinstance(first, (int, float)) or not isinstance(last, (int, float)):
        return None
    if isinstance(first, bool) or isinstance(last, bool):
        return None
    if not math.isfinite(float(first)) or not math.isfinite(float(last)):
        return None
    return abs(float(last) - float(first))
