"""P-011 reliability scoring. 500-cycle 측정은 evidence runner가 한다."""

import math

import pytest

from reliability.metrics import classify_cycle, drift_matches, percentile, summarize, unbounded_rss_growth, validation_matrix
from reliability.model import DRIFT_SETTLING_ABS_SEC, OUTPUT, load_reliability_config
from reliability.evidence import assert_output_directory


def test_workload_is_500_interleaved_cycles():
    config = load_reliability_config()
    assert config["total_cycles"] == 500
    assert config["iterations"] == 100
    assert config["sequence"] == ["A-zero", "B-positive-1", "C-mixed-1", "E-fixed-1", "E-fixed-3"]
    assert config["fault_injection_in_primary"] is False
    assert config["recovery_attempts"] == 10


def test_percentile_and_summary_use_the_sorted_sample():
    assert percentile([1.0, 2.0, 3.0, 4.0], 50) == pytest.approx(2.5)
    summary = summarize([0.0, 1.0, 2.0, 3.0])
    assert summary["min"] == 0.0
    assert summary["max"] == 3.0
    assert summary["mean"] == 1.5
    assert summary["count"] == 4
    assert summarize([])["mean"] is None


def test_reject_submission_is_the_critical_category():
    row = _row("E-fixed-1", "REJECT", "SAFE_TO_ATTEMPT", submitted=True, tracked=True)
    assert classify_cycle(row) == "UNSAFE_SUBMISSION"
    blocked = _row("E-fixed-1", "REJECT", "REJECT", submitted=False, tracked=False, reasons=["COLLISION"])
    assert classify_cycle(blocked) is None


def test_stale_execute_does_not_pass_a_reject():
    row = _row("E-fixed-1", "REJECT", "REJECT", submitted=True, tracked=False, reasons=["COLLISION"])
    row["state_leak"] = True
    assert classify_cycle(row) == "UNSAFE_SUBMISSION"


def test_safe_cycle_must_settle_and_separate():
    safe = _row("B-positive-1", "SAFE_TO_ATTEMPT", "SAFE_TO_ATTEMPT", submitted=True, tracked=True)
    safe.update({"completed": True, "settled": True, "separation_observed": True})
    assert classify_cycle(safe) is None
    safe["settled"] = False
    assert classify_cycle(safe) == "NOT_SETTLED"
    safe["settled"] = True
    safe["separation_observed"] = False
    assert classify_cycle(safe) == "METRIC_FAILURE"


def test_non_finite_and_fault_are_not_success():
    row = _row("A-zero", "SAFE_TO_ATTEMPT", "SAFE_TO_ATTEMPT", submitted=True, tracked=True)
    row.update({"completed": True, "settled": True, "non_finite": True})
    assert classify_cycle(row) == "NON_FINITE"
    row["non_finite"] = False
    row["fault"] = True
    row["completed"] = False
    row["settled"] = False
    assert classify_cycle(row) == "UNEXPECTED_FAULT"


def test_validation_matrix_counts_both_axes():
    rows = [
        _row("A-zero", "SAFE_TO_ATTEMPT", "SAFE_TO_ATTEMPT", submitted=True, tracked=True),
        _row("E-fixed-1", "REJECT", "REJECT", submitted=False, tracked=False, reasons=["COLLISION"]),
    ]
    matrix = validation_matrix(rows)
    assert matrix["expected_safe_actual_safe"] == 1
    assert matrix["expected_reject_actual_reject"] == 1
    assert matrix["expected_safe_actual_reject"] == 0
    assert matrix["expected_reject_actual_safe"] == 0


def test_rss_growth_must_keep_increasing_to_count_as_unbounded():
    assert unbounded_rss_growth([100, 200, 300, 400]) is True
    assert unbounded_rss_growth([100, 180, 190, 191]) is False


def test_writer_refuses_other_evidence_directories(tmp_path):
    with pytest.raises(RuntimeError):
        assert_output_directory(tmp_path)
    assert assert_output_directory(OUTPUT) == OUTPUT.resolve()


def test_settling_drift_bound_is_below_one_sample():
    assert DRIFT_SETTLING_ABS_SEC < 0.01
    noisy = {
        "final_joint_error_delta": 0.0,
        "final_tcp_position_error_delta": 0.0,
        "final_tcp_orientation_error_delta": 0.0,
        "sample_count_delta": 0.0,
        "settling_time_delta": 1.3e-13,
    }
    assert drift_matches(noisy) is True
    noisy["settling_time_delta"] = 0.01
    assert drift_matches(noisy) is False
    noisy["settling_time_delta"] = 1.3e-13
    noisy["sample_count_delta"] = 1.0
    assert drift_matches(noisy) is False


def test_nan_is_not_a_finite_metric():
    assert math.isfinite(float("nan")) is False


def _row(case_name: str, expected: str, actual: str, submitted: bool, tracked: bool, reasons: list[str] | None = None) -> dict:
    return {
        "case_name": case_name,
        "expected_decision": expected,
        "actual_decision": actual,
        "controller_submitted": submitted,
        "tracking_started": tracked,
        "completed": False,
        "settled": False,
        "timeout": False,
        "non_finite": False,
        "fault": False,
        "state_leak": False,
        "reset_result": "NOT_REQUIRED",
        "validation_reason": reasons or [],
        "separation_observed": False,
        "result": "PASS",
    }
