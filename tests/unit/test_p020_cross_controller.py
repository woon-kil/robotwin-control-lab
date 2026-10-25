"""P-020 semantic comparison. 위치 숫자는 비교하지 않는다."""

from pathlib import Path

from controller.p020_cross_controller import (
    COMPARISON_CLASSES,
    classify,
    disconnect_rows,
    e_fixed_boundary,
    fault_rows,
    invalid_rows,
    positive_rows,
    release_if_safe,
    reset_rows,
    scripted_pmac_positive,
    timeout_rows,
    unsupported_rows,
    virtual_positive_observation,
)

ROOT = Path(__file__).resolve().parents[2]


def test_comparison_classes_cover_match_difference_and_gaps():
    assert COMPARISON_CLASSES == ("MATCH", "DIFFERENT", "NOT_COMPARABLE", "NOT_TESTED")
    assert classify("ACCEPTED", "ACCEPTED") == "MATCH"
    assert classify("REJECTED", "FAULTED") == "DIFFERENT"
    assert classify(None, "REJECTED") == "NOT_TESTED"
    assert classify(0, 0, comparable=False) == "NOT_COMPARABLE"


def test_positive_semantics_match_without_numeric_equivalence():
    rows = positive_rows(virtual_positive_observation(), scripted_pmac_positive())
    by_name = {row["semantic"]: row for row in rows}
    for semantic in ("command_acceptance", "completion", "target_observable", "actual_observable", "fault_free"):
        assert by_name[semantic]["comparison"] == "MATCH"
    numeric = by_name["numeric_position_equality"]
    assert numeric["comparison"] == "NOT_COMPARABLE"
    assert numeric["virtual"]["result"] == "NOT_EVALUATED"
    assert numeric["power_pmac"]["result"] == "NOT_EVALUATED"
    assert numeric["virtual"]["state"]["units"] == "radian"
    assert numeric["power_pmac"]["state"]["units"] == "pmac_motor_unit"


def test_invalid_input_rejects_without_motion_or_fault():
    rows = invalid_rows()
    rejected = [row for row in rows if row["semantic"] == "command_rejected"]
    assert [row["scenario"] for row in rejected] == [
        "invalid_nan",
        "invalid_positive_infinity",
        "invalid_negative_infinity",
        "invalid_non_numeric",
    ]
    assert all(row["comparison"] == "MATCH" for row in rejected)
    assert all(row["virtual"]["command_sent"] is False for row in rejected)
    assert all(row["power_pmac"]["command_sent"] is False for row in rejected)
    assert all(row["power_pmac"]["state"]["jog_count"] == 0 for row in rejected)
    faults = [row for row in rows if row["semantic"] == "fault_not_created"]
    assert all(row["comparison"] == "MATCH" for row in faults)
    codes = [row for row in rows if row["semantic"] == "error_code"]
    assert all(row["comparison"] == "DIFFERENT" for row in codes)


def test_unsupported_scope_is_rejected_on_both_backends():
    rows = unsupported_rows()
    rejected = next(row for row in rows if row["semantic"] == "command_rejected")
    codes = next(row for row in rows if row["semantic"] == "error_code")
    assert rejected["comparison"] == "MATCH"
    assert rejected["virtual"]["command_sent"] is False
    assert rejected["power_pmac"]["command_sent"] is False
    assert rejected["power_pmac"]["state"]["jog_count"] == 0
    assert codes["comparison"] == "DIFFERENT"
    assert codes["virtual"]["result"] == "UNSUPPORTED_COMMAND"
    assert codes["power_pmac"]["result"] == "UNSUPPORTED_MOTOR"


def test_disconnect_blocks_execution_without_stopping_the_simulator():
    rows = disconnect_rows()
    rejected = next(row for row in rows if row["semantic"] == "command_rejected")
    state = next(row for row in rows if row["semantic"] == "disconnected_state")
    assert rejected["comparison"] == "MATCH"
    assert rejected["virtual"]["command_sent"] is False
    assert rejected["power_pmac"]["command_sent"] is False
    assert rejected["power_pmac"]["state"]["jog_count"] == 0
    assert state["comparison"] == "MATCH"
    assert state["virtual"]["result"] == "DISCONNECTED"
    assert state["power_pmac"]["result"] == "DISCONNECTED"


def test_fault_blocks_completion_and_keeps_the_result_token_difference():
    rows = fault_rows()
    by_name = {row["semantic"]: row for row in rows}
    assert by_name["fault_detected"]["comparison"] == "MATCH"
    assert by_name["command_not_executed"]["comparison"] == "MATCH"
    assert by_name["command_not_executed"]["power_pmac"]["state"]["jog_count"] == 0
    assert by_name["not_completed"]["comparison"] == "MATCH"
    assert by_name["result_token"]["comparison"] == "DIFFERENT"
    assert by_name["result_token"]["virtual"]["result"] == "REJECTED"
    assert by_name["result_token"]["power_pmac"]["result"] == "FAULTED"


def test_timeout_stays_not_comparable_and_does_not_complete():
    row = timeout_rows()[0]
    assert row["comparison"] == "NOT_COMPARABLE"
    assert row["virtual"]["result"] == "NOT_PRESENT"
    assert row["power_pmac"]["state"]["result"] == "FAULTED"
    assert row["power_pmac"]["state"]["completed"] is False
    assert row["power_pmac"]["state"]["jog_count"] == 1
    assert row["power_pmac"]["command_sent"] is True


def test_reset_is_not_invented_for_the_pmac_seam():
    row = reset_rows()[0]
    assert row["comparison"] == "NOT_COMPARABLE"
    assert row["power_pmac"]["result"] == "NOT_PRESENT"
    assert row["power_pmac"]["state"]["called"] is False


def test_validator_reject_blocks_virtual_and_pmac():
    boundary = e_fixed_boundary()
    assert boundary["validator_result"] == "REJECT"
    assert boundary["collision_result"] == "FAIL"
    assert boundary["involved_objects"][0] == {"a": "wrist_3_link", "b": "bolt_panel"}
    assert boundary["virtual_backend_calls"] == 0
    assert boundary["execution_gate_submit_count"] == 0
    assert boundary["pmac_backend_calls"] == 0
    assert boundary["harness_virtual_calls"] == 0
    assert boundary["released"] is False
    assert boundary["acceptance_criterion_changed"] is False


def test_safe_decision_is_the_only_release_for_both_backends():
    calls = {"virtual": 0, "pmac": 0}

    def virtual_call() -> None:
        calls["virtual"] += 1

    def pmac_call() -> None:
        calls["pmac"] += 1

    blocked = release_if_safe("REJECT", virtual_call, pmac_call)
    assert blocked["released"] is False
    assert calls == {"virtual": 0, "pmac": 0}
    released = release_if_safe("SAFE_TO_ATTEMPT", virtual_call, pmac_call)
    assert released["released"] is True
    assert calls == {"virtual": 1, "pmac": 1}


def test_existing_contracts_do_not_gain_the_cross_controller_seam():
    gate = (ROOT / "validator" / "gate.py").read_text(encoding="utf-8")
    virtual = (ROOT / "controller" / "virtual_pmac.py").read_text(encoding="utf-8")
    assert "move_test_motor" not in gate
    assert "VirtualPmacController" in gate
    assert "DesPos" not in virtual
    assert "#1j=" not in virtual
