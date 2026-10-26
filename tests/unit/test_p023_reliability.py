"""P-023 reliability accounting tests. Simulator에는 접속하지 않는다."""

from controller.p023_reliability import (
    alternate_targets,
    apply_event,
    classify_reliability,
    disconnect_blocked,
    empty_counters,
    evaluate_observation,
    motion_completed,
    percentile,
    reconnect_plan,
    state_leakage,
    stop_decision,
)
from digital_twin.controller_axis import MAPPING_BOUNDARY

OBSERVED = "2026-10-26T00:00:00+00:00"


def _post(target: str = "2", **overrides: str) -> dict[str, str]:
    values = {
        "Motor[1].JogPos": target,
        "Motor[1].DesPos": target,
        "Motor[1].Pos": target,
        "Motor[1].ActPos": target,
        "Motor[1].ServoCtrl": "1",
        "Motor[1].ClosedLoop": "1",
        "Motor[1].InPos": "1",
        "Motor[1].FeFatal": "0",
        "Motor[1].FeWarn": "0",
        "Motor[1].AmpFault": "0",
        "Motor[1].LimitStop": "0",
    }
    values.update(overrides)
    return values


def _eval(post: dict[str, str], target: str = "2", **kwargs: object) -> dict[str, object]:
    fresh = bool(kwargs.get("fresh", True))
    communication = str(kwargs.get("communication", "HEALTHY"))
    error_present = bool(kwargs.get("error_present", False))
    return evaluate_observation(
        post,
        target=target,
        ack_present=True,
        error_present=error_present,
        fresh=fresh,
        communication_state=communication,
        observed_at=OBSERVED,
    )


def test_target_sequence_alternates_for_one_hundred_cycles() -> None:
    targets = alternate_targets("1", 100)
    assert targets is not None
    assert len(targets) == 100
    assert targets[:6] == [2, 3, 1, 2, 3, 1]
    assert all(left != right for left, right in zip(targets, targets[1:]))


def test_ack_alone_does_not_complete() -> None:
    stalled = {"Motor[1].JogPos": "2", "Motor[1].DesPos": "1", "Motor[1].Pos": "1", "Motor[1].ActPos": "1"}
    assert motion_completed(stalled, "2", True) is False


def test_successful_cycle_accounting() -> None:
    counters = empty_counters()
    updated = apply_event(counters, "COMPLETED")
    assert updated["attempted_cycles"] == 1
    assert updated["completed_cycles"] == 1
    assert updated["failed_cycles"] == 0


def test_tracking_mismatch_is_counted_and_does_not_stop() -> None:
    result = _eval(_post("1"), target="2")
    counters = apply_event(empty_counters(), str(result["event"]))
    assert result["event"] == "TRACKING_MISMATCH"
    assert result["stop"] is False
    assert counters["tracking_mismatch_count"] == 1


def test_timeout_disconnect_and_protocol_are_counted() -> None:
    timeout = apply_event(empty_counters(), "TIMEOUT")
    disconnected = apply_event(empty_counters(), "DISCONNECT")
    protocol = apply_event(empty_counters(), "PROTOCOL_ERROR")
    assert timeout["timeout_count"] == 1
    assert disconnected["disconnect_count"] == 1
    assert protocol["protocol_error_count"] == 1
    assert stop_decision("TIMEOUT") is True
    assert stop_decision("DISCONNECT") is True
    assert stop_decision("PROTOCOL_ERROR") is True


def test_fault_closed_loop_loss_and_malformed_state_stop() -> None:
    fault = _eval(_post(**{"Motor[1].AmpFault": "1"}))
    closed = _eval(_post(**{"Motor[1].ClosedLoop": "0"}))
    malformed = _eval(_post(**{"Motor[1].ActPos": "malformed"}))
    assert fault["event"] == "FAULT" and fault["stop"] is True
    assert closed["event"] == "CLOSED_LOOP_LOST" and closed["stop"] is True
    assert malformed["event"] == "MALFORMED" and malformed["stop"] is True


def test_state_leakage_is_recorded_without_a_cause() -> None:
    assert state_leakage({"Motor[1].ActPos": "2"}, {"Motor[1].ActPos": "1"}) is True
    assert state_leakage({"Motor[1].ActPos": "2"}, {"Motor[1].ActPos": "2"}) is False
    assert state_leakage(None, {"Motor[1].ActPos": "2"}) is False


def test_runtime_failure_is_not_a_confirmed_cause() -> None:
    result = classify_reliability(
        preflight_transport=True,
        preflight_ready=True,
        motion_attempted=10,
        motion_completed=10,
        motion_defect_count=0,
        reconnect_ran=False,
        reconnect_acceptance=False,
        controller_stop=False,
        transport_stop=False,
        runtime_failure=True,
        final_ready=True,
        negative_pass=True,
        domain_ok=True,
        forbidden_write=False,
        regression_passed=True,
        evidence_complete=True,
    )
    assert result["classification"] == "REL-6"
    assert result["root_cause"] == "NOT_CONFIRMED"
    assert result["confirmed"] is False


def test_disconnected_command_is_blocked() -> None:
    assert disconnect_blocked(result="REJECTED", error_code="DISCONNECTED", command_sent=False, jog_observed=False) is True
    assert disconnect_blocked(result="REJECTED", error_code="DISCONNECTED", command_sent=True, jog_observed=False) is False


def test_reconnect_requires_a_fresh_ready_read_and_does_not_activate() -> None:
    missing = reconnect_plan(fresh_read=False, ready=True)
    not_ready = reconnect_plan(fresh_read=True, ready=False)
    ready = reconnect_plan(fresh_read=True, ready=True)
    assert missing["motion"] is False
    assert missing["reason"] == "FRESH_READ_REQUIRED"
    assert not_ready["reason"] == "RECONNECT_NOT_READY"
    assert not_ready["activation"] == []
    assert ready["activation"] == []
    assert "Motor[1].ServoCtrl=1" not in ready["activation"]
    assert "#1J/" not in ready["activation"]


def test_domain_boundary_stays_undefined() -> None:
    assert MAPPING_BOUNDARY["robot_joint_mapping"] == "NOT_DEFINED"
    assert MAPPING_BOUNDARY["pmac_motor_unit_to_rad"] == "NOT_DEFINED"
    assert MAPPING_BOUNDARY["Motor[1]"] == "controller_axis_only"


def test_percentiles_use_linear_interpolation() -> None:
    values = [0.0, 10.0, 20.0, 30.0, 40.0]
    assert percentile(values, 50) == 20.0
    assert percentile(values, 95) == 38.0
    assert percentile(values, 99) == 39.6
    assert percentile(values, 100) == 40.0


def test_full_pass_is_not_a_final_pass_declaration() -> None:
    result = classify_reliability(
        preflight_transport=True,
        preflight_ready=True,
        motion_attempted=100,
        motion_completed=100,
        motion_defect_count=0,
        reconnect_ran=True,
        reconnect_acceptance=True,
        controller_stop=False,
        transport_stop=False,
        runtime_failure=False,
        final_ready=True,
        negative_pass=True,
        domain_ok=True,
        forbidden_write=False,
        regression_passed=True,
        evidence_complete=True,
    )
    assert result["classification"] == "REL-1"
    assert result["p023_final_pass_declared"] is False
    assert result["industrial_reliability_claimed"] is False
