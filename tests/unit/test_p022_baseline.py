"""P-022 clean-restart baseline decision tests."""

from controller.p022_baseline import (
    SNAPSHOT_NAMES,
    activation_decision,
    allowed_writes,
    classify_baseline,
    motion_completed,
    readback_ok,
)


def _snapshot(**overrides: str) -> dict[str, str]:
    values = {name: "0" for name in SNAPSHOT_NAMES}
    values["Motor[1].Status[0]"] = "$0"
    values["Motor[1].Status[1]"] = "$0"
    values.update(overrides)
    return values


def test_untouched_snapshot_comes_before_any_write() -> None:
    assert allowed_writes(transport_ok=True, snapshot=None, readback_failed=False) == []


def test_servo_already_on_does_not_write_servo_ctrl() -> None:
    snapshot = _snapshot(**{"Motor[1].ServoCtrl": "1", "Motor[1].ClosedLoop": "1"})
    assert activation_decision(snapshot)["command"] is None
    assert "Motor[1].ServoCtrl=1" not in allowed_writes(transport_ok=True, snapshot=snapshot, readback_failed=False)


def test_closed_loop_already_on_does_not_send_jog_stop() -> None:
    ready = _snapshot(**{"Motor[1].ServoCtrl": "1", "Motor[1].ClosedLoop": "1"})
    servo_off = _snapshot(**{"Motor[1].ServoCtrl": "0", "Motor[1].ClosedLoop": "1"})
    assert activation_decision(ready)["command"] != "#1J/"
    assert allowed_writes(transport_ok=True, snapshot=servo_off, readback_failed=False) == ["Motor[1].ServoCtrl=1"]


def test_fault_blocks_activation() -> None:
    snapshot = _snapshot(**{"Motor[1].ServoCtrl": "0", "Motor[1].FeFatal": "1"})
    assert activation_decision(snapshot)["status"] == "FAULT"
    assert allowed_writes(transport_ok=True, snapshot=snapshot, readback_failed=False) == []


def test_failed_activation_readback_allows_no_further_write() -> None:
    snapshot = _snapshot(**{"Motor[1].ServoCtrl": "0"})
    assert readback_ok("Motor[1].ServoCtrl=1", "0") is False
    assert allowed_writes(transport_ok=True, snapshot=snapshot, readback_failed=True) == []


def test_ack_alone_does_not_complete_motion() -> None:
    sample = {
        "Motor[1].JogPos": "1",
        "Motor[1].DesPos": "0",
        "Motor[1].Pos": "0",
        "Motor[1].ActPos": "0",
    }
    assert motion_completed(sample, "1", True) is False


def test_transport_failure_allows_no_motor_write() -> None:
    snapshot = _snapshot(**{"Motor[1].ServoCtrl": "0"})
    assert allowed_writes(transport_ok=False, snapshot=snapshot, readback_failed=False) == []
    result = classify_baseline(
        transport_ok=False,
        gpascii_ok=False,
        snapshot_recorded=False,
        snapshot_complete=False,
        writes_before_snapshot=False,
        fault_present=False,
        activation_required=False,
        activation_readback_ok=True,
        ready=False,
        motion_completed=None,
        unexpected_write=False,
    )
    assert result["classification"] == "BR-2"
    assert result["p022_final_pass_declared"] is False


def test_ready_motion_is_not_a_final_pass_declaration() -> None:
    result = classify_baseline(
        transport_ok=True,
        gpascii_ok=True,
        snapshot_recorded=True,
        snapshot_complete=True,
        writes_before_snapshot=False,
        fault_present=False,
        activation_required=True,
        activation_readback_ok=True,
        ready=True,
        motion_completed=True,
        unexpected_write=False,
    )
    assert result["classification"] == "BR-1"
    assert result["root_cause"] == "NOT_CONFIRMED"
    assert result["p022_final_pass_declared"] is False
