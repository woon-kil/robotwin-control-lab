"""P-021 software recovery. live simulator와 hardware fault reset은 여기 없다."""

from controller.p020_cross_controller import ExchangeMotor
from controller.p021_recovery import (
    move_if_recovered,
    recovery_leaves_validator_closed,
    scripted_disconnect_recovery,
    scripted_fault_blocks,
    scripted_fault_cleared_but_not_ready,
    scripted_fault_recovery,
    scripted_fault_remains,
    scripted_malformed_recovery,
    scripted_reconnect_does_not_assume_ready,
    scripted_reconnect_failure,
    scripted_warning_does_not_block_recovery,
    virtual_fault_recovery,
)
from controller.power_pmac_adapter import PowerPmacControllerAdapter
from controller.power_pmac_motor import readiness_result
from controller.power_pmac_transport import ScriptedTransport


def test_disconnect_blocks_motion_and_sends_no_jog():
    row = scripted_disconnect_recovery()
    assert row["blocked_result"] == "REJECTED"
    assert row["blocked_error"] == "DISCONNECTED"
    assert row["blocked_sent"] is False
    assert row["verification"] == "RECOVERED"
    assert row["post_result"] == "COMPLETED"
    assert row["post_command"] == "#1j=2"
    assert row["jog_count"] == 1


def test_reconnect_does_not_imply_ready_and_reads_again():
    row = scripted_reconnect_does_not_assume_ready()
    assert row["cached_ready"] == "RECOVERED"
    assert row["communication"] == "HEALTHY"
    assert row["result"] == "NOT_READY"
    assert row["read_after_reconnect"] is True
    assert row["servo_ctrl"] == 0
    assert row["closed_loop"] is False
    assert row["command_sent"] is False
    assert row["jog_count"] == 0


def test_fault_blocks_motion_and_sends_no_jog():
    for name in ("AmpFault", "FeFatal", "LimitStop"):
        row = scripted_fault_blocks(name)
        assert row["result"] == "FAULTED"
        assert row["command_sent"] is False
        assert row["jog_count"] == 0
        assert row["completed"] is False


def test_fault_removed_but_not_ready_stays_blocked():
    row = scripted_fault_cleared_but_not_ready()
    assert row["result"] == "NOT_READY"
    assert row["command_sent"] is False
    assert row["jog_count"] == 0
    assert row["activation_count"] == 0
    assert row["enable_count"] == 0


def test_successful_scripted_recovery_completes_a_new_command():
    row = scripted_fault_recovery()
    assert row["blocked_sent"] is False
    assert row["verification"] == "RECOVERED"
    assert row["post_command"] == "#1j=2"
    assert row["post_result"] == "COMPLETED"
    assert row["jog_count"] == 1
    assert row["live_fault_injection"] == "NOT_TESTED"


def test_failed_recovery_and_malformed_response_and_reconnect_failure_block_motion():
    remains = scripted_fault_remains()
    assert remains["result"] == "FAULT_PRESENT"
    assert remains["command_sent"] is False
    assert remains["jog_count"] == 0
    malformed = scripted_malformed_recovery()
    assert malformed["result"] == "PROTOCOL_ERROR"
    assert malformed["command_sent"] is False
    assert malformed["jog_count"] == 0
    failed = scripted_reconnect_failure()
    assert failed["result"] == "DISCONNECTED"
    assert failed["communication"] == "DISCONNECTED"
    assert failed["command_sent"] is False
    assert failed["jog_count"] == 0


def test_warning_is_not_a_fault_recovery_failure():
    row = scripted_warning_does_not_block_recovery()
    assert row["result"] == "RECOVERED"
    assert row["fault_code"] is None
    assert row["command_sent"] is False


def test_forged_recovered_label_does_not_send_motion():
    executor = ExchangeMotor({"Motor[1].AmpFault": "1"})
    transport = ScriptedTransport()
    transport.queue_read("1")
    adapter = PowerPmacControllerAdapter(transport, motor_executor=executor)
    adapter.connect()
    state = adapter.recover_test_motor().state
    forged = readiness_result("FAULT_PRESENT", state)
    forged = type(forged)(
        result="RECOVERED",
        error_code=None,
        stages=("RECOVERY_VERIFIED",),
        command=None,
        command_sent=False,
        commands_sent=(),
        state=state,
        poll_count=0,
        elapsed_s=0.0,
        response_text="",
        activation_write=False,
        enable_command_sent=False,
    )
    moved = move_if_recovered(adapter, forged, 2)
    assert moved.result == "RECOVERY_FAILED"
    assert moved.command_sent is False
    calls = executor.calls
    assert not any(len(call) == 1 and call[0].startswith("#1j=") for call in calls)


def test_virtual_recovery_uses_existing_reset_and_a_new_command():
    row = virtual_fault_recovery()
    assert row["blocked_result"] == "REJECTED"
    assert row["blocked_sent"] is False
    assert row["after_reset_state"] == "CONNECTED"
    assert row["before_ready_result"] == "REJECTED"
    assert row["before_ready_sent"] is False
    assert row["post_command_id"] == "p021-virtual-after"
    assert row["post_result"] == "COMPLETED"
    assert row["post_trace"] == ["ACCEPTED", "EXECUTING", "COMPLETED"]
    assert row["same_recovery_implementation"] is False


def test_recovery_does_not_bypass_validator():
    row = recovery_leaves_validator_closed()
    assert row["verification"] == "RECOVERED"
    assert row["validator_result"] == "REJECT"
    assert row["virtual_backend_calls"] == 0
    assert row["pmac_backend_calls"] == 0
    assert row["released"] is False
    assert row["jog_count"] == 0
    assert row["boundary"]["involved_objects"][0] == {"a": "wrist_3_link", "b": "bolt_panel"}
