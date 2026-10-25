"""P-021 final recovery lifecycle classification tests."""

from controller.p021_final_investigation import classify_final_lifecycle, motion_completed


def _classify(**overrides: object) -> dict[str, object]:
    values: dict[str, object] = {
        "evidence_complete": True,
        "unexpected_write": False,
        "restart_performed": False,
        "blind_reactivation": False,
        "precondition_ready": True,
        "precondition_completed": True,
        "disconnect_verified": True,
        "blocked_result": "REJECTED",
        "blocked_error": "DISCONNECTED",
        "command_sent": False,
        "simulator_alive": True,
        "reconnect_healthy": True,
        "fresh_read": True,
        "readiness": "RECOVERED",
        "new_target_distinct": True,
        "motion_attempted": True,
        "motion_accepted": True,
        "motion_completed_value": True,
        "rb1": False,
        "faults_clear": True,
    }
    values.update(overrides)
    return classify_final_lifecycle(**values)  # type: ignore[arg-type]


def test_disconnect_blocks_before_any_recovery_motion() -> None:
    result = _classify()
    assert result["classification"] == "PF-1"
    blocked = _classify(blocked_result="ACCEPTED", command_sent=True)
    assert blocked["classification"] == "PF-2"


def test_reconnect_alone_is_not_completed_recovery() -> None:
    result = _classify(fresh_read=False, readiness=None, motion_attempted=False, motion_completed_value=None, rb1=None, faults_clear=None)
    assert result["classification"] != "PF-1"
    assert result["final_pass_declared"] is False


def test_fresh_read_is_required() -> None:
    result = _classify(fresh_read=False, motion_attempted=False, motion_completed_value=None)
    assert result["classification"] == "PF-7"


def test_disconnected_command_cannot_send_jog() -> None:
    result = _classify(command_sent=True, motion_attempted=False, motion_completed_value=None)
    assert result["classification"] == "PF-2"
    assert result["final_pass_declared"] is False


def test_ready_fresh_read_can_complete_a_new_motion() -> None:
    result = _classify()
    assert result["classification"] == "PF-1"
    assert result["classification_name"] == "FULL_RECOVERY_PASS"
    assert result["confirmed"] is False
    assert result["final_pass_declared"] is False


def test_ready_fresh_read_keeps_rb1_as_failure() -> None:
    result = _classify(motion_completed_value=False, rb1=True, faults_clear=True)
    assert result["classification"] == "PF-5"
    assert result["final_pass_declared"] is False


def test_activation_required_path_still_needs_new_motion() -> None:
    lost = _classify(readiness="NOT_READY", motion_attempted=False, motion_completed_value=None, rb1=None, faults_clear=None)
    assert lost["classification"] == "PF-4"
    recovered = _classify()
    assert recovered["classification"] == "PF-1"


def test_ack_with_unchanged_positions_is_not_completion() -> None:
    stalled = {"Motor[1].JogPos": "2", "Motor[1].DesPos": "0", "Motor[1].Pos": "0", "Motor[1].ActPos": "0"}
    assert motion_completed(stalled, "2", True) is False


def test_faulted_state_remains_blocked() -> None:
    result = _classify(readiness="FAULT_PRESENT", motion_attempted=False, motion_accepted=False, motion_completed_value=None, rb1=None, faults_clear=False)
    assert result["classification"] == "PF-4"
    assert result["final_pass_declared"] is False


def test_incomplete_evidence_is_invalid() -> None:
    result = _classify(evidence_complete=False)
    assert result["classification"] == "PF-7"
    assert result["confirmed"] is False
