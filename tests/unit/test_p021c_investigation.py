"""P-021C session-reset classification tests."""

from controller.p021c_investigation import (
    classify_session_reset,
    motion_completed,
    readiness,
    restart_diff,
)


def test_restart_diff_marks_zero_reset_without_inventing_defaults() -> None:
    before = {"Motor[1].ServoCtrl": "1", "Motor[1].JogTa": "-10", "Motor[1].JogPos": "2"}
    after = {"Motor[1].ServoCtrl": "0", "Motor[1].JogTa": "-10", "Motor[1].JogPos": "NOT_AVAILABLE"}
    rows = {row["variable"]: row["tag"] for row in restart_diff(before, after)}
    assert rows["Motor[1].ServoCtrl"] == "RESET_TO_ZERO"
    assert rows["Motor[1].JogTa"] == "UNCHANGED"
    assert rows["Motor[1].JogPos"] == "NOT_AVAILABLE"


def test_ack_does_not_complete_motion_and_closed_loop_is_required() -> None:
    stalled = {"Motor[1].DesPos": "0", "Motor[1].Pos": "0", "Motor[1].ActPos": "0"}
    assert motion_completed(stalled, "2", True) is False
    assert readiness({"Motor[1].ServoCtrl": "0", "Motor[1].ClosedLoop": "0", "Motor[1].FeFatal": "0", "Motor[1].AmpFault": "0", "Motor[1].LimitStop": "0"}) == "NOT_READY"


def test_waiting_is_not_a_transport_reconnect_result() -> None:
    result = classify_session_reset(
        before_completed=False,
        after_ran=False,
        after_completed=None,
        activation_required=False,
        boot_ok=False,
        snapshot_ok=False,
    )
    assert result["classification"] == "WAITING_FOR_MANUAL_RESTART"
    assert result["confirmed"] is False


def test_recovery_without_activation_is_a_candidate_not_a_confirmed_cause() -> None:
    result = classify_session_reset(
        before_completed=False,
        after_ran=True,
        after_completed=True,
        activation_required=False,
        boot_ok=True,
        snapshot_ok=True,
    )
    assert result["classification"] == "RC-S1"
    assert result["causal_claim"] == "SIMULATOR_SESSION_STATE_CANDIDATE"
    assert result["confirmed"] is False


def test_activation_after_restart_requires_a_follow_up() -> None:
    result = classify_session_reset(
        before_completed=False,
        after_ran=True,
        after_completed=True,
        activation_required=True,
        boot_ok=True,
        snapshot_ok=True,
    )
    assert result["classification"] == "RC-S3"
    assert result["causal_claim"] == "FOLLOW-UP CAUSAL TEST REQUIRED"
    assert result["confirmed"] is False


def test_stall_after_restart_weakens_the_session_residue_hypothesis() -> None:
    result = classify_session_reset(
        before_completed=False,
        after_ran=True,
        after_completed=False,
        activation_required=True,
        boot_ok=True,
        snapshot_ok=True,
    )
    assert result["classification"] == "RC-S2"
    assert result["confirmed"] is False
