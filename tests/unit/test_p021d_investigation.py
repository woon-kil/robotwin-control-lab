"""P-021D reactivation classification tests."""

from controller.p021d_investigation import classify_reactivation, motion_completed


def _classify(**overrides: object) -> dict[str, object]:
    values: dict[str, object] = {
        "evidence_complete": True,
        "unexpected_write": False,
        "restart_performed": False,
        "probe_results": (),
        "stall_servo_ok": None,
        "stall_closed_loop_ok": None,
        "stall_fault_present": None,
        "intervention": None,
        "recovery_result": None,
        "cycles": 1,
    }
    values.update(overrides)
    return classify_reactivation(**values)  # type: ignore[arg-type]


def test_healthy_probes_do_not_count_as_reactivation() -> None:
    result = _classify(probe_results=("COMPLETED", "COMPLETED", "COMPLETED"))
    assert result["classification"] == "RD-4"
    assert result["operational_state"] == "P021D_WAITING_FOR_STALL"
    assert result["confirmed"] is False


def test_one_reactivation_cycle_is_associated_below_high_confidence() -> None:
    result = _classify(
        probe_results=("RB-1",),
        stall_servo_ok=True,
        stall_closed_loop_ok=True,
        stall_fault_present=False,
        intervention="#1J/",
        recovery_result="COMPLETED",
        cycles=1,
    )
    assert result["classification"] == "RD-1"
    assert result["confidence"] == "MEDIUM"
    assert result["causal_claim"] == "#1J/ reactivation is causally associated with restoring motion generation"
    assert result["confirmed"] is False
    repeated = _classify(
        probe_results=("RB-1",),
        stall_servo_ok=True,
        stall_closed_loop_ok=True,
        stall_fault_present=False,
        intervention="#1J/",
        recovery_result="COMPLETED",
        cycles=2,
    )
    assert repeated["confidence"] == "HIGH"
    assert repeated["confirmed"] is False


def test_stall_after_reactivation_weakens_the_single_command_hypothesis() -> None:
    result = _classify(
        probe_results=("RB-1",),
        stall_servo_ok=True,
        stall_closed_loop_ok=True,
        stall_fault_present=False,
        intervention="#1J/",
        recovery_result="RB-1",
    )
    assert result["classification"] == "RD-2"
    assert result["confirmed"] is False


def test_lost_activation_is_not_the_jog_stop_experiment() -> None:
    result = _classify(
        probe_results=("RB-1",),
        stall_servo_ok=False,
        stall_closed_loop_ok=False,
        stall_fault_present=False,
        intervention=None,
    )
    assert result["classification"] == "RD-3"
    assert result["confirmed"] is False


def test_mixed_outcomes_without_intervention_are_intermittent() -> None:
    result = _classify(
        probe_results=("COMPLETED", "RB-1", "COMPLETED"),
        stall_servo_ok=True,
        stall_closed_loop_ok=True,
        stall_fault_present=False,
        intervention=None,
    )
    assert result["classification"] == "RD-5"
    assert result["confirmed"] is False


def test_ack_with_jog_position_only_is_not_completion() -> None:
    stalled = {
        "Motor[1].JogPos": "2",
        "Motor[1].DesPos": "0",
        "Motor[1].Pos": "0",
        "Motor[1].ActPos": "0",
        "Motor[1].DesVel": "0",
    }
    assert motion_completed(stalled, "2", True) is False


def test_unexpected_write_or_incomplete_evidence_is_invalid() -> None:
    incomplete = _classify(evidence_complete=False, probe_results=("COMPLETED", "COMPLETED", "COMPLETED"))
    unexpected = _classify(unexpected_write=True, probe_results=("RB-1",), stall_servo_ok=True, stall_closed_loop_ok=True, stall_fault_present=False, intervention="#1J/", recovery_result="COMPLETED")
    assert incomplete["classification"] == "RD-6"
    assert unexpected["classification"] == "RD-6"
    assert incomplete["confirmed"] is False
