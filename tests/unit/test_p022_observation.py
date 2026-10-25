"""P-022 controller-domain observation tests."""

import pytest

from controller.p022_integration import classify_integration, motion_completed
from digital_twin.controller_axis import (
    MAPPING_BOUNDARY,
    ControllerObservationBridge,
    DomainBoundaryError,
    accept_tracking,
    normalize_motor_snapshot,
)
from digital_twin.monitor import DigitalTwinMonitor

OBSERVED_AT = "2026-10-26T00:00:00+00:00"


def _snapshot(**overrides: str) -> dict[str, str]:
    values = {
        "Motor[1].DesPos": "2",
        "Motor[1].Pos": "2",
        "Motor[1].ActPos": "2",
        "Motor[1].JogPos": "2",
        "Motor[1].ClosedLoop": "1",
        "Motor[1].InPos": "1",
        "Motor[1].FeFatal": "0",
        "Motor[1].FeWarn": "0",
        "Motor[1].AmpFault": "0",
        "Motor[1].LimitStop": "0",
    }
    values.update(overrides)
    return values


def _observe(fresh: bool = True, communication: str = "HEALTHY", target: float | None = 2, **overrides: str):
    snapshot = _snapshot(**overrides)
    return normalize_motor_snapshot(
        snapshot,
        communication_state=communication,
        command_target=target,
        observed_at=OBSERVED_AT,
        fresh=fresh,
    )


def test_valid_observation_stays_in_pmac_motor_units() -> None:
    observation = _observe()
    assert observation.valid is True
    assert observation.invalid_reason is None
    assert observation.unit == "pmac_motor_unit"
    assert observation.controller_id == "power_pmac_simulator"
    assert observation.axis_id == "Motor[1]"
    assert observation.source == "official_power_pmac_simulator"
    assert observation.desired_position == 2
    assert observation.actual_position == 2


def test_tracking_error_is_absolute_motor_unit_difference() -> None:
    matched = _observe()
    assert matched.tracking_error == 0
    assert matched.tracking_success is True
    offset = _observe(**{"Motor[1].ActPos": "1", "Motor[1].DesPos": "1", "Motor[1].Pos": "1"}, target=3)
    assert offset.tracking_error == 2
    assert offset.unit == "pmac_motor_unit"
    assert offset.tracking_success is False


def test_fresh_post_is_required_for_a_valid_observation() -> None:
    fresh = _observe(fresh=True)
    stale = _observe(fresh=False)
    assert fresh.valid is True
    assert stale.valid is False
    assert stale.invalid_reason == "STALE"
    assert accept_tracking(stale) is False


def test_disconnected_observation_is_rejected() -> None:
    observation = _observe(communication="DISCONNECTED")
    assert observation.valid is False
    assert observation.invalid_reason == "DISCONNECTED"
    assert accept_tracking(observation) is False


def test_amp_fault_is_not_tracking_success() -> None:
    observation = _observe(**{"Motor[1].AmpFault": "1"})
    assert observation.faulted is True
    assert observation.fault_code == "AMP_FAULT"
    assert observation.tracking_success is False
    assert accept_tracking(observation) is False


def test_malformed_actual_position_is_rejected() -> None:
    observation = _observe(**{"Motor[1].ActPos": "malformed"})
    assert observation.valid is False
    assert observation.invalid_reason == "MALFORMED"
    assert observation.actual_position is None
    assert accept_tracking(observation) is False


def test_missing_desired_position_is_rejected() -> None:
    snapshot = _snapshot()
    del snapshot["Motor[1].DesPos"]
    observation = normalize_motor_snapshot(
        snapshot,
        communication_state="HEALTHY",
        command_target=2,
        observed_at=OBSERVED_AT,
        fresh=True,
    )
    assert observation.valid is False
    assert observation.invalid_reason == "INCOMPLETE"
    assert accept_tracking(observation) is False


def test_robot_unit_request_is_fail_closed() -> None:
    bridge = ControllerObservationBridge()
    with pytest.raises(DomainBoundaryError):
        bridge.request_unit_conversion("pmac_motor_unit", "rad")


def test_bridge_does_not_map_motor_1_onto_the_robot_monitor() -> None:
    monitor = DigitalTwinMonitor()
    bridge = ControllerObservationBridge()
    bridge.publish(_observe())
    assert monitor.samples == ()
    assert monitor.status == "IDLE"
    assert MAPPING_BOUNDARY["robot_joint_mapping"] == "NOT_DEFINED"
    assert MAPPING_BOUNDARY["pmac_motor_unit_to_rad"] == "NOT_DEFINED"
    assert MAPPING_BOUNDARY["UR10e_mapping"] == "NOT_DEFINED"
    published = bridge.published[0].to_dict()
    assert published["unit_conversion"] == "NOT_DEFINED"
    assert "joint_position_rad" not in published


def test_ack_alone_does_not_complete_motion() -> None:
    sample = {"Motor[1].DesPos": "0", "Motor[1].Pos": "0", "Motor[1].ActPos": "0", "Motor[1].JogPos": "2"}
    assert motion_completed(sample, "2", True) is False


def test_full_pass_is_not_a_final_pass_declaration() -> None:
    result = classify_integration(
        precondition_ready=True,
        cycles_completed=3,
        observation_ok=True,
        domain_violation=False,
        fail_closed_ok=True,
        regression_passed=True,
        forbidden_write=False,
    )
    assert result["classification"] == "DT-1"
    assert result["p022_final_pass_declared"] is False
