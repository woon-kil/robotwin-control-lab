"""Controller-domain observation.

Robot-domain radian/meter Target과 Actual을 바꾸지 않는다.
PMAC motor unit을 rad, degree, meter로 바꾸지 않는다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

CONTROLLER_ID = "power_pmac_simulator"
AXIS_ID = "Motor[1]"
UNIT = "pmac_motor_unit"
SOURCE = "official_power_pmac_simulator"

MAPPING_BOUNDARY = {
    "Motor[1]": "controller_axis_only",
    "robot_joint_mapping": "NOT_DEFINED",
    "UR10e_mapping": "NOT_DEFINED",
    "pmac_motor_unit_to_rad": "NOT_DEFINED",
    "pmac_motor_unit_to_degree": "NOT_DEFINED",
    "physical_encoder": "NOT_PRESENT",
    "physical_robot": "NOT_PRESENT",
}


class DomainBoundaryError(Exception):
    """Robot-domain 단위 요청은 변환하지 않고 거부한다."""


@dataclass(frozen=True)
class ControllerAxisObservation:
    controller_id: str
    axis_id: str
    command_target: float | None
    desired_position: float | None
    actual_position: float | None
    unit: str
    closed_loop: bool | None
    in_position: bool | None
    faulted: bool
    fault_code: str | None
    communication_state: str
    observed_at: str
    source: str
    valid: bool
    invalid_reason: str | None
    tracking_error: float | None
    tracking_success: bool

    def to_dict(self) -> dict[str, object]:
        return {
            "controller_id": self.controller_id,
            "axis_id": self.axis_id,
            "command_target": self.command_target,
            "desired_position": self.desired_position,
            "actual_position": self.actual_position,
            "unit": self.unit,
            "closed_loop": self.closed_loop,
            "in_position": self.in_position,
            "faulted": self.faulted,
            "fault_code": self.fault_code,
            "communication_state": self.communication_state,
            "observed_at": self.observed_at,
            "source": self.source,
            "valid": self.valid,
            "invalid_reason": self.invalid_reason,
            "tracking_error": self.tracking_error,
            "tracking_error_unit": UNIT,
            "tracking_success": self.tracking_success,
            "robot_joint_mapping": "NOT_DEFINED",
            "unit_conversion": "NOT_DEFINED",
        }


class ControllerObservationBridge:
    """Read된 motor state를 controller-domain observation으로만 올린다."""

    def __init__(self) -> None:
        self.published: list[ControllerAxisObservation] = []

    def publish(self, observation: ControllerAxisObservation) -> ControllerAxisObservation:
        if observation.unit != UNIT:
            raise DomainBoundaryError("controller observation unit is not pmac_motor_unit")
        if observation.axis_id != AXIS_ID:
            raise DomainBoundaryError("controller observation axis is not Motor[1]")
        self.published.append(observation)
        return observation

    def request_unit_conversion(self, source_unit: str, requested_unit: str) -> float:
        del source_unit, requested_unit
        raise DomainBoundaryError("pmac_motor_unit conversion is NOT_DEFINED")


def normalize_motor_snapshot(
    snapshot: dict[str, str | None],
    *,
    communication_state: str,
    command_target: float | None,
    observed_at: str,
    fresh: bool,
) -> ControllerAxisObservation:
    """Snapshot을 정규화한다. motion, activation, fault reset, 단위 변환은 하지 않는다."""
    invalid = _invalid_reason(snapshot, communication_state, fresh)
    desired, _desired_problem = _parse(snapshot.get("Motor[1].DesPos"))
    actual, _actual_problem = _parse(snapshot.get("Motor[1].ActPos"))
    closed = _flag(snapshot.get("Motor[1].ClosedLoop"))
    in_position = _flag(snapshot.get("Motor[1].InPos"))
    fault_code = _fault_code(snapshot)
    faulted = fault_code is not None
    error = None
    if command_target is not None and actual is not None and math.isfinite(command_target):
        error = abs(command_target - actual)
    valid = invalid is None
    tracking_success = bool(
        valid
        and not faulted
        and error == 0
        and desired is not None
        and command_target is not None
        and desired == command_target
        and actual == command_target
    )
    return ControllerAxisObservation(
        controller_id=CONTROLLER_ID,
        axis_id=AXIS_ID,
        command_target=command_target,
        desired_position=desired,
        actual_position=actual,
        unit=UNIT,
        closed_loop=closed,
        in_position=in_position,
        faulted=faulted,
        fault_code=fault_code,
        communication_state=communication_state,
        observed_at=observed_at,
        source=SOURCE,
        valid=valid,
        invalid_reason=invalid,
        tracking_error=error,
        tracking_success=tracking_success,
    )


def accept_tracking(observation: ControllerAxisObservation) -> bool:
    return observation.valid and observation.tracking_success and not observation.faulted


def _invalid_reason(snapshot: dict[str, str | None], communication_state: str, fresh: bool) -> str | None:
    if communication_state != "HEALTHY":
        return "DISCONNECTED"
    if not fresh:
        return "STALE"
    _actual, actual_problem = _parse(snapshot.get("Motor[1].ActPos"))
    if actual_problem == "MALFORMED":
        return "MALFORMED"
    _desired, desired_problem = _parse(snapshot.get("Motor[1].DesPos"))
    if desired_problem == "MISSING" or actual_problem == "MISSING":
        return "INCOMPLETE"
    if desired_problem == "MALFORMED":
        return "MALFORMED"
    return None


def _parse(value: str | None) -> tuple[float | None, str | None]:
    if value is None or value == "" or value == "NOT_AVAILABLE":
        return None, "MISSING"
    try:
        number = float(value)
    except ValueError:
        return None, "MALFORMED"
    if not math.isfinite(number):
        return None, "MALFORMED"
    return number, None


def _flag(value: str | None) -> bool | None:
    number, problem = _parse(value)
    if problem is not None or number is None:
        return None
    return number == 1


def _fault_code(snapshot: dict[str, str | None]) -> str | None:
    fatal, fatal_problem = _parse(snapshot.get("Motor[1].FeFatal"))
    amp, amp_problem = _parse(snapshot.get("Motor[1].AmpFault"))
    limit, limit_problem = _parse(snapshot.get("Motor[1].LimitStop"))
    if fatal_problem is None and fatal not in (None, 0):
        return "FE_FATAL"
    if amp_problem is None and amp not in (None, 0):
        return "AMP_FAULT"
    if limit_problem is None and limit not in (None, 0):
        return "LIMIT_STOP"
    return None
