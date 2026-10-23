"""VirtualPmacController.

PMAC-compatible controller semantics.
PROJECT VIRTUAL CONTROLLER.
NOT VERIFIED AGAINST REAL POWER PMAC.

controller_state는 REQ-011 어휘만 쓴다.
이 baseline의 enable은 READY이고, 동작 중은 RUNNING이며, fault는 ERROR다.
관절 명령 타입은 Canonical API의 MOVE_JOINT다.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from controller.plant import slew_step
from core.kinematics.fk import forward_kinematics
from core.kinematics.limits import limit_violations
from core.model.types import RobotModel

CONTROLLER_STATES = (
    "DISCONNECTED",
    "CONNECTED",
    "SERVO_OFF",
    "SERVO_ON",
    "HOMING",
    "READY",
    "RUNNING",
    "HOLD",
    "ERROR",
    "ESTOP",
)
COMMAND_TYPE_MOVE_JOINT = "MOVE_JOINT"
EXECUTION_STATES = ("ACCEPTED", "EXECUTING", "COMPLETED", "REJECTED", "FAULTED")
FAULT_CODES = ("FAULT_INJECTED", "COMMAND_INVALID", "JOINT_LIMIT", "INTERNAL_ERROR", "TEST_FAULT")
COMPLETION_TOLERANCE_RAD = 1.0e-9
CLAIM = "PMAC-compatible controller semantics"
CLAIM_BOUNDARY = (
    "PROJECT VIRTUAL CONTROLLER",
    "NOT VERIFIED AGAINST REAL POWER PMAC",
    "DETERMINISTIC PROJECT TEST MODEL",
    "NOT ACTUAL UR10e SERVO DYNAMICS",
    "NOT ACTUAL PMAC SERVO DYNAMICS",
)

# P-006 virtual controller table. docs/ROBOT_INTERFACE.md를 고치지 않는다.
# SERVO_ON, HOMING, HOLD, ESTOP은 이 baseline의 체류 상태가 아니다.
_TRANSITIONS = {
    ("DISCONNECTED", "connect"): "CONNECTED",
    ("CONNECTED", "disconnect"): "DISCONNECTED",
    ("CONNECTED", "servo_on"): "READY",
    ("CONNECTED", "fault"): "ERROR",
    ("READY", "servo_off"): "SERVO_OFF",
    ("READY", "disconnect"): "DISCONNECTED",
    ("READY", "execute"): "RUNNING",
    ("READY", "fault"): "ERROR",
    ("RUNNING", "complete"): "READY",
    ("RUNNING", "servo_off"): "SERVO_OFF",
    ("RUNNING", "disconnect"): "DISCONNECTED",
    ("RUNNING", "fault"): "ERROR",
    ("SERVO_OFF", "servo_on"): "READY",
    ("SERVO_OFF", "disconnect"): "DISCONNECTED",
    ("SERVO_OFF", "fault"): "ERROR",
    ("ERROR", "reset"): "CONNECTED",
    ("ERROR", "disconnect"): "DISCONNECTED",
}


class ControllerError(Exception):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class JointCommand:
    command_id: str
    command_type: str
    joint_targets: tuple[float, ...]
    sequence: int | None = None
    timestamp_s: float = 0.0
    robot_id: str = "ur10e"


@dataclass(frozen=True)
class CommandResult:
    command_id: str
    result: str
    error_code: str | None


@dataclass(frozen=True)
class RobotState:
    timestamp_s: float
    robot_id: str
    controller_state: str
    joint_position: tuple[float, ...]
    joint_velocity: tuple[float, ...]
    target_joint_position: tuple[float, ...]
    joint_error: tuple[float, ...]
    tcp_pose: dict[str, object]
    command_id: str | None
    active_command_id: str | None
    execution_state: str | None
    error_code: str | None
    fault: str | None


class IRobotController(Protocol):
    def connect(self) -> None: ...

    def disconnect(self) -> None: ...

    def servo_on(self) -> None: ...

    def servo_off(self) -> None: ...

    def execute_command(self, command: JointCommand) -> CommandResult: ...

    def step(self, dt_s: float) -> RobotState: ...

    def get_state(self) -> RobotState: ...

    def reset(self) -> None: ...


class VirtualPmacController:
    """시뮬레이터 API를 호출하지 않는 joint command 실행기다."""

    def __init__(self, model: RobotModel) -> None:
        self._model = model
        self._order = model.actuated_joint_order
        self._max_velocity = tuple(_velocity_limit(model, name) for name in self._order)
        zeros = tuple(0.0 for _ in self._order)
        self._state = "DISCONNECTED"
        self._time_s = 0.0
        self._actual = zeros
        self._target = zeros
        self._velocity = zeros
        self._command_id: str | None = None
        self._active_command_id: str | None = None
        self._execution: str | None = None
        self._error_code: str | None = None
        self._fault: str | None = None
        self._sequence = 0
        self._seen_ids: set[str] = set()
        self._traces: dict[str, list[str]] = {}

    @property
    def max_velocity_rad_s(self) -> tuple[float, ...]:
        return self._max_velocity

    def connect(self) -> None:
        self._transition("connect")

    def disconnect(self) -> None:
        if self._active_command_id is not None:
            self._finish_active("REJECTED", "DISCONNECTED")
        self._transition("disconnect")
        self._hold_position()
        self._fault = None
        self._error_code = None

    def servo_on(self) -> None:
        self._transition("servo_on")

    def servo_off(self) -> None:
        if self._active_command_id is not None:
            self._finish_active("REJECTED", "SERVO_OFF")
        self._transition("servo_off")
        self._hold_position()

    def enable(self) -> None:
        self.servo_on()

    def disable(self) -> None:
        self.servo_off()

    def reset(self) -> None:
        actual = self._actual
        self._transition("reset")
        self._actual = actual
        self._target = actual
        self._velocity = tuple(0.0 for _ in self._order)
        self._fault = None
        self._error_code = None
        self._active_command_id = None

    def reset_fault(self) -> None:
        self.reset()

    def inject_fault(self, code: str) -> None:
        if code not in FAULT_CODES:
            raise ControllerError("UNKNOWN_FAULT", "fault code is not in the project set")
        if self._active_command_id is not None:
            self._finish_active("FAULTED", code)
        self._transition("fault")
        self._hold_position()
        self._fault = code
        self._error_code = code

    def execute_command(self, command: JointCommand) -> CommandResult:
        return self.submit(command)

    def submit(self, command: JointCommand) -> CommandResult:
        if self._state == "RUNNING":
            return self._reject(command.command_id, "BUSY")
        if self._state != "READY":
            return self._reject(command.command_id, "ILLEGAL_STATE")
        if command.command_type != COMMAND_TYPE_MOVE_JOINT:
            return self._reject(command.command_id, "UNSUPPORTED_COMMAND")
        if command.command_id in self._seen_ids:
            return self._reject(command.command_id, "DUPLICATE_COMMAND_ID")
        parsed = _parse_targets(command.joint_targets, len(self._order))
        if parsed[0] is None:
            return self._reject(command.command_id, parsed[1])
        targets = parsed[0]
        assert targets is not None
        violated = limit_violations(self._model, targets)
        if violated:
            return self._reject(command.command_id, "JOINT_LIMIT")
        self._seen_ids.add(command.command_id)
        self._sequence += 1
        self._transition("execute")
        self._command_id = command.command_id
        self._active_command_id = command.command_id
        self._target = targets
        self._execution = "ACCEPTED"
        self._error_code = None
        self._traces[command.command_id] = ["ACCEPTED"]
        return CommandResult(command.command_id, "ACCEPTED", None)

    def step(self, dt_s: float) -> RobotState:
        if not isinstance(dt_s, (int, float)) or dt_s != dt_s or dt_s in (float("inf"), float("-inf")) or dt_s <= 0.0:
            raise ControllerError("ILLEGAL_DT", "dt must be finite and positive")
        if self._state == "DISCONNECTED":
            raise ControllerError("ILLEGAL_STATE", "step while DISCONNECTED")
        if self._state == "RUNNING" and self._active_command_id is not None:
            self._mark("EXECUTING")
            previous = self._actual
            self._actual = slew_step(previous, self._target, self._max_velocity, float(dt_s))
            self._velocity = tuple((after - before) / float(dt_s) for before, after in zip(previous, self._actual, strict=True))
            self._time_s += float(dt_s)
            if _max_abs_error(self._target, self._actual) <= COMPLETION_TOLERANCE_RAD:
                self._mark("COMPLETED")
                self._transition("complete")
                self._active_command_id = None
                self._velocity = tuple(0.0 for _ in self._order)
        else:
            self._time_s += float(dt_s)
            self._velocity = tuple(0.0 for _ in self._order)
        return self.get_state()

    def get_state(self) -> RobotState:
        pose = forward_kinematics(self._model, self._actual)
        error = tuple(target - actual for target, actual in zip(self._target, self._actual, strict=True))
        return RobotState(
            timestamp_s=self._time_s,
            robot_id=self._model.robot_id,
            controller_state=self._state,
            joint_position=self._actual,
            joint_velocity=self._velocity,
            target_joint_position=self._target,
            joint_error=error,
            tcp_pose={
                "frame": pose.frame,
                "reference_frame": pose.reference_frame,
                "position_m": {"x": pose.x, "y": pose.y, "z": pose.z},
                "quaternion_xyzw": {"x": pose.qx, "y": pose.qy, "z": pose.qz, "w": pose.qw},
            },
            command_id=self._command_id,
            active_command_id=self._active_command_id,
            execution_state=self._execution,
            error_code=self._error_code,
            fault=self._fault,
        )

    def trace(self, command_id: str) -> tuple[str, ...]:
        return tuple(self._traces.get(command_id, ()))

    def _transition(self, action: str) -> None:
        destination = _TRANSITIONS.get((self._state, action))
        if destination is None:
            raise ControllerError("ILLEGAL_TRANSITION", self._state + " " + action)
        self._state = destination

    def _reject(self, command_id: str, code: str) -> CommandResult:
        if command_id not in self._traces:
            self._traces[command_id] = ["REJECTED"]
        return CommandResult(command_id, "REJECTED", code)

    def _mark(self, execution: str) -> None:
        self._execution = execution
        if self._active_command_id is None:
            return
        trace = self._traces[self._active_command_id]
        if not trace or trace[-1] != execution:
            trace.append(execution)

    def _finish_active(self, execution: str, code: str) -> None:
        self._execution = execution
        self._error_code = code
        if self._active_command_id is not None:
            trace = self._traces[self._active_command_id]
            if not trace or trace[-1] != execution:
                trace.append(execution)
        self._active_command_id = None

    def _hold_position(self) -> None:
        self._target = self._actual
        self._velocity = tuple(0.0 for _ in self._order)


def _velocity_limit(model: RobotModel, name: str) -> float:
    limits = model.joint_by_name(name).limits
    if limits is None or limits.velocity <= 0.0:
        raise ControllerError("MISSING_VELOCITY_LIMIT", name + " has no sourced positive velocity limit")
    return limits.velocity


def _parse_targets(values: object, count: int) -> tuple[tuple[float, ...] | None, str]:
    if not isinstance(values, (list, tuple)) or len(values) != count:
        return None, "JOINT_COUNT"
    parsed: list[float] = []
    for value in values:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return None, "NON_FINITE"
        number = float(value)
        if number != number or number in (float("inf"), float("-inf")):
            return None, "NON_FINITE"
        parsed.append(number)
    return tuple(parsed), ""


def _max_abs_error(target: tuple[float, ...], actual: tuple[float, ...]) -> float:
    return max(abs(goal - current) for goal, current in zip(target, actual, strict=True))
