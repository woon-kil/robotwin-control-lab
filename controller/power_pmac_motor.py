"""P-019C Motor[1] integration seam.

이 API는 6축 robot command가 아니다. ExecutionGate와 Validator는 이 모듈을 호출하지 않는다.
위치 단위는 PMAC motor unit이다.

Poll timeout과 interval은 PROJECT TEST PARAMETER다.
elapsed time은 SSH, portproxy, gpascii, polling을 포함한다. servo latency가 아니다.

Status[0]/Status[1] bit는 decode하지 않는다.
FeWarn은 기록만 하고 FeFatal로 올리지 않는다.
ServoCtrl이 0이면 Motor[1].ServoCtrl=1을 쓰고 read-back이 0보다 큰 뒤에만 #1J/를 보낸다.
"""

from __future__ import annotations

import math
import re
import time
from collections.abc import Callable
from dataclasses import dataclass

from controller.power_pmac_transport import GpasciiExchange, PowerPmacError

TEST_MOTOR_ID = 1
ENABLE_COMMAND = "#1J/"
SERVO_ACTIVATION = "Motor[1].ServoCtrl=1"
POLL_TIMEOUT_S = 12.0
POLL_INTERVAL_S = 0.25
_VARIABLES = (
    "Motor[1].ServoCtrl",
    "Motor[1].ClosedLoop",
    "Motor[1].Pos",
    "Motor[1].ActPos",
    "Motor[1].DesPos",
    "Motor[1].DesVelZero",
    "Motor[1].FeFatal",
    "Motor[1].FeWarn",
    "Motor[1].AmpFault",
    "Motor[1].LimitStop",
    "Motor[1].InPos",
    "Motor[1].Status[0]",
    "Motor[1].Status[1]",
)
_ASSIGN_RE = re.compile(
    r"(Motor\[1\]\.(?:Status\[1\]|Status\[0\]|ServoCtrl|ClosedLoop|ActPos|DesPos|DesVelZero|"
    r"FeFatal|FeWarn|AmpFault|LimitStop|InPos|Pos))\s*=\s*(\$?[+-]?[0-9A-Fa-f.]+)"
)
_ERROR_RE = re.compile(r"error\s*#\s*\d+|motor not active", re.IGNORECASE)
_JOG_RE = re.compile(r"#1j=[+-]?\d+(?:\.\d+)?")
_NUMBER_RE = re.compile(r"[+-]?\d+(?:\.\d+)?")
Now = Callable[[], float]
Sleep = Callable[[float], None]


@dataclass(frozen=True)
class MotorState:
    motor_id: int
    raw: dict[str, str | None]
    normalized: dict[str, object]
    complete: bool
    missing: tuple[str, ...]
    fault_code: str | None

    def to_dict(self) -> dict[str, object]:
        return {
            "motor_id": self.motor_id,
            "raw": self.raw,
            "normalized": self.normalized,
            "complete": self.complete,
            "missing": list(self.missing),
            "fault_code": self.fault_code,
            "status_bits_decoded": False,
        }


@dataclass(frozen=True)
class SingleMotorResult:
    result: str
    error_code: str | None
    stages: tuple[str, ...]
    command: str | None
    command_sent: bool
    commands_sent: tuple[str, ...]
    state: MotorState | None
    poll_count: int
    elapsed_s: float
    response_text: str
    activation_write: bool
    enable_command_sent: bool

    def to_dict(self) -> dict[str, object]:
        return {
            "result": self.result,
            "error_code": self.error_code,
            "stages": list(self.stages),
            "command": self.command,
            "command_sent": self.command_sent,
            "commands_sent": list(self.commands_sent),
            "state": None if self.state is None else self.state.to_dict(),
            "poll_count": self.poll_count,
            "elapsed_s": self.elapsed_s,
            "response_text": self.response_text,
            "activation_write": self.activation_write,
            "enable_command_sent": self.enable_command_sent,
            "elapsed_note": "SSH + portproxy + gpascii + polling. Not controller or servo latency.",
        }


class LineExecutor:
    def execute(self, lines: list[str], timeout_s: float | None = None) -> GpasciiExchange:
        raise NotImplementedError


class TransportExecutor:
    def __init__(self, transport: object) -> None:
        self._transport = transport

    def execute(self, lines: list[str], timeout_s: float | None = None) -> GpasciiExchange:
        execute_lines = getattr(self._transport, "execute_lines", None)
        if not callable(execute_lines):
            raise PowerPmacError("DISCONNECTED", "transport has no gpascii execute_lines")
        exchange = execute_lines(lines, timeout_s)
        if not isinstance(exchange, GpasciiExchange):
            raise PowerPmacError("PROTOCOL_ERROR", "transport exchange was not parseable")
        return exchange


class SingleMotorSession:
    def __init__(
        self,
        executor: object,
        *,
        now: Now = time.monotonic,
        sleep: Sleep = time.sleep,
        timeout_s: float = POLL_TIMEOUT_S,
        interval_s: float = POLL_INTERVAL_S,
    ) -> None:
        self._executor = executor
        self._now = now
        self._sleep = sleep
        self._timeout_s = timeout_s
        self._interval_s = interval_s
        self.calls: list[list[str]] = []

    def read_test_motor_state(self) -> MotorState:
        raw: dict[str, str] = {}
        exchange = self._execute(list(_VARIABLES))
        raw.update(parse_assignments(exchange.stdout))
        if any(name not in raw for name in _VARIABLES):
            raw = {}
            for name in _VARIABLES:
                one = self._execute([name])
                raw.update(parse_assignments(one.stdout))
        return motor_state(raw)

    def enable_test_motor(self) -> SingleMotorResult:
        self.calls = []
        started = self._now()
        try:
            state = self.read_test_motor_state()
        except PowerPmacError as exc:
            return self._failed(exc.code if exc.code != "TIMEOUT" else "TIMEOUT", started, "FAULTED" if exc.code == "TIMEOUT" else "REJECTED")
        if not state.complete:
            return self._done("REJECTED", "PROTOCOL_ERROR", started, state=state)
        if state.fault_code is not None:
            return self._done("FAULTED", state.fault_code, started, state=state)
        ready = self._ensure_ready(state, started)
        if isinstance(ready, SingleMotorResult):
            return ready
        return self._done("COMPLETED", None, started, state=ready, stages=("COMPLETED",))

    def move_test_motor(self, target: object, motor_id: int = TEST_MOTOR_ID) -> SingleMotorResult:
        self.calls = []
        started = self._now()
        if motor_id != TEST_MOTOR_ID:
            return self._done("REJECTED", "UNSUPPORTED_MOTOR", started)
        try:
            token = serialize_target(target)
        except PowerPmacError:
            return self._done("REJECTED", "INVALID_TARGET", started)
        command = "#1j=" + token
        try:
            state = self.read_test_motor_state()
        except PowerPmacError as exc:
            return self._failed(exc.code, started, "FAULTED" if exc.code == "TIMEOUT" else "REJECTED")
        if not state.complete:
            return self._done("REJECTED", "PROTOCOL_ERROR", started, state=state)
        if state.fault_code is not None:
            return self._done("FAULTED", state.fault_code, started, state=state)
        ready = self._ensure_ready(state, started)
        if isinstance(ready, SingleMotorResult):
            return ready
        try:
            response = self._execute([command])
        except PowerPmacError as exc:
            result_name = "FAULTED" if exc.code == "TIMEOUT" else "REJECTED"
            code = "TIMEOUT" if exc.code == "TIMEOUT" else ("DISCONNECTED" if exc.code in {"CONNECTION_UNREACHABLE", "AUTHENTICATION_FAILURE", "DISCONNECTED"} else "PROTOCOL_ERROR")
            return self._done(result_name, code, started, command=command, command_sent=True, stages=("ACCEPTED",))
        text = response.stdout + "\n" + response.stderr
        if command_error(text):
            return self._done(
                "REJECTED",
                "COMMAND_ERROR",
                started,
                command=command,
                command_sent=True,
                stages=("ACCEPTED",),
                response_text=text,
            )
        polls = 0
        last = ready
        while True:
            polls += 1
            try:
                last = self.read_test_motor_state()
            except PowerPmacError as exc:
                result_name = "FAULTED" if exc.code == "TIMEOUT" else "REJECTED"
                code = "TIMEOUT" if exc.code == "TIMEOUT" else "DISCONNECTED"
                return self._done(result_name, code, started, state=last if isinstance(last, MotorState) else None, command=command, command_sent=True, stages=("ACCEPTED",), poll_count=polls, response_text=text)
            if not last.complete:
                return self._done("REJECTED", "PROTOCOL_ERROR", started, state=last, command=command, command_sent=True, stages=("ACCEPTED",), poll_count=polls, response_text=text)
            if last.fault_code is not None:
                return self._done("FAULTED", last.fault_code, started, state=last, command=command, command_sent=True, stages=("ACCEPTED",), poll_count=polls, response_text=text)
            if target_reached(last, token):
                return self._done("COMPLETED", None, started, state=last, command=command, command_sent=True, stages=("ACCEPTED", "COMPLETED"), poll_count=polls, response_text=text)
            if self._now() >= started + self._timeout_s:
                return self._done("FAULTED", "TIMEOUT", started, state=last, command=command, command_sent=True, stages=("ACCEPTED",), poll_count=polls, response_text=text)
            self._sleep(self._interval_s)

    def _ensure_ready(self, state: MotorState, started: float) -> MotorState | SingleMotorResult:
        current = state
        activation = False
        enabled = False
        servo = current.normalized.get("servo_ctrl")
        if not isinstance(servo, (int, float)) or isinstance(servo, bool) or servo <= 0:
            try:
                self._execute([SERVO_ACTIVATION])
                activation = True
                current = self.read_test_motor_state()
            except PowerPmacError as exc:
                return self._failed(exc.code, started, "FAULTED" if exc.code == "TIMEOUT" else "REJECTED", activation_write=True)
            servo = current.normalized.get("servo_ctrl")
            if not current.complete or not isinstance(servo, (int, float)) or isinstance(servo, bool) or servo <= 0:
                return self._done("REJECTED", "ACTIVATION_FAILED", started, state=current, activation_write=activation)
        if current.fault_code is not None:
            return self._done("FAULTED", current.fault_code, started, state=current, activation_write=activation)
        if current.normalized.get("closed_loop") is not True:
            try:
                response = self._execute([ENABLE_COMMAND])
                enabled = True
            except PowerPmacError as exc:
                return self._failed(exc.code, started, "FAULTED" if exc.code == "TIMEOUT" else "REJECTED", activation_write=activation, enable_command_sent=True)
            text = response.stdout + "\n" + response.stderr
            if command_error(text):
                return self._done("REJECTED", "ENABLE_FAILED", started, state=current, activation_write=activation, enable_command_sent=True, response_text=text)
            try:
                current = self.read_test_motor_state()
            except PowerPmacError as exc:
                return self._failed(exc.code, started, "FAULTED" if exc.code == "TIMEOUT" else "REJECTED", activation_write=activation, enable_command_sent=True)
            if not current.complete or current.normalized.get("closed_loop") is not True:
                return self._done("REJECTED", "ENABLE_FAILED", started, state=current, activation_write=activation, enable_command_sent=True)
            if current.fault_code is not None:
                return self._done("FAULTED", current.fault_code, started, state=current, activation_write=activation, enable_command_sent=True)
        self._activation = activation
        self._enabled = enabled
        return current

    def _execute(self, lines: list[str]) -> GpasciiExchange:
        guard_motor_lines(lines)
        self.calls.append(list(lines))
        execute = getattr(self._executor, "execute", None)
        if not callable(execute):
            raise PowerPmacError("DISCONNECTED", "executor is unavailable")
        exchange = execute(lines, None)
        if not isinstance(exchange, GpasciiExchange):
            raise PowerPmacError("PROTOCOL_ERROR", "executor exchange was not parseable")
        return exchange

    def _failed(
        self,
        code: str,
        started: float,
        result: str,
        *,
        activation_write: bool = False,
        enable_command_sent: bool = False,
    ) -> SingleMotorResult:
        mapped = code
        if code in {"CONNECTION_UNREACHABLE", "AUTHENTICATION_FAILURE"}:
            mapped = "DISCONNECTED"
            result = "REJECTED"
        return self._done(result, mapped, started, activation_write=activation_write, enable_command_sent=enable_command_sent)

    def _done(
        self,
        result: str,
        error_code: str | None,
        started: float,
        *,
        state: MotorState | None = None,
        command: str | None = None,
        command_sent: bool = False,
        stages: tuple[str, ...] = (),
        poll_count: int = 0,
        response_text: str = "",
        activation_write: bool = False,
        enable_command_sent: bool = False,
    ) -> SingleMotorResult:
        if not activation_write:
            activation_write = any(call == [SERVO_ACTIVATION] for call in self.calls)
        if not enable_command_sent:
            enable_command_sent = any(call == [ENABLE_COMMAND] for call in self.calls)
        warning = False
        if state is not None:
            warning = state.normalized.get("following_error_warning") is True
        flat: list[str] = []
        for call in self.calls:
            flat.extend(call)
        return SingleMotorResult(
            result=result,
            error_code=error_code,
            stages=stages,
            command=command,
            command_sent=command_sent,
            commands_sent=tuple(flat),
            state=state,
            poll_count=poll_count,
            elapsed_s=self._now() - started,
            response_text=response_text,
            activation_write=activation_write,
            enable_command_sent=enable_command_sent,
        )


def guard_motor_lines(lines: list[str]) -> None:
    if not lines:
        raise PowerPmacError("PROTOCOL_ERROR", "Motor[1] command list is empty")
    if lines == [ENABLE_COMMAND] or lines == [SERVO_ACTIVATION]:
        return
    if len(lines) == 1 and _JOG_RE.fullmatch(lines[0]) is not None:
        return
    if all(line in _VARIABLES for line in lines):
        return
    raise PowerPmacError("PROTOCOL_ERROR", "command is outside the Motor[1] seam")


def serialize_target(value: object) -> str:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PowerPmacError("INVALID_TARGET", "target must be a finite number")
    if isinstance(value, float) and not math.isfinite(value):
        raise PowerPmacError("INVALID_TARGET", "target must be a finite number")
    if isinstance(value, float) and value.is_integer():
        text = str(int(value))
    elif isinstance(value, float):
        text = format(value, "f").rstrip("0").rstrip(".")
    else:
        text = str(value)
    if len(text) > 32 or _NUMBER_RE.fullmatch(text) is None:
        raise PowerPmacError("INVALID_TARGET", "target cannot be serialized as a PMAC number")
    return text


def parse_assignments(stdout: str) -> dict[str, str]:
    found: dict[str, str] = {}
    for match in _ASSIGN_RE.finditer(stdout):
        found[match.group(1)] = match.group(2)
    return found


def parse_number(raw: str | None) -> float | None:
    if raw is None:
        return None
    text = raw.strip()
    try:
        if text.startswith("$"):
            return float(int(text[1:], 16))
        return float(text)
    except ValueError:
        return None


def command_error(text: str) -> bool:
    return _ERROR_RE.search(text) is not None


def motor_state(found: dict[str, str]) -> MotorState:
    raw = {name: found.get(name) for name in _VARIABLES}
    missing = tuple(name for name, value in raw.items() if value is None)
    numbers = {
        "servo_ctrl": parse_number(raw["Motor[1].ServoCtrl"]),
        "closed_loop_value": parse_number(raw["Motor[1].ClosedLoop"]),
        "pos": parse_number(raw["Motor[1].Pos"]),
        "actual_pos": parse_number(raw["Motor[1].ActPos"]),
        "desired_pos": parse_number(raw["Motor[1].DesPos"]),
        "desired_velocity_zero_value": parse_number(raw["Motor[1].DesVelZero"]),
        "in_position_value": parse_number(raw["Motor[1].InPos"]),
        "following_error_fatal_value": parse_number(raw["Motor[1].FeFatal"]),
        "following_error_warning_value": parse_number(raw["Motor[1].FeWarn"]),
        "amp_fault_value": parse_number(raw["Motor[1].AmpFault"]),
        "limit_stop_value": parse_number(raw["Motor[1].LimitStop"]),
    }
    numeric_missing = any(value is None for key, value in numbers.items() if not key.endswith("_value") or key in numbers)
    # Status tokens are raw. Numeric motor variables must parse.
    parsed_ok = all(
        numbers[key] is not None
        for key in (
            "servo_ctrl",
            "closed_loop_value",
            "pos",
            "actual_pos",
            "desired_pos",
            "desired_velocity_zero_value",
            "in_position_value",
            "following_error_fatal_value",
            "following_error_warning_value",
            "amp_fault_value",
            "limit_stop_value",
        )
    )
    fatal = numbers["following_error_fatal_value"]
    amp = numbers["amp_fault_value"]
    limit = numbers["limit_stop_value"]
    fault_code = None
    if fatal not in (None, 0):
        fault_code = "FE_FATAL"
    elif amp not in (None, 0):
        fault_code = "AMP_FAULT"
    elif limit not in (None, 0):
        fault_code = "LIMIT_STOP"
    normalized: dict[str, object] = {
        "servo_ctrl": numbers["servo_ctrl"],
        "closed_loop": None if numbers["closed_loop_value"] is None else numbers["closed_loop_value"] == 1,
        "pos": numbers["pos"],
        "actual_pos": numbers["actual_pos"],
        "desired_pos": numbers["desired_pos"],
        "desired_velocity_zero": None if numbers["desired_velocity_zero_value"] is None else numbers["desired_velocity_zero_value"] == 1,
        "in_position": None if numbers["in_position_value"] is None else numbers["in_position_value"] == 1,
        "following_error_fatal": None if fatal is None else fatal != 0,
        "following_error_warning": None if numbers["following_error_warning_value"] is None else numbers["following_error_warning_value"] != 0,
        "amp_fault": None if amp is None else amp != 0,
        "limit_stop": None if limit is None else limit != 0,
        "faulted": None if None in (fatal, amp, limit) else fault_code is not None,
        "status_0": raw["Motor[1].Status[0]"],
        "status_1": raw["Motor[1].Status[1]"],
        "status_bits_decoded": False,
        "warning_is_fatal": False,
    }
    del numeric_missing
    complete = len(missing) == 0 and parsed_ok and raw["Motor[1].Status[0]"] is not None and raw["Motor[1].Status[1]"] is not None
    return MotorState(TEST_MOTOR_ID, raw, normalized, complete, missing, fault_code)


def target_reached(state: MotorState, token: str) -> bool:
    desired = state.normalized.get("desired_pos")
    return (
        state.complete
        and state.normalized.get("closed_loop") is True
        and state.fault_code is None
        and desired == parse_number(token)
        and state.normalized.get("pos") is not None
        and state.normalized.get("actual_pos") is not None
    )


def assess_test_motor_readiness(state: MotorState) -> str:
    """Read-back으로만 readiness를 계산한다. jog나 fault clear를 보내지 않는다."""
    if not state.complete:
        return "PROTOCOL_ERROR"
    if state.fault_code is not None:
        return "FAULT_PRESENT"
    servo = state.normalized.get("servo_ctrl")
    if isinstance(servo, bool) or not isinstance(servo, (int, float)) or servo <= 0:
        return "NOT_READY"
    if state.normalized.get("closed_loop") is not True:
        return "NOT_READY"
    return "RECOVERED"


def readiness_result(token: str, state: MotorState | None = None) -> SingleMotorResult:
    recovered = token == "RECOVERED"
    return SingleMotorResult(
        result=token,
        error_code=None if recovered else token,
        stages=("RECOVERY_VERIFIED",) if recovered else ("RECOVERY_BLOCKED",),
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


def disconnected_result() -> SingleMotorResult:
    return SingleMotorResult(
        result="REJECTED",
        error_code="DISCONNECTED",
        stages=("DISCONNECTED",),
        command=None,
        command_sent=False,
        commands_sent=(),
        state=None,
        poll_count=0,
        elapsed_s=0.0,
        response_text="",
        activation_write=False,
        enable_command_sent=False,
    )


def choose_targets(current: float | None) -> tuple[int, int]:
    picked = [value for value in (0, 2, 3) if value != current]
    if len(picked) < 2:
        return (4, 5)
    return (picked[0], picked[1])
