"""P-019C Motor[1] seam. Simulator 없이 reject와 completion을 본다."""

import math

import pytest

from controller.power_pmac_adapter import PowerPmacControllerAdapter
from controller.power_pmac_motor import (
    ENABLE_COMMAND,
    SERVO_ACTIVATION,
    SingleMotorSession,
    guard_motor_lines,
    motor_state,
    parse_assignments,
    serialize_target,
)
from controller.power_pmac_transport import (
    READ_NAMES,
    GpasciiExchange,
    PowerPmacError,
    ScriptedTransport,
    format_gpascii_remote,
)
from controller.virtual_pmac import JointCommand

_NAMES = (
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


def test_negative_catalog_records_motion_not_sent():
    from controller.p019c_evidence import negative_report

    cases = negative_report()
    assert cases
    assert all(case["ok"] is True for case in cases)
    assert all(case["motion_commands"] == [] or case["name"] == "timeout" for case in cases)
    timeout = next(case for case in cases if case["name"] == "timeout")
    assert timeout["command_sent"] is True
    assert timeout["motion_commands"] == ["#1j=2"]


def test_p018_allowlist_and_unsafe_tokens_stay_closed():
    assert READ_NAMES == frozenset({"P100", "Sys.ServoCount"})
    with pytest.raises(PowerPmacError):
        format_gpascii_remote(["1\n#2j=100"])
    with pytest.raises(PowerPmacError):
        guard_motor_lines(["Motor[2].Pos"])
    with pytest.raises(PowerPmacError):
        guard_motor_lines(["#1j+"])
    with pytest.raises(PowerPmacError):
        guard_motor_lines(["Sys.SimConfigOk=1"])


def test_state_parse_keeps_status_raw_and_does_not_promote_warning():
    text = "\n".join(
        [
            "Motor[1].ServoCtrl=1",
            "Motor[1].ClosedLoop=1",
            "Motor[1].Pos=1",
            "Motor[1].ActPos=1",
            "Motor[1].DesPos=1",
            "Motor[1].DesVelZero=1",
            "Motor[1].FeFatal=0",
            "Motor[1].FeWarn=1",
            "Motor[1].AmpFault=0",
            "Motor[1].LimitStop=0",
            "Motor[1].InPos=1",
            "Motor[1].Status[0]=$7800",
            "Motor[1].Status[1]=$0",
        ]
    )
    state = motor_state(parse_assignments(text))
    assert state.raw["Motor[1].Status[0]"] == "$7800"
    assert state.normalized["status_bits_decoded"] is False
    assert state.normalized["closed_loop"] is True
    assert state.normalized["in_position"] is True
    assert state.normalized["following_error_warning"] is True
    assert state.normalized["warning_is_fatal"] is False
    assert state.fault_code is None
    assert state.normalized["faulted"] is False


def test_activation_and_closed_loop_are_read_back_before_motion():
    fake = _Fake({"Motor[1].ServoCtrl": "0", "Motor[1].ClosedLoop": "0"})
    session = SingleMotorSession(fake, now=_clock(), sleep=_no_sleep, timeout_s=1.0)
    result = session.move_test_motor(2)
    assert result.result == "COMPLETED"
    assert result.activation_write is True
    assert result.enable_command_sent is True
    assert result.command == "#1j=2"
    assert result.stages == ("ACCEPTED", "COMPLETED")
    assert [SERVO_ACTIVATION, ENABLE_COMMAND, "#1j=2"] == [call[0] for call in fake.calls if len(call) == 1]


def test_invalid_targets_and_other_motors_send_nothing():
    for target in (math.nan, math.inf, -math.inf, "1\n#2j=100", "1;something", "#1j=5", None, True):
        fake = _Fake({})
        result = SingleMotorSession(fake, now=_clock(), sleep=_no_sleep).move_test_motor(target)
        assert result.result == "REJECTED"
        assert result.error_code == "INVALID_TARGET"
        assert result.command_sent is False
        assert fake.calls == []
    fake = _Fake({})
    refused = SingleMotorSession(fake, now=_clock(), sleep=_no_sleep).move_test_motor(1, motor_id=2)
    assert refused.error_code == "UNSUPPORTED_MOTOR"
    assert fake.calls == []
    with pytest.raises(PowerPmacError):
        serialize_target("2")


def test_fail_closed_paths_do_not_send_motion():
    malformed = _Fake({}, stdout="not a pmac value")
    missed = SingleMotorSession(malformed, now=_clock(), sleep=_no_sleep).move_test_motor(2)
    assert missed.error_code == "PROTOCOL_ERROR"
    assert not _jogged(malformed)

    enable = _Fake({"Motor[1].ClosedLoop": "0"}, enable_closed_loop="0")
    blocked = SingleMotorSession(enable, now=_clock(), sleep=_no_sleep).move_test_motor(2)
    assert blocked.error_code == "ENABLE_FAILED"
    assert not _jogged(enable)

    faulted = _Fake({"Motor[1].FeFatal": "1"})
    fault = SingleMotorSession(faulted, now=_clock(), sleep=_no_sleep).move_test_motor(2)
    assert fault.result == "FAULTED"
    assert fault.error_code == "FE_FATAL"
    assert not _jogged(faulted)
    assert ENABLE_COMMAND not in [call[0] for call in faulted.calls if len(call) == 1]

    disconnected = _Raising(PowerPmacError("CONNECTION_UNREACHABLE", "down"))
    down = SingleMotorSession(disconnected, now=_clock(), sleep=_no_sleep).move_test_motor(2)
    assert down.error_code == "DISCONNECTED"
    assert down.command_sent is False


def test_timeout_is_not_completion_and_warning_does_not_block():
    waiting = _Fake({}, apply_target=False)
    clock = _clock()
    timed = SingleMotorSession(waiting, now=clock, sleep=_no_sleep, timeout_s=0.0).move_test_motor(2)
    assert timed.result == "FAULTED"
    assert timed.error_code == "TIMEOUT"
    assert timed.command_sent is True
    assert timed.stages == ("ACCEPTED",)
    assert _jog_count(waiting) == 1

    warned = _Fake({"Motor[1].FeWarn": "1"})
    done = SingleMotorSession(warned, now=_clock(), sleep=_no_sleep, timeout_s=1.0).move_test_motor(3)
    assert done.result == "COMPLETED"
    assert done.state is not None
    assert done.state.normalized["following_error_warning"] is True
    assert done.state.fault_code is None
    assert done.state.normalized["desired_pos"] == 3


def test_polling_waits_until_desired_position_matches():
    delayed = _Fake({}, delay_polls=1)
    sleeps: list[float] = []
    result = SingleMotorSession(delayed, now=_clock(), sleep=sleeps.append, timeout_s=5.0, interval_s=0.25).move_test_motor(2)
    assert result.result == "COMPLETED"
    assert result.poll_count == 2
    assert sleeps == [0.25]
    assert result.state is not None
    assert result.state.normalized["pos"] == 2
    assert result.state.normalized["actual_pos"] == 2


def test_adapter_robot_command_does_not_enter_the_motor_seam():
    transport = ScriptedTransport()
    transport.queue_read("10")
    fake = _Fake({})
    adapter = PowerPmacControllerAdapter(transport, motor_executor=fake)
    refused = adapter.move_test_motor(2)
    assert refused.error_code == "DISCONNECTED"
    assert fake.calls == []
    adapter.connect()
    robot = adapter.execute_command(JointCommand("robot", "MOVE_JOINT", (0, 0, 0, 0, 0, 0)))
    assert robot.result == "REJECTED"
    assert robot.error_code == "MOTION_NOT_IN_SCOPE"
    assert not _jogged(fake)
    moved = adapter.move_test_motor(2)
    assert moved.result == "COMPLETED"
    assert moved.command == "#1j=2"


class _Fake:
    def __init__(
        self,
        overrides: dict[str, str],
        *,
        stdout: str | None = None,
        enable_closed_loop: str = "1",
        apply_target: bool = True,
        delay_polls: int = 0,
    ) -> None:
        self.values = {name: "0" for name in _NAMES}
        self.values["Motor[1].ServoCtrl"] = "1"
        self.values["Motor[1].ClosedLoop"] = "1"
        self.values["Motor[1].DesVelZero"] = "1"
        self.values["Motor[1].Status[0]"] = "$4000"
        self.values.update(overrides)
        self.calls: list[list[str]] = []
        self.fixed_stdout = stdout
        self.enable_closed_loop = enable_closed_loop
        self.apply_target = apply_target
        self.delay_polls = delay_polls
        self._jogged = False
        self._reads_after_jog = 0

    def execute(self, lines: list[str], timeout_s: float | None = None) -> GpasciiExchange:
        del timeout_s
        self.calls.append(list(lines))
        if self.fixed_stdout is not None and lines != [ENABLE_COMMAND] and not _is_jog(lines):
            return _exchange(self.fixed_stdout)
        if lines == [SERVO_ACTIVATION]:
            self.values["Motor[1].ServoCtrl"] = "1"
            return _exchange("")
        if lines == [ENABLE_COMMAND]:
            self.values["Motor[1].ClosedLoop"] = self.enable_closed_loop
            return _exchange("")
        if _is_jog(lines):
            self._jogged = True
            self._reads_after_jog = 0
            if self.apply_target and self.delay_polls == 0:
                _apply_target(self.values, lines[0])
            return _exchange("")
        if self._jogged and self.apply_target and self.delay_polls > 0:
            self._reads_after_jog += 1
            if self._reads_after_jog > self.delay_polls:
                jog = next(call[0] for call in self.calls if _is_jog(call))
                _apply_target(self.values, jog)
        return _exchange("\n".join(name + "=" + self.values[name] for name in lines))


class _Raising:
    def __init__(self, error: PowerPmacError) -> None:
        self.error = error
        self.calls: list[list[str]] = []

    def execute(self, lines: list[str], timeout_s: float | None = None) -> GpasciiExchange:
        del lines, timeout_s
        raise self.error


def _apply_target(values: dict[str, str], command: str) -> None:
    target = command.split("=", 1)[1]
    values["Motor[1].Pos"] = target
    values["Motor[1].ActPos"] = target
    values["Motor[1].DesPos"] = target


def _exchange(stdout: str) -> GpasciiExchange:
    return GpasciiExchange("STDIN Open for ASCII Input\n" + stdout + "\n", "", 1)


def _clock():
    holder = {"now": 0.0}

    def now() -> float:
        return holder["now"]

    return now


def _no_sleep(delay_s: float) -> None:
    del delay_s


def _is_jog(lines: list[str]) -> bool:
    return len(lines) == 1 and lines[0].startswith("#1j=")


def _jogged(fake: _Fake) -> bool:
    return _jog_count(fake) > 0


def _jog_count(fake: _Fake) -> int:
    return sum(1 for call in fake.calls if _is_jog(call))
