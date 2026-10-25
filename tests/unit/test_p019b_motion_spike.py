"""P-019B gate. Simulator 없이 command 허용 범위와 STOP 조건만 본다."""

import pytest

from controller.p019_motion_spike import (
    ENABLE_COMMAND,
    MOTION_COMMAND,
    STATE_VARIABLES,
    command_error,
    gate_decision,
    guard_lines,
    run_spike,
)
from controller.power_pmac_transport import READ_NAMES, PowerPmacError


def test_p018_read_allowlist_stays_on_harmless_variables():
    assert READ_NAMES == frozenset({"P100", "Sys.ServoCount"})


def test_spike_rejects_config_changes_and_other_motors():
    for line in ("Motor[1].ServoCtrl=1", "Motor[2].Pos", "#1j+", "#1j-", "Sys.SimConfigOk=1", "j/"):
        with pytest.raises(PowerPmacError):
            guard_lines([line])
    guard_lines([ENABLE_COMMAND])
    guard_lines([MOTION_COMMAND])
    guard_lines(list(STATE_VARIABLES))


def test_closed_loop_failure_does_not_send_the_motion_command():
    executor = _executor(closed_loop="0")
    report = run_spike(executor, sleep=_no_sleep)
    gate = report["gate"]
    motion = report["motion_attempt_2"]
    assert isinstance(gate, dict)
    assert isinstance(motion, dict)
    assert gate["authorized"] is False
    assert "ClosedLoop is not 1" in gate["reasons"]
    assert motion["executed"] is False
    assert [call for call in executor.calls if call == [MOTION_COMMAND]] == []
    assert executor.calls.count([ENABLE_COMMAND]) == 1


def test_gate_authorizes_only_the_same_motion_command():
    executor = _executor(closed_loop="1")
    report = run_spike(executor, sleep=_no_sleep, observation_delay_s=1.0)
    gate = report["gate"]
    motion = report["motion_attempt_2"]
    assert isinstance(gate, dict)
    assert isinstance(motion, dict)
    assert gate["authorized"] is True
    assert gate["reasons"] == []
    assert motion["executed"] is True
    assert motion["command"] == "#1j=1"
    assert executor.calls.count([ENABLE_COMMAND]) == 1
    assert executor.calls.count([MOTION_COMMAND]) == 1


def test_position_jump_fault_and_command_error_block_motion():
    before = _values(closed_loop="0")
    jumped = _values(closed_loop="1")
    jumped["Motor[1].Pos"] = "5"
    decision = gate_decision(before, jumped, "", True)
    assert decision["authorized"] is False
    assert "abnormal Pos jump" in decision["reasons"]

    faulted = _values(closed_loop="1")
    faulted["Motor[1].FeFatal"] = "1"
    assert gate_decision(before, faulted, "", True)["authorized"] is False

    amp = _values(closed_loop="1")
    amp["Motor[1].AmpFault"] = "1"
    assert gate_decision(before, amp, "", True)["authorized"] is False

    limited = _values(closed_loop="1")
    limited["Motor[1].LimitStop"] = "1"
    assert gate_decision(before, limited, "", True)["authorized"] is False

    assert command_error("error #45: MOTOR NOT ACTIVE") is True
    assert gate_decision(before, _values(closed_loop="1"), "error #45: MOTOR NOT ACTIVE", True)["authorized"] is False


class _Script:
    def __init__(self, closed_loop: str) -> None:
        self.closed_loop = closed_loop
        self.calls: list[list[str]] = []

    def __call__(self, lines: list[str]) -> dict[str, object]:
        guard_lines(lines)
        self.calls.append(list(lines))
        if lines == [ENABLE_COMMAND]:
            return _response("")
        if lines == [MOTION_COMMAND]:
            return _response("")
        values = _values(self.closed_loop)
        stdout = "\n".join(f"{name}={values[name]}" for name in lines)
        return _response(stdout)


def _executor(closed_loop: str) -> _Script:
    return _Script(closed_loop)


def _response(stdout: str) -> dict[str, object]:
    return {
        "ok": True,
        "code": None,
        "stdout": "STDIN Open for ASCII Input\n" + stdout + "\n",
        "stderr": "",
        "returncode": 1,
        "elapsed_ms": 1.0,
        "command_error": command_error(stdout),
        "lines": [],
    }


def _values(closed_loop: str) -> dict[str, str]:
    values = {name: "0" for name in STATE_VARIABLES}
    values["Motor[1].ServoCtrl"] = "1"
    values["Motor[1].ClosedLoop"] = closed_loop
    values["Motor[1].Status[0]"] = "$4000"
    return values


def _no_sleep(delay_s: float) -> None:
    del delay_s
