"""P-018 adapter. Simulator 없이 scripted transport와 SSH 결과 분류만 본다."""

import subprocess
from pathlib import Path

import pytest

from controller.power_pmac_adapter import PowerPmacControllerAdapter
from controller.power_pmac_config import P017_TEST_CONFIG
from controller.power_pmac_transport import (
    MARKER,
    PowerPmacError,
    ScriptedTransport,
    SshGpasciiTransport,
    interpret,
)
from controller.virtual_pmac import JointCommand

ROOT = Path(__file__).resolve().parents[2]
SECRET = "secret-value"


def test_connect_health_read_and_write_read_back():
    transport = ScriptedTransport()
    transport.queue_read("10")
    transport.queue_read("11")
    transport.queue_read("17020")
    transport.queue_write_read("18018")
    adapter = PowerPmacControllerAdapter(transport)

    adapter.connect()
    assert adapter.communication_state == "HEALTHY"
    assert adapter.get_state().controller_state == "CONNECTED"
    assert adapter.get_state().controller_state not in {"READY", "RUNNING", "SERVO_ON", "SERVO_OFF", "HOLD"}

    health = adapter.probe_health()
    assert health.name == "Sys.ServoCount"
    assert health.value == 11
    assert adapter.read("P100").value == 17020
    assert adapter.write_read_back("P100", 18018).value == 18018
    assert [call[0] for call in transport.calls] == ["read", "read", "read", "write_read"]


def test_connect_failure_stays_disconnected():
    transport = ScriptedTransport()
    transport.queue_read(PowerPmacError("CONNECTION_UNREACHABLE", "SSH connection failed"))
    adapter = PowerPmacControllerAdapter(transport)

    with pytest.raises(PowerPmacError) as caught:
        adapter.connect()

    assert caught.value.code == "CONNECTION_UNREACHABLE"
    assert adapter.communication_state == "DISCONNECTED"
    assert adapter.get_state().controller_state == "DISCONNECTED"


def test_disconnect_rejects_later_operations_and_closes_transport():
    transport = _connected()
    adapter = PowerPmacControllerAdapter(transport)
    adapter.connect()
    before = len(transport.calls)

    adapter.disconnect()

    assert transport.closed is True
    assert adapter.communication_state == "DISCONNECTED"
    refused = adapter.execute_command(JointCommand("after", "MOVE_JOINT", (0, 0, 0, 0, 0, 0)))
    assert refused.result == "REJECTED"
    assert refused.error_code == "DISCONNECTED"
    with pytest.raises(PowerPmacError) as caught:
        adapter.read("P100")
    assert caught.value.code == "DISCONNECTED"
    assert transport.calls[before:] == [("close",)]


def test_operation_before_connect_is_rejected():
    adapter = PowerPmacControllerAdapter(ScriptedTransport())
    refused = adapter.execute_command(JointCommand("early", "MOVE_JOINT", (0, 0, 0, 0, 0, 0)))
    assert refused.result == "REJECTED"
    assert refused.error_code == "DISCONNECTED"
    with pytest.raises(PowerPmacError) as caught:
        adapter.probe_health()
    assert caught.value.code == "DISCONNECTED"


def test_timeout_malformed_response_and_mismatch_are_distinct():
    timed = SshGpasciiTransport(P017_TEST_CONFIG, runner=_timeout_runner)
    adapter = PowerPmacControllerAdapter(timed)
    with pytest.raises(PowerPmacError) as timed_out:
        adapter.connect()
    assert timed_out.value.code == "TIMEOUT"
    assert timed_out.type is PowerPmacError
    assert adapter.communication_state == "DISCONNECTED"

    with pytest.raises(PowerPmacError) as malformed:
        interpret("STDIN Open for ASCII Input\n// *** exit\n", "", 1, "Sys.ServoCount", None)
    assert malformed.value.code == "PROTOCOL_ERROR"

    transport = _connected()
    transport.queue_write_read("11")
    mismatch = PowerPmacControllerAdapter(transport)
    mismatch.connect()
    with pytest.raises(PowerPmacError) as missed:
        mismatch.write_read_back("P100", 18018)
    assert missed.value.code == "READBACK_MISMATCH"
    assert mismatch.communication_state == "FAILED"
    with pytest.raises(PowerPmacError) as blocked:
        mismatch.read("P100")
    assert blocked.value.code == "COMMUNICATION_FAILED"


def test_ssh_mapping_keeps_subprocess_exceptions_inside_transport(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setenv("PMAC_SSH_PASSWORD", SECRET)
    seen: dict[str, object] = {}

    def runner(command: list[str], env: dict[str, str], timeout_s: float) -> subprocess.CompletedProcess[str]:
        seen["command"] = command
        seen["env"] = env
        return subprocess.CompletedProcess(command, 255, stdout="", stderr=f"Permission denied {SECRET}")

    transport = SshGpasciiTransport(P017_TEST_CONFIG, runner=runner)
    with pytest.raises(PowerPmacError) as caught:
        transport.read_variable("Sys.ServoCount")
    assert caught.value.code == "AUTHENTICATION_FAILURE"
    assert SECRET not in str(caught.value)
    command = seen["command"]
    assert isinstance(command, list)
    assert SECRET not in " ".join(command)
    assert "PreferredAuthentications=password" in command
    assert caught.type is PowerPmacError

    def refused(command: list[str], env: dict[str, str], timeout_s: float) -> subprocess.CompletedProcess[str]:
        del env, timeout_s
        return subprocess.CompletedProcess(command, 255, stdout="", stderr="Connection refused")

    with pytest.raises(PowerPmacError) as unreachable:
        SshGpasciiTransport(P017_TEST_CONFIG, runner=refused).read_variable("P100")
    assert unreachable.value.code == "CONNECTION_UNREACHABLE"


def test_nonzero_gpascii_exit_still_parses_and_hides_the_password(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.delenv("PMAC_SSH_PASSWORD", raising=False)
    seen: dict[str, list[str]] = {}

    def runner(command: list[str], env: dict[str, str], timeout_s: float) -> subprocess.CompletedProcess[str]:
        del env, timeout_s
        seen["command"] = command
        remote = command[-1]
        if "P100=" in remote:
            stdout = f"STDIN Open for ASCII Input\n{MARKER}\nSTDIN Open for ASCII Input\nP100=18018\n// *** exit\n"
        else:
            stdout = "STDIN Open for ASCII Input\nSys.ServoCount=10\n// *** exit\n"
        return subprocess.CompletedProcess(command, 1, stdout=stdout, stderr="")

    transport = SshGpasciiTransport(P017_TEST_CONFIG, runner=runner)
    assert transport.read_variable("Sys.ServoCount") == "10"
    assert transport.write_and_read("P100", 18018) == "18018"
    assert "BatchMode=yes" in seen["command"]
    remote = seen["command"][-1]
    assert "jog" not in remote
    assert "#1" not in remote
    assert MARKER in remote


def test_motion_methods_do_not_call_transport_or_claim_servo_state():
    transport = _connected()
    adapter = PowerPmacControllerAdapter(transport)
    adapter.connect()
    before = list(transport.calls)

    refused = adapter.execute_command(JointCommand("motion", "MOVE_JOINT", (0.1, 0, 0, 0, 0, 0)))
    assert refused.result == "REJECTED"
    assert refused.error_code == "MOTION_NOT_IN_SCOPE"
    with pytest.raises(PowerPmacError) as servo:
        adapter.servo_on()
    assert servo.value.code == "MOTION_NOT_IN_SCOPE"
    with pytest.raises(PowerPmacError) as reset:
        adapter.reset()
    assert reset.value.code == "MOTION_NOT_IN_SCOPE"
    assert transport.calls == before
    assert adapter.get_state().tcp_pose["measured"] is False
    assert adapter.communication_state == "HEALTHY"


def test_sources_do_not_store_a_password_or_motor_command():
    for relative in (
        "controller/power_pmac_adapter.py",
        "controller/power_pmac_transport.py",
        "controller/power_pmac_config.py",
        "controller/virtual_pmac.py",
        "validator/gate.py",
    ):
        text = (ROOT / relative).read_text(encoding="utf-8")
        assert "PMAC_SSH_PASSWORD=" not in text
        assert "#1j" not in text
    assert "class ExecutionGate" in (ROOT / "validator" / "gate.py").read_text(encoding="utf-8")
    assert "VirtualPmacController" in (ROOT / "validator" / "gate.py").read_text(encoding="utf-8")


def _connected() -> ScriptedTransport:
    transport = ScriptedTransport()
    transport.queue_read("10")
    return transport


def _timeout_runner(command: list[str], env: dict[str, str], timeout_s: float) -> subprocess.CompletedProcess[str]:
    del env
    raise subprocess.TimeoutExpired(command, timeout_s)
