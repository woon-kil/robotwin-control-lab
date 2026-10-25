"""P-018 evidence. credential 값은 쓰지 않는다.

실행: python3 -m controller.p018_evidence
"""

from __future__ import annotations

import json
import os
import platform
import subprocess
from datetime import datetime, timezone
from pathlib import Path

from controller.power_pmac_adapter import PowerPmacControllerAdapter
from controller.power_pmac_config import P017_TEST_CONFIG
from controller.power_pmac_transport import (
    PowerPmacError,
    ScriptedTransport,
    SshGpasciiTransport,
    interpret,
)
from controller.virtual_pmac import JointCommand

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "evidence" / "P-018"
RAW = EVIDENCE / "raw"
CLAIM = (
    "RoboTwin PowerPmacControllerAdapter was integrated with "
    "Official Power PMAC Simulator through SSH + gpascii."
)
NOT_CLAIMED = (
    "Real Power PMAC hardware validated",
    "PMAC servo control validated",
    "Real motor control validated",
    "EtherCAT validated",
    "Industrial hard real-time validated",
    "Robot motion validated",
    "Production-ready PMAC integration",
)


def main() -> None:
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    RAW.mkdir(parents=True, exist_ok=True)
    started_at = datetime.now(timezone.utc).isoformat()
    mapping = _failure_mapping()
    live = _live()
    _write_json(EVIDENCE / "failure_mapping.json", {"started_at": started_at, "cases": mapping})
    _write_json(RAW / "failure_mapping.jsonl", mapping, lines=True)
    _write_json(EVIDENCE / "live_connection.json", live["connection"])
    _write_json(EVIDENCE / "write_read_results.json", live["write_read"])
    _write_json(EVIDENCE / "adapter_results.json", live["adapter"])
    _write_json(RAW / "live.jsonl", live["steps"], lines=True)
    (EVIDENCE / "environment.md").write_text(_environment(started_at), encoding="utf-8")
    _reject_secret(EVIDENCE)
    print(json.dumps({"failure_cases": len(mapping), "live_error": live.get("error")}, indent=2))


def _failure_mapping() -> list[dict[str, object]]:
    cases: list[dict[str, object]] = []

    def runner_auth(command: list[str], env: dict[str, str], timeout_s: float) -> subprocess.CompletedProcess[str]:
        del env, timeout_s
        return subprocess.CompletedProcess(command, 255, stdout="", stderr="Permission denied (publickey,password).")

    def runner_refused(command: list[str], env: dict[str, str], timeout_s: float) -> subprocess.CompletedProcess[str]:
        del env, timeout_s
        return subprocess.CompletedProcess(command, 255, stdout="", stderr="Connection refused")

    def runner_timeout(command: list[str], env: dict[str, str], timeout_s: float) -> subprocess.CompletedProcess[str]:
        del env
        raise subprocess.TimeoutExpired(command, timeout_s)

    cases.append(_transport_case("authentication failure", runner_auth, "AUTHENTICATION_FAILURE"))
    cases.append(_transport_case("connection refused", runner_refused, "CONNECTION_UNREACHABLE"))
    cases.append(_transport_case("timeout", runner_timeout, "TIMEOUT"))

    try:
        interpret("STDIN Open for ASCII Input\n// *** exit\n", "", 1, "P100", None)
        cases.append(_case("malformed response", False, None, "expected PROTOCOL_ERROR"))
    except PowerPmacError as exc:
        cases.append(_case("malformed response", exc.code == "PROTOCOL_ERROR", exc.code, None))

    disconnected = PowerPmacControllerAdapter(ScriptedTransport())
    refused = disconnected.execute_command(JointCommand("disconnected", "MOVE_JOINT", (0, 0, 0, 0, 0, 0)))
    cases.append(
        _case(
            "disconnected operation",
            refused.result == "REJECTED" and refused.error_code == "DISCONNECTED",
            refused.error_code,
            None,
        )
    )

    transport = ScriptedTransport()
    transport.queue_read("10")
    transport.queue_write_read("1")
    adapter = PowerPmacControllerAdapter(transport)
    adapter.connect()
    try:
        adapter.write_read_back("P100", 18018)
        cases.append(_case("read-back mismatch", False, None, "expected READBACK_MISMATCH"))
    except PowerPmacError as exc:
        cases.append(_case("read-back mismatch", exc.code == "READBACK_MISMATCH", exc.code, None))
    return cases


def _transport_case(name: str, runner, expected: str) -> dict[str, object]:
    adapter = PowerPmacControllerAdapter(SshGpasciiTransport(P017_TEST_CONFIG, runner=runner))
    try:
        adapter.connect()
    except PowerPmacError as exc:
        leaked = exc.__cause__ is not None or isinstance(exc, subprocess.SubprocessError)
        return _case(name, exc.code == expected and not leaked, exc.code, None)
    except subprocess.SubprocessError as exc:
        return _case(name, False, type(exc).__name__, "subprocess exception left the transport")
    return _case(name, False, None, "connect returned without an error")


def _live() -> dict[str, object]:
    config = P017_TEST_CONFIG
    adapter = PowerPmacControllerAdapter(SshGpasciiTransport(config))
    steps: list[dict[str, object]] = []
    connection: dict[str, object] = {"connect": False, "health_probe": False}
    write_read: dict[str, object] = {
        "variable": "P100",
        "read": False,
        "write": False,
        "read_back": False,
        "expected_equals_actual": False,
    }
    adapter_result: dict[str, object] = {
        "disconnect": False,
        "post_disconnect_reject": False,
        "controller_state_during_session": None,
        "communication_state_during_session": None,
        "motion_sent": False,
    }
    error: dict[str, str] | None = None
    try:
        adapter.connect()
        state = adapter.get_state()
        connection["connect"] = adapter.communication_state == "HEALTHY"
        adapter_result["controller_state_during_session"] = state.controller_state
        adapter_result["communication_state_during_session"] = adapter.communication_state
        steps.append({"step": "connect", "ok": connection["connect"], "communication_state": adapter.communication_state})

        health = adapter.probe_health()
        connection["health_probe"] = health.name == "Sys.ServoCount"
        connection["health_variable"] = health.name
        connection["health_value"] = health.value
        connection["health_claim"] = "parsed Sys.ServoCount; not a verified servo or motor state"
        steps.append({"step": "health", "ok": True, "value": health.value})

        before = adapter.read("P100")
        test_value = 18019 if before.value == 18018 else 18018
        write_read["read"] = True
        write_read["read_value"] = before.value
        steps.append({"step": "read", "ok": True, "value": before.value})

        written = adapter.write_read_back("P100", test_value)
        write_read["write"] = True
        write_read["test_value"] = test_value
        write_read["read_back"] = True
        write_read["read_back_value"] = written.value
        write_read["expected_equals_actual"] = written.value == test_value
        steps.append({"step": "write_read_back", "ok": written.value == test_value, "value": written.value})

        adapter.disconnect()
        adapter_result["disconnect"] = adapter.communication_state == "DISCONNECTED"
        refused = adapter.execute_command(JointCommand("p018-after-disconnect", "MOVE_JOINT", (0, 0, 0, 0, 0, 0)))
        adapter_result["post_disconnect_reject"] = refused.result == "REJECTED" and refused.error_code == "DISCONNECTED"
        adapter_result["post_disconnect_result"] = refused.result
        adapter_result["post_disconnect_error_code"] = refused.error_code
        try:
            adapter.read("P100")
            adapter_result["post_disconnect_read_rejected"] = False
        except PowerPmacError as exc:
            adapter_result["post_disconnect_read_rejected"] = exc.code == "DISCONNECTED"
        steps.append(
            {
                "step": "disconnect",
                "ok": adapter_result["disconnect"] and adapter_result["post_disconnect_reject"],
                "error_code": refused.error_code,
            }
        )
    except PowerPmacError as exc:
        error = {"code": exc.code, "message": str(exc)}
        steps.append({"step": "error", "ok": False, "code": exc.code})
    connection["path"] = (
        f"MacBook -> {config.host}:{config.port} -> {config.simulator_address}:22 -> gpascii"
    )
    connection["interface"] = "SSH + gpascii"
    return {
        "connection": connection,
        "write_read": write_read,
        "adapter": adapter_result,
        "steps": steps,
        "error": error,
    }


def _environment(started_at: str) -> str:
    config = P017_TEST_CONFIG
    return "\n".join(
        [
            "# P-018 environment",
            "",
            f"측정: {started_at}",
            "",
            "```text",
            "client: MacBook",
            f"client_os: {platform.platform()}",
            "server/forwarder: Windows notebook",
            f"Windows address: {config.host}",
            f"forwarding port: {config.port}",
            "target: Official Power PMAC Simulator 1.2.1",
            f"target PMAC address: {config.simulator_address}",
            "interface: SSH + gpascii",
            "variables: Sys.ServoCount, P100",
            f"credential: runtime environment variable {config.password_env}; value not recorded",
            "```",
            "",
            "P-018 controller state represents communication/integration state only. "
            "It does not claim verified PMAC servo or motor state.",
            "",
            "Sys.ServoCount 값은 통신 health를 확인한 파싱 결과다. servo enable이나 motor state가 아니다.",
            "",
        ]
    )


def _case(name: str, ok: bool, code: str | None, detail: str | None) -> dict[str, object]:
    row: dict[str, object] = {
        "name": name,
        "ok": ok,
        "code": code,
        "exception": "PowerPmacError" if code not in (None, "DISCONNECTED") or name != "disconnected operation" else "CommandResult",
        "subprocess_leaked": False,
    }
    if name == "disconnected operation":
        row["exception"] = "CommandResult"
    if detail is not None:
        row["detail"] = detail
        row["subprocess_leaked"] = "subprocess" in detail
    return row


def _write_json(path: Path, payload: object, lines: bool = False) -> None:
    if lines:
        rows = payload if isinstance(payload, list) else []
        path.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")
        return
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _reject_secret(directory: Path) -> None:
    secret = os.environ.get(P017_TEST_CONFIG.password_env, "")
    if not secret:
        return
    for path in directory.rglob("*"):
        if path.is_file() and secret in path.read_text(encoding="utf-8"):
            raise SystemExit("credential leaked into evidence")


if __name__ == "__main__":
    main()
