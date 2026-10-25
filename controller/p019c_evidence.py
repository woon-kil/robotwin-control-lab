"""P-019C live evidence. credential 값은 기록하지 않는다.

실행: python3 -m controller.p019c_evidence
"""

from __future__ import annotations

import json
import math
import os
import platform
from datetime import datetime, timezone
from pathlib import Path

from controller.power_pmac_adapter import PowerPmacControllerAdapter
from controller.power_pmac_config import P017_TEST_CONFIG
from controller.power_pmac_motor import (
    ENABLE_COMMAND,
    POLL_INTERVAL_S,
    POLL_TIMEOUT_S,
    SERVO_ACTIVATION,
    SingleMotorSession,
    choose_targets,
    motor_state,
    parse_assignments,
)
from controller.power_pmac_transport import GpasciiExchange, PowerPmacError, SshGpasciiTransport

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "evidence" / "P-019C"
RAW = EVIDENCE / "raw"
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


def main() -> None:
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    RAW.mkdir(parents=True, exist_ok=True)
    started_at = datetime.now(timezone.utc).isoformat()
    live = _live()
    negative = negative_report()
    mapping = _state_mapping()
    _write_json(EVIDENCE / "live_motion.json", live["motion"])
    _write_json(EVIDENCE / "state_mapping.json", mapping)
    _write_json(EVIDENCE / "negative_tests.json", {"cases": negative})
    _write_jsonl(RAW / "live_session.jsonl", live["rows"])
    (EVIDENCE / "environment.md").write_text(_environment(started_at), encoding="utf-8")
    (EVIDENCE / "summary.md").write_text(_summary(started_at, live, negative), encoding="utf-8")
    _reject_secret(EVIDENCE)
    print(json.dumps({"moves": [row.get("result") for row in live["motion"]["moves"]], "negative": len(negative)}, indent=2))


def negative_report() -> list[dict[str, object]]:
    cases: list[dict[str, object]] = []
    for name, target in (
        ("nan", math.nan),
        ("positive infinity", math.inf),
        ("negative infinity", -math.inf),
        ("newline injection", "1\n#2j=100"),
        ("separator injection", "1;something"),
        ("command injection", "#1j=5"),
    ):
        fake = _Catalog()
        result = SingleMotorSession(fake, now=lambda: 0.0, sleep=lambda _delay: None).move_test_motor(target)
        cases.append(_case(name, result.error_code == "INVALID_TARGET" and not _jogged(fake), result))
    fake = _Catalog()
    other = SingleMotorSession(fake, now=lambda: 0.0, sleep=lambda _delay: None).move_test_motor(1, motor_id=2)
    cases.append(_case("unsupported motor", other.error_code == "UNSUPPORTED_MOTOR" and not _jogged(fake), other))
    down = SingleMotorSession(_Raising(PowerPmacError("CONNECTION_UNREACHABLE", "down")), now=lambda: 0.0, sleep=lambda _delay: None).move_test_motor(2)
    cases.append(_case("disconnect", down.error_code == "DISCONNECTED" and down.command_sent is False, down))
    broken = _Catalog(fixed="not a pmac value")
    malformed = SingleMotorSession(broken, now=lambda: 0.0, sleep=lambda _delay: None).move_test_motor(2)
    cases.append(_case("malformed response", malformed.error_code == "PROTOCOL_ERROR" and not _jogged(broken), malformed))
    blocked = _Catalog(closed_loop="0", after_enable="0")
    enable = SingleMotorSession(blocked, now=lambda: 0.0, sleep=lambda _delay: None).move_test_motor(2)
    cases.append(_case("enable failure", enable.error_code == "ENABLE_FAILED" and not _jogged(blocked), enable))
    faulted = _Catalog(overrides={"Motor[1].AmpFault": "1"})
    fault = SingleMotorSession(faulted, now=lambda: 0.0, sleep=lambda _delay: None).move_test_motor(2)
    cases.append(_case("pre-motion fault", fault.error_code == "AMP_FAULT" and not _jogged(faulted), fault))
    waiting = _Catalog(apply_target=False)
    timed = SingleMotorSession(waiting, now=lambda: 0.0, sleep=lambda _delay: None, timeout_s=0.0).move_test_motor(2)
    cases.append(_case("timeout", timed.error_code == "TIMEOUT" and timed.command_sent is True and _jog_count(waiting) == 1, timed))
    return cases


def _live() -> dict[str, object]:
    transport = SshGpasciiTransport(P017_TEST_CONFIG)
    adapter = PowerPmacControllerAdapter(transport)
    rows: list[dict[str, object]] = []
    motion: dict[str, object] = {"connect": False, "enable": None, "moves": [], "error": None}
    try:
        adapter.connect()
        motion["connect"] = adapter.communication_state == "HEALTHY"
        rows.append({"step": "connect", "ok": motion["connect"]})
        before = adapter.read_test_motor_state()
        current = before.normalized.get("pos")
        target_a, target_b = choose_targets(current if isinstance(current, (int, float)) and not isinstance(current, bool) else None)
        enabled = adapter.enable_test_motor()
        motion["enable"] = enabled.to_dict()
        rows.append({"step": "enable", "result": enabled.result, "error_code": enabled.error_code, "commands": list(enabled.commands_sent)})
        if enabled.result != "COMPLETED":
            motion["error"] = {"step": "enable", "code": enabled.error_code}
            return {"motion": motion, "rows": rows}
        for label, target in (("A", target_a), ("B", target_b)):
            pre = adapter.read_test_motor_state()
            result = adapter.move_test_motor(target)
            motion["moves"].append({"label": label, "target": target, "pre": _positions(pre), **_move_view(result)})
            rows.append({"step": "move", "label": label, "target": target, "result": result.result, "response": result.response_text, "commands": list(result.commands_sent)})
    except PowerPmacError as exc:
        motion["error"] = {"code": exc.code, "message": str(exc)}
        rows.append({"step": "error", "code": exc.code})
    return {"motion": motion, "rows": rows}


def _move_view(result: object) -> dict[str, object]:
    payload = result.to_dict() if hasattr(result, "to_dict") else {}
    state = payload.get("state") if isinstance(payload, dict) else None
    normalized = state.get("normalized") if isinstance(state, dict) else {}
    raw = state.get("raw") if isinstance(state, dict) else {}
    assert isinstance(normalized, dict)
    assert isinstance(raw, dict)
    return {
        "result": payload.get("result"),
        "error_code": payload.get("error_code"),
        "command": payload.get("command"),
        "command_sent": payload.get("command_sent"),
        "stages": payload.get("stages"),
        "poll_count": payload.get("poll_count"),
        "elapsed_s": payload.get("elapsed_s"),
        "elapsed_note": payload.get("elapsed_note"),
        "response_text": payload.get("response_text"),
        "pos": raw.get("Motor[1].Pos"),
        "actual_pos": raw.get("Motor[1].ActPos"),
        "desired_pos": raw.get("Motor[1].DesPos"),
        "closed_loop": normalized.get("closed_loop"),
        "in_position": normalized.get("in_position"),
        "following_error_fatal": normalized.get("following_error_fatal"),
        "following_error_warning": normalized.get("following_error_warning"),
        "amp_fault": normalized.get("amp_fault"),
        "limit_stop": normalized.get("limit_stop"),
    }


def _positions(state: object) -> dict[str, str | None]:
    raw = getattr(state, "raw", {})
    if not isinstance(raw, dict):
        return {"pos": None, "actual_pos": None, "desired_pos": None}
    return {
        "pos": raw.get("Motor[1].Pos"),
        "actual_pos": raw.get("Motor[1].ActPos"),
        "desired_pos": raw.get("Motor[1].DesPos"),
    }


def _state_mapping() -> dict[str, object]:
    text = "\n".join(
        [
            "Motor[1].ServoCtrl=1",
            "Motor[1].ClosedLoop=1",
            "Motor[1].Pos=1",
            "Motor[1].ActPos=1",
            "Motor[1].DesPos=1",
            "Motor[1].DesVelZero=1",
            "Motor[1].FeFatal=0",
            "Motor[1].FeWarn=0",
            "Motor[1].AmpFault=0",
            "Motor[1].LimitStop=0",
            "Motor[1].InPos=1",
            "Motor[1].Status[0]=$7800",
            "Motor[1].Status[1]=$0",
        ]
    )
    state = motor_state(parse_assignments(text))
    return {
        "source": "parser check using the P-019B observed token shapes",
        "status_bits_decoded": False,
        "warning_is_fatal": False,
        "state": state.to_dict(),
    }


def _environment(started_at: str) -> str:
    config = P017_TEST_CONFIG
    return "\n".join(
        [
            "# P-019C environment",
            "",
            f"측정: {started_at}",
            "",
            "```text",
            "client: MacBook",
            f"client_os: {platform.platform()}",
            "Simulator: Official Power PMAC Simulator 1.2.1",
            "firmware: 2.8.3.0 stated in the P-019B brief; this runner did not query a firmware variable",
            "target motor: Motor[1]",
            f"path: MacBook -> {config.host}:{config.port} -> {config.simulator_address}:22 -> SSH + gpascii",
            f"poll_timeout_s: {POLL_TIMEOUT_S} PROJECT TEST PARAMETER",
            f"poll_interval_s: {POLL_INTERVAL_S} PROJECT TEST PARAMETER",
            f"credential: runtime environment variable {config.password_env}; value not recorded",
            "```",
            "",
            "P-019B evidence는 수정하지 않는다.",
            "",
        ]
    )


def _summary(started_at: str, live: dict[str, object], negative: list[dict[str, object]]) -> str:
    motion = live["motion"]
    assert isinstance(motion, dict)
    moves = motion.get("moves", [])
    assert isinstance(moves, list)
    lines = [
        "# P-019C summary",
        "",
        f"측정: {started_at}",
        "",
        "이 문서는 P-019C PASS 또는 FAIL을 확정하지 않는다. 최종 판정은 별도 review다. P-020은 시작하지 않는다.",
        "",
        "기준선: `a36f17b7be1f6b964a750a1da0d88acb13daf185`",
        "",
        "Motor[1] seam은 `PowerPmacControllerAdapter` 안에 있다. `execute_command`와 ExecutionGate는 이 seam을 호출하지 않는다.",
        "",
        "ServoCtrl이 0이면 `Motor[1].ServoCtrl=1`을 쓰고 read-back이 0보다 클 때만 `#1J/`를 보낸다. 이미 ClosedLoop가 1이면 `#1J/`를 다시 보내지 않는다.",
        "",
        "FeWarn은 기록한다. FeFatal, AmpFault, LimitStop과 같지 않으며 fatal로 올리지 않는다.",
        "",
        "Status[0]과 Status[1]은 raw token이다. status bit는 decode하지 않는다.",
        "",
        "## Live",
        "",
        f"connect: {motion.get('connect')}",
        "",
    ]
    enable = motion.get("enable")
    if isinstance(enable, dict):
        lines.append(
            "enable: result "
            + str(enable.get("result"))
            + ", activation_write "
            + str(enable.get("activation_write"))
            + ", enable_command_sent "
            + str(enable.get("enable_command_sent"))
        )
    for move in moves:
        if isinstance(move, dict):
            lines.append(
                f"target {move.get('label')} {move.get('target')}: {move.get('result')} "
                f"command {move.get('command')} DesPos {move.get('desired_pos')} "
                f"Pos {move.get('pos')} ActPos {move.get('actual_pos')} "
                f"polls {move.get('poll_count')} elapsed_s {move.get('elapsed_s')}"
            )
    if motion.get("error") is not None:
        lines.append("error: " + json.dumps(motion.get("error")))
    lines.extend(["", "elapsed time은 SSH + portproxy + gpascii + polling이다. controller/servo latency가 아니다.", "", "## Negative", ""])
    for case in negative:
        lines.append(f"- {case.get('name')}: ok {case.get('ok')} code {case.get('error_code')} command_sent {case.get('command_sent')}")
    lines.extend(
        [
            "",
            "## Claim boundary",
            "",
            "- Official Power PMAC Simulator only",
            "- software virtual Motor[1] only",
            "- PMAC motor units only",
            "- not radians/degrees/meters",
            "- ActPos is simulator virtual feedback",
            "- not physical encoder feedback",
            "- no real Power PMAC hardware",
            "- no physical servo/motor",
            "- no physical encoder",
            "- no I/O validation",
            "- no EtherCAT validation",
            "- no robot joint mapping",
            "- no UR10e motion",
            "- no real robot",
            "- no hard-real-time validation",
            "- no industrial commissioning",
            "",
        ]
    )
    return "\n".join(lines)


def _case(name: str, ok: bool, result: object) -> dict[str, object]:
    payload = result.to_dict() if hasattr(result, "to_dict") else {}
    return {
        "name": name,
        "ok": ok,
        "result": payload.get("result"),
        "error_code": payload.get("error_code"),
        "command_sent": payload.get("command_sent"),
        "motion_commands": [line for line in payload.get("commands_sent", []) if isinstance(line, str) and line.startswith("#1j=")],
    }


class _Catalog:
    def __init__(self, overrides: dict[str, str] | None = None, *, fixed: str | None = None, closed_loop: str = "1", after_enable: str = "1", apply_target: bool = True) -> None:
        self.values = {name: "0" for name in _NAMES}
        self.values["Motor[1].ServoCtrl"] = "1"
        self.values["Motor[1].ClosedLoop"] = closed_loop
        self.values["Motor[1].DesVelZero"] = "1"
        self.values["Motor[1].Status[0]"] = "$4000"
        if overrides:
            self.values.update(overrides)
        self.fixed = fixed
        self.after_enable = after_enable
        self.apply_target = apply_target
        self.calls: list[list[str]] = []

    def execute(self, lines: list[str], timeout_s: float | None = None) -> GpasciiExchange:
        del timeout_s
        self.calls.append(list(lines))
        if self.fixed is not None and lines != [ENABLE_COMMAND] and not (len(lines) == 1 and lines[0].startswith("#1j=")):
            return GpasciiExchange(self.fixed, "", 1)
        if lines == [SERVO_ACTIVATION]:
            self.values["Motor[1].ServoCtrl"] = "1"
        elif lines == [ENABLE_COMMAND]:
            self.values["Motor[1].ClosedLoop"] = self.after_enable
        elif len(lines) == 1 and lines[0].startswith("#1j=") and self.apply_target:
            target = lines[0].split("=", 1)[1]
            self.values["Motor[1].Pos"] = target
            self.values["Motor[1].ActPos"] = target
            self.values["Motor[1].DesPos"] = target
        if lines == [ENABLE_COMMAND] or (len(lines) == 1 and lines[0].startswith("#1j=")):
            return GpasciiExchange("STDIN Open for ASCII Input\n\n", "", 1)
        return GpasciiExchange("\n".join(name + "=" + self.values[name] for name in lines), "", 1)


class _Raising:
    def __init__(self, error: PowerPmacError) -> None:
        self.error = error

    def execute(self, lines: list[str], timeout_s: float | None = None) -> GpasciiExchange:
        del lines, timeout_s
        raise self.error


def _jogged(fake: _Catalog) -> bool:
    return _jog_count(fake) > 0


def _jog_count(fake: _Catalog) -> int:
    return sum(1 for call in fake.calls if len(call) == 1 and call[0].startswith("#1j="))


def _write_json(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _write_jsonl(path: Path, rows: list[dict[str, object]]) -> None:
    path.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")


def _reject_secret(directory: Path) -> None:
    secret = os.environ.get(P017_TEST_CONFIG.password_env, "")
    if not secret:
        return
    for path in directory.rglob("*"):
        if path.is_file() and secret in path.read_text(encoding="utf-8"):
            raise SystemExit("credential leaked into evidence")


if __name__ == "__main__":
    main()
