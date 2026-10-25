"""P-019B Motor[1] closed-loop spike.

P-018 allowlist와 controller contract는 바꾸지 않는다.
보내는 online command는 `#1J/` 한 번과, gate를 통과한 경우에만 `#1j=1` 한 번이다.
password는 PMAC_SSH_PASSWORD 환경변수로만 받는다.
"""

from __future__ import annotations

import json
import os
import re
import time
from collections.abc import Callable
from datetime import datetime, timezone
from pathlib import Path

from controller.power_pmac_config import P017_TEST_CONFIG
from controller.power_pmac_transport import PowerPmacError, SshGpasciiTransport

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "evidence" / "P-019"
RAW = EVIDENCE / "raw"
ENABLE_COMMAND = "#1J/"
MOTION_COMMAND = "#1j=1"
OBSERVATION_DELAY_S = 1.0
STATE_VARIABLES = (
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
_ALLOWED = frozenset(STATE_VARIABLES) | {ENABLE_COMMAND, MOTION_COMMAND}
_LINE_RE = re.compile(r"[#\[\]0-9A-Za-z.=/]+")
_ASSIGN_RE = re.compile(
    r"(Motor\[1\]\.(?:Status\[1\]|Status\[0\]|ServoCtrl|ClosedLoop|ActPos|DesPos|DesVelZero|"
    r"FeFatal|FeWarn|AmpFault|LimitStop|InPos|Pos))\s*=\s*(\$?[+-]?[0-9A-Fa-f.]+)"
)
_ERROR_RE = re.compile(r"error\s*#\s*\d+|motor not active", re.IGNORECASE)
Execute = Callable[[list[str]], dict[str, object]]
Sleep = Callable[[float], None]


def guard_lines(lines: list[str]) -> None:
    if not lines:
        raise PowerPmacError("PROTOCOL_ERROR", "P-019B command list is empty")
    for line in lines:
        if _LINE_RE.fullmatch(line) is None or line not in _ALLOWED:
            raise PowerPmacError("PROTOCOL_ERROR", "command is outside the P-019B spike")
    if ENABLE_COMMAND in lines and lines != [ENABLE_COMMAND]:
        raise PowerPmacError("PROTOCOL_ERROR", "#1J/ must be sent alone")
    if MOTION_COMMAND in lines and lines != [MOTION_COMMAND]:
        raise PowerPmacError("PROTOCOL_ERROR", "#1j=1 must be sent alone")


def remote_command(lines: list[str]) -> str:
    guard_lines(lines)
    quoted = " ".join("'" + line + "'" for line in lines)
    return f"printf '%s\\n' {quoted} | gpascii"


def command_error(text: str) -> bool:
    return _ERROR_RE.search(text) is not None


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


def gate_decision(before: dict[str, str], after: dict[str, str], enable_text: str, communication_ok: bool) -> dict[str, object]:
    reasons: list[str] = []
    if not communication_ok:
        reasons.append("communication failure")
    if command_error(enable_text):
        reasons.append("gpascii command error")
    servo = parse_number(after.get("Motor[1].ServoCtrl"))
    if servo is None or servo <= 0:
        reasons.append("ServoCtrl is not > 0")
    if parse_number(after.get("Motor[1].ClosedLoop")) != 1:
        reasons.append("ClosedLoop is not 1")
    for name in ("FeFatal", "AmpFault", "LimitStop"):
        if parse_number(after.get("Motor[1]." + name)) != 0:
            reasons.append(name + " is not 0")
    for name in ("Pos", "ActPos", "DesPos"):
        key = "Motor[1]." + name
        left = parse_number(before.get(key))
        right = parse_number(after.get(key))
        if left is None or right is None or right != left:
            reasons.append("abnormal " + name + " jump")
    return {"authorized": len(reasons) == 0, "reasons": reasons}


def run_spike(execute: Execute, sleep: Sleep = time.sleep, observation_delay_s: float = OBSERVATION_DELAY_S) -> dict[str, object]:
    pre = _capture(execute, STATE_VARIABLES)
    report: dict[str, object] = {
        "pre_state": pre,
        "closed_loop_enable": None,
        "post_enable_state": None,
        "gate": None,
        "motion_attempt_2": {"executed": False, "command_sent": False},
    }
    if not pre["complete"]:
        report["gate"] = {
            "authorized": False,
            "reasons": ["pre-state capture incomplete"],
            "enable_sent": False,
        }
        return report
    enable = execute([ENABLE_COMMAND])
    enable_record = _call_record(enable)
    report["closed_loop_enable"] = enable_record
    post = _capture(execute, STATE_VARIABLES)
    report["post_enable_state"] = {
        **post,
        "comparison": _comparison(pre["values"], post["values"]),
    }
    communication_ok = bool(enable.get("ok")) and bool(post["complete"]) and not bool(enable.get("code"))
    decision = gate_decision(
        pre["values"],
        post["values"],
        str(enable.get("stdout", "")) + "\n" + str(enable.get("stderr", "")),
        communication_ok,
    )
    decision["enable_sent"] = True
    report["gate"] = decision
    if not decision["authorized"]:
        return report
    before_motion = _capture(execute, STATE_VARIABLES)
    motion = execute([MOTION_COMMAND])
    immediate = _capture(execute, STATE_VARIABLES)
    sleep(observation_delay_s)
    observed = _capture(execute, STATE_VARIABLES)
    report["motion_attempt_2"] = {
        "executed": True,
        "command_sent": True,
        "command": MOTION_COMMAND,
        "response": _call_record(motion),
        "before": before_motion,
        "immediate": immediate,
        "observation_delay_s": observation_delay_s,
        "observation_note": (
            "Delay between the immediate read and the later read. "
            "Not a servo settling measurement and not hard real-time."
        ),
        "observation": observed,
    }
    return report


def live_execute(lines: list[str]) -> dict[str, object]:
    remote = remote_command(lines)
    transport = SshGpasciiTransport(P017_TEST_CONFIG)
    started = time.perf_counter()
    try:
        stdout, stderr, returncode = transport._run(remote, None)
    except PowerPmacError as exc:
        return {
            "ok": False,
            "code": exc.code,
            "stdout": "",
            "stderr": "",
            "returncode": None,
            "elapsed_ms": (time.perf_counter() - started) * 1000.0,
            "lines": list(lines),
            "command_error": False,
        }
    text = stdout + "\n" + stderr
    return {
        "ok": True,
        "code": None,
        "stdout": stdout,
        "stderr": stderr,
        "returncode": returncode,
        "elapsed_ms": (time.perf_counter() - started) * 1000.0,
        "lines": list(lines),
        "command_error": command_error(text),
    }


def write_evidence(report: dict[str, object], started_at: str) -> None:
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    RAW.mkdir(parents=True, exist_ok=True)
    pre = report["pre_state"]
    post = report["post_enable_state"]
    enable = report["closed_loop_enable"]
    gate = report["gate"]
    motion = report["motion_attempt_2"]
    assert isinstance(pre, dict)
    assert isinstance(gate, dict)
    assert isinstance(motion, dict)
    _write_json(EVIDENCE / "pre_state.json", {"started_at": started_at, "phase": "PRE_STATE", **_public_capture(pre)})
    _write_json(
        EVIDENCE / "closed_loop_enable.json",
        {
            "started_at": started_at,
            "phase": "CLOSED_LOOP_ENABLE",
            "command": ENABLE_COMMAND,
            "sent": enable is not None,
            "result": enable,
        },
    )
    _write_json(
        EVIDENCE / "post_enable_state.json",
        {
            "started_at": started_at,
            "phase": "POST_ENABLE_STATE",
            **(_public_capture(post) if isinstance(post, dict) else {"complete": False}),
        },
    )
    _write_json(
        EVIDENCE / "motion_attempt_2.json",
        {
            "started_at": started_at,
            "phase": "MOTION_ATTEMPT",
            "gate_authorized": gate.get("authorized"),
            "gate_reasons": gate.get("reasons"),
            **_public_motion(motion),
        },
    )
    _write_jsonl(RAW / "session.jsonl", _session_rows(report))
    (EVIDENCE / "summary.md").write_text(_summary(started_at, report), encoding="utf-8")
    _reject_secret(EVIDENCE)


def main() -> None:
    started_at = datetime.now(timezone.utc).isoformat()
    report = run_spike(live_execute)
    write_evidence(report, started_at)
    gate = report["gate"]
    motion = report["motion_attempt_2"]
    assert isinstance(gate, dict)
    assert isinstance(motion, dict)
    print(json.dumps({"authorized": gate.get("authorized"), "reasons": gate.get("reasons"), "motion_executed": motion.get("executed")}, indent=2))


def _capture(execute: Execute, names: tuple[str, ...] | list[str]) -> dict[str, object]:
    wanted = list(names)
    calls: list[dict[str, object]] = []
    first = execute(wanted)
    calls.append(first)
    if first.get("code"):
        return {"complete": False, "communication_ok": False, "values": {}, "missing": wanted, "calls": calls}
    values = parse_assignments(str(first.get("stdout", "")))
    if any(name not in values for name in wanted):
        values = {}
        for name in wanted:
            row = execute([name])
            calls.append(row)
            if row.get("code"):
                return {
                    "complete": False,
                    "communication_ok": False,
                    "values": values,
                    "missing": [item for item in wanted if item not in values],
                    "calls": calls,
                }
            values.update(parse_assignments(str(row.get("stdout", ""))))
    missing = [name for name in wanted if name not in values]
    return {
        "complete": len(missing) == 0,
        "communication_ok": len(missing) == 0,
        "values": values,
        "missing": missing,
        "calls": calls,
    }


def _comparison(before: dict[str, str], after: dict[str, str]) -> dict[str, dict[str, object]]:
    compared: dict[str, dict[str, object]] = {}
    for name in STATE_VARIABLES:
        compared[name] = {"before": before.get(name), "after": after.get(name)}
    return compared


def _call_record(result: dict[str, object]) -> dict[str, object]:
    return {
        "ok": result.get("ok"),
        "code": result.get("code"),
        "returncode": result.get("returncode"),
        "elapsed_ms": result.get("elapsed_ms"),
        "command_error": result.get("command_error"),
        "stdout": result.get("stdout", ""),
        "stderr": result.get("stderr", ""),
        "lines": result.get("lines", []),
    }


def _public_capture(capture: dict[str, object]) -> dict[str, object]:
    values = capture.get("values", {})
    raw_values = values if isinstance(values, dict) else {}
    return {
        "complete": capture.get("complete"),
        "communication_ok": capture.get("communication_ok"),
        "missing": capture.get("missing", []),
        "values": {name: {"raw": raw_values.get(name), "number": parse_number(raw_values.get(name))} for name in STATE_VARIABLES},
        "comparison": capture.get("comparison"),
    }


def _public_motion(motion: dict[str, object]) -> dict[str, object]:
    if not motion.get("executed"):
        return {"executed": False, "command_sent": False, "command": None, "response": None}
    return {
        "executed": True,
        "command_sent": True,
        "command": motion.get("command"),
        "response": motion.get("response"),
        "observation_delay_s": motion.get("observation_delay_s"),
        "observation_note": motion.get("observation_note"),
        "before": _public_capture(motion["before"]) if isinstance(motion.get("before"), dict) else None,
        "immediate": _public_capture(motion["immediate"]) if isinstance(motion.get("immediate"), dict) else None,
        "observation": _public_capture(motion["observation"]) if isinstance(motion.get("observation"), dict) else None,
    }


def _session_rows(report: dict[str, object]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []

    def take(phase: str, capture: object) -> None:
        if not isinstance(capture, dict):
            return
        calls = capture.get("calls")
        if not isinstance(calls, list):
            return
        for call in calls:
            if isinstance(call, dict):
                rows.append({"phase": phase, **_call_record(call)})

    take("PRE_STATE", report.get("pre_state"))
    enable = report.get("closed_loop_enable")
    if isinstance(enable, dict):
        rows.append({"phase": "CLOSED_LOOP_ENABLE", **enable})
    post = report.get("post_enable_state")
    take("POST_ENABLE_STATE", post)
    motion = report.get("motion_attempt_2")
    if isinstance(motion, dict) and motion.get("executed"):
        take("PRE_MOTION_STATE", motion.get("before"))
        response = motion.get("response")
        if isinstance(response, dict):
            rows.append({"phase": "MOTION_ATTEMPT", **response})
        take("POST_MOTION_IMMEDIATE", motion.get("immediate"))
        take("POST_MOTION_OBSERVATION", motion.get("observation"))
    return rows


def _raw_of(capture: object, key: str) -> str:
    if not isinstance(capture, dict):
        return ""
    values = capture.get("values")
    if not isinstance(values, dict):
        return ""
    raw = values.get(key)
    return "" if raw is None else str(raw)


def _summary(started_at: str, report: dict[str, object]) -> str:
    gate = report["gate"]
    motion = report["motion_attempt_2"]
    pre = report["pre_state"]
    post = report["post_enable_state"]
    assert isinstance(gate, dict)
    assert isinstance(motion, dict)
    assert isinstance(pre, dict)
    pre_values = pre.get("values", {})
    post_values = post.get("values", {}) if isinstance(post, dict) else {}
    assert isinstance(pre_values, dict)
    assert isinstance(post_values, dict)
    config = P017_TEST_CONFIG
    lines = [
        "# P-019B summary",
        "",
        f"측정: {started_at}",
        "",
        "이 문서는 P-019B PASS 또는 FAIL을 확정하지 않는다. 최종 판정은 별도 review다. P-020은 시작하지 않는다.",
        "",
        "기준선: `3515fd13eb6ce39391187ffd81aa9c3cac070e6c`",
        "",
        "## Environment",
        "",
        "```text",
        "Simulator: Official Power PMAC Simulator 1.2.1",
        "firmware: 2.8.3.0 stated in the P-019B brief; this runner did not query a firmware variable",
        "target motor: Motor[1]",
        f"path: MacBook -> {config.host}:{config.port} -> {config.simulator_address}:22 -> SSH + gpascii",
        "credential: runtime environment variable PMAC_SSH_PASSWORD; value not recorded",
        "```",
        "",
        "`#1J/`는 Power PMAC on-line command `j/`다. 2011 On-Line Command Specification의 기능명은 "
        "`Jog stop, or close loop`다. 연속 jog인 `j+`/`j-`는 보내지 않았다.",
        "",
        "위치 값은 PMAC motor unit이다. radian도 meter도 아니다. ActPos는 simulator virtual motor의 응답이며 physical encoder가 아니다.",
        "",
        "## Gate",
        "",
        f"motion attempt authorized: {'YES' if gate.get('authorized') else 'NO'}",
        "",
        "reasons:",
        "",
    ]
    reasons = gate.get("reasons", [])
    if isinstance(reasons, list) and reasons:
        lines.extend(f"- {reason}" for reason in reasons)
    else:
        lines.append("- none")
    lines.extend(["", "## Values", "", "| name | before | after enable |", "|---|---|---|"])
    for name in STATE_VARIABLES:
        lines.append(f"| {name} | {pre_values.get(name)} | {post_values.get(name)} |")
    lines.extend(["", "## Motion Attempt #2", ""])
    if motion.get("executed"):
        lines.append("executed: YES")
        lines.append(f"command: `{MOTION_COMMAND}`")
        lines.append("")
        lines.append("| name | before command | immediate | observation |")
        lines.append("|---|---|---|---|")
        before = motion.get("before")
        immediate = motion.get("immediate")
        observed = motion.get("observation")
        for name in ("Pos", "ActPos", "DesPos", "ClosedLoop", "InPos", "DesVelZero", "FeFatal", "FeWarn", "AmpFault", "LimitStop"):
            key = "Motor[1]." + name
            lines.append(
                "| "
                + name
                + " | "
                + _raw_of(before, key)
                + " | "
                + _raw_of(immediate, key)
                + " | "
                + _raw_of(observed, key)
                + " |"
            )
        lines.append("")
        lines.append(f"observation_delay_s: {motion.get('observation_delay_s')}")
        lines.append(str(motion.get("observation_note")))
    else:
        lines.append("executed: NO")
        lines.append("command: not sent")
    lines.extend(
        [
            "",
            "## Claim boundary",
            "",
            "- Official Power PMAC Simulator only",
            "- software virtual motor only",
            "- no real Power PMAC hardware",
            "- no physical motor",
            "- no encoder/I/O/EtherCAT validation",
            "- no real robot",
            "- no hard-real-time validation",
            "",
            "P-018 controller contract, VirtualPmacController, ExecutionGate, Validator는 수정하지 않았다.",
            "",
        ]
    )
    return "\n".join(lines)


def _write_json(path: Path, payload: dict[str, object]) -> None:
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
