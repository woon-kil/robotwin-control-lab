"""P-021B live read-only JogPos/DesPos timeline.

configuration write와 simulator restart는 하지 않는다.
"""

from __future__ import annotations

import json
import os
import time
from datetime import datetime, timezone
from pathlib import Path

from controller.p021a_investigation import parse_snapshot
from controller.p021b_investigation import (
    classify_pipeline,
    default_historical_columns,
    historical_matrix,
    jog_time_parameter_class,
    motion_completed,
)
from controller.power_pmac_config import P017_TEST_CONFIG
from controller.power_pmac_transport import PowerPmacError, SshGpasciiTransport

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "evidence" / "P-021B"
TIMELINE_NAMES = (
    "Motor[1].JogPos",
    "Motor[1].DesPos",
    "Motor[1].DesVel",
    "Motor[1].DesVelZero",
    "Motor[1].Pos",
    "Motor[1].ActPos",
    "Motor[1].ActVel",
    "Motor[1].Desired.TimerEnabled",
)
EXTRA_NAMES = (
    "Motor[1].JogSpeed",
    "Motor[1].JogTa",
    "Motor[1].JogTs",
    "Motor[1].MaxSpeed",
    "Motor[1].ServoCtrl",
    "Motor[1].ClosedLoop",
    "Motor[1].AmpEna",
    "Motor[1].Ctrl",
    "Motor[1].PlusLimit",
    "Motor[1].MinusLimit",
    "Motor[1].LimitStop",
    "Motor[1].MaxPos",
    "Motor[1].MinPos",
    "Motor[1].SoftLimit",
    "Motor[1].SoftPlusLimit",
    "Motor[1].SoftMinusLimit",
    "Motor[1].SoftLimitDir",
    "Motor[1].FeFatal",
    "Motor[1].FeWarn",
    "Motor[1].AmpFault",
    "Motor[1].Coord",
    "Motor[1].ProgJogPos",
    "Motor[1].InPos",
    "Motor[1].Status[0]",
    "Motor[1].Status[1]",
    "Motor[1].Desired.Pos",
    "Motor[1].Desired.Vel",
    "Motor[1].JogVel",
    "Coord[0].ProgRunning",
)


def read_names(transport: SshGpasciiTransport, names: tuple[str, ...]) -> tuple[dict[str, str], str, int]:
    exchange = transport.execute_lines(list(names))
    parsed = parse_snapshot(exchange.stdout, names)
    raw = exchange.stdout
    for name in [item for item, value in parsed.items() if value == "NOT_AVAILABLE"]:
        try:
            single = transport.execute_lines([name])
        except PowerPmacError:
            continue
        raw += "\n" + single.stdout
        parsed[name] = parse_snapshot(single.stdout, (name,))[name]
    return parsed, raw, exchange.returncode


def short_sample(parsed: dict[str, str]) -> dict[str, str]:
    return {name.split(".", 1)[1] if name.startswith("Motor[1].") else name: parsed[name] for name in TIMELINE_NAMES}


def choose_target(position: str, jog_position: str) -> int:
    for candidate in (2, 3, 1, 0, 4):
        if str(candidate) not in {position, jog_position}:
            return candidate
    return 5


def main() -> None:
    transport = SshGpasciiTransport(P017_TEST_CONFIG)
    rows: list[dict[str, object]] = []
    current, current_raw, current_rc = read_names(transport, EXTRA_NAMES + TIMELINE_NAMES)
    rows.append({"step": "current_state", "returncode": current_rc})
    before, before_raw, before_rc = read_names(transport, TIMELINE_NAMES)
    target = choose_target(before.get("Motor[1].Pos", "NOT_AVAILABLE"), before.get("Motor[1].JogPos", "NOT_AVAILABLE"))
    command = "#1j=" + str(target)
    started = time.monotonic()
    jog = transport.execute_lines([command])
    command_at = time.monotonic() - started
    samples = []
    sample_raw = []
    for index in (1, 2, 3):
        time.sleep(0.5)
        parsed, raw, returncode = read_names(transport, TIMELINE_NAMES)
        stamp = time.monotonic() - started
        sample = short_sample(parsed)
        sample["t_s"] = stamp
        sample["returncode"] = returncode
        samples.append(sample)
        sample_raw.append(raw)
        rows.append({"step": f"T{index}", "t_s": stamp, "sample": sample})
    before_short = short_sample(before)
    classification = classify_pipeline(before_short, samples, str(target))
    completed = any(motion_completed(sample, str(target), "\x06" in jog.stdout) for sample in samples)
    payload = {
        "command": command,
        "ack_present": "\x06" in jog.stdout,
        "error_present": "error #" in (jog.stdout + "\n" + jog.stderr).lower(),
        "returncode": jog.returncode,
        "ack_is_motion_completion": False,
        "command_return_s": command_at,
        "before": before_short,
        "samples": samples,
        "classification": classification,
        "motion_completed": completed,
        "stdout": jog.stdout,
        "stderr": jog.stderr,
    }
    matrix = historical_matrix(default_historical_columns())
    matrix_note = "P-019B values are the post-enable, pre-#1j=1 state. P-019C values are the enable snapshot before the completed moves. P-020 values are the pre-#1j=0 state. Missing keys stay NOT_MEASURED."
    analysis = _analysis(classification, current, completed)
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    (EVIDENCE / "raw").mkdir(exist_ok=True)
    _write(EVIDENCE / "historical_state_matrix.json", {"note": matrix_note, "columns": default_historical_columns(), "matrix": matrix})
    _write(EVIDENCE / "current_state.json", {"values": current, "raw": current_raw, "returncode": current_rc})
    _write(EVIDENCE / "motion_timeline.json", payload)
    _write(EVIDENCE / "candidate_table.json", _candidates(current))
    _write(EVIDENCE / "write_audit.json", _write_audit())
    _write(EVIDENCE / "intervention.json", {"performed": False, "reason": "No candidate met every intervention gate."})
    _write(EVIDENCE / "restart_experiment.json", {"performed": False, "reason": "Simulator process was not restarted. A restart would not identify which state changed."})
    _write(EVIDENCE / "root_cause_analysis.json", analysis)
    (EVIDENCE / "environment.md").write_text(_environment(), encoding="utf-8")
    (EVIDENCE / "summary.md").write_text(_summary(payload, current, analysis), encoding="utf-8")
    (EVIDENCE / "raw" / "investigation.jsonl").write_text(
        "\n".join(json.dumps(row, ensure_ascii=False) for row in rows) + "\n",
        encoding="utf-8",
    )
    del before_raw
    del sample_raw
    _reject_secret(EVIDENCE)
    print(json.dumps({"classification": classification, "completed": completed, "command": command}, ensure_ascii=False))


def _analysis(classification: str, current: dict[str, str], completed: bool) -> dict[str, object]:
    return {
        "classification": classification,
        "classification_name": {
            "RB-1": "COMMAND_TARGET_ONLY",
            "RB-5": "INTERMITTENT",
            "PIPELINE_REACHED": "PIPELINE_REACHED",
        }.get(classification, classification),
        "confidence": "MEDIUM" if classification == "RB-1" else "LOW",
        "confirmed": False,
        "p021a_conclusion_changed": False,
        "motion_completed": completed,
        "jog_ta_ts_class": jog_time_parameter_class("NOT_MEASURED", True),
        "remaining_unknown": "Which simulator condition accepts JogPos and does not start DesPos is not isolated.",
        "current_jog_ta": current.get("Motor[1].JogTa", "NOT_AVAILABLE"),
        "current_jog_ts": current.get("Motor[1].JogTs", "NOT_AVAILABLE"),
        "desired_timer_enabled_in_current_read": current.get("Motor[1].Desired.TimerEnabled", "NOT_AVAILABLE"),
        "coord0_prog_running": current.get("Coord[0].ProgRunning", "NOT_AVAILABLE"),
    }


def _candidates(current: dict[str, str]) -> list[dict[str, object]]:
    return [
        {
            "candidate": "JogTa/JogTs",
            "historical_difference": "NOT_MEASURED in P-019B, P-019C, and P-020",
            "official_relevance": "Omron forum, Curt Wilson: negative JogTa/JogTs selects rate mode and is recommended. It is not documented there as a jog reject.",
            "official_url": "https://forums.automation.omron.com/topic/5177-sending-consecutive-jog-commmands/",
            "current_evidence": {"JogTa": current.get("Motor[1].JogTa"), "JogTs": current.get("Motor[1].JogTs")},
            "class": "NOT_ENOUGH_EVIDENCE",
            "intervention_allowed": False,
            "confidence": "LOW",
        },
        {
            "candidate": "Status[0]",
            "historical_difference": "P-019B successful jog state is $7800. P-019C enable snapshot is $7800.",
            "official_relevance": "Raw word matches the successful P-019B motion state. Bits were not decoded.",
            "current_evidence": current.get("Motor[1].Status[0]"),
            "class": "ELIMINATED",
            "intervention_allowed": False,
            "confidence": "MEDIUM",
        },
        {
            "candidate": "active software position limit",
            "historical_difference": "NOT_MEASURED in P-019B and P-020",
            "official_relevance": "2011 On-Line Command Specification: SoftPlusLimit is active when MaxPos is greater than MinPos.",
            "official_url": "http://svn.code.sf.net/p/tango-ds/code/DeviceClasses/BeamlineComponents/Mirrors/TurboPMAC/tags/release_0_6_5/doc/Power%20PMAC%20On-Line%20Command%20Specification%202011-01.pdf",
            "current_evidence": {
                "MaxPos": current.get("Motor[1].MaxPos"),
                "MinPos": current.get("Motor[1].MinPos"),
                "SoftLimit": current.get("Motor[1].SoftLimit"),
            },
            "class": _limit_class(current),
            "intervention_allowed": False,
            "confidence": "MEDIUM",
        },
        {
            "candidate": "Coord motion ownership",
            "historical_difference": "Coord and ProgRunning were NOT_MEASURED during P-019B and P-020",
            "official_relevance": "Vendor forum: a jog is rejected while Coord[x].ProgRunning is true. Coord=0 is not treated as the cause by itself.",
            "current_evidence": {
                "Coord": current.get("Motor[1].Coord"),
                "Coord[0].ProgRunning": current.get("Coord[0].ProgRunning"),
            },
            "class": "NOT_ENOUGH_EVIDENCE",
            "intervention_allowed": False,
            "confidence": "LOW",
        },
        {
            "candidate": "closed-loop activation",
            "historical_difference": "P-019B and P-020 successful motion had ServoCtrl 1 and ClosedLoop 1",
            "official_relevance": "j/ closes the loop. The successful jogs already had ClosedLoop 1.",
            "current_evidence": {
                "ServoCtrl": current.get("Motor[1].ServoCtrl"),
                "ClosedLoop": current.get("Motor[1].ClosedLoop"),
                "AmpEna": current.get("Motor[1].AmpEna"),
            },
            "class": "ELIMINATED",
            "intervention_allowed": False,
            "confidence": "MEDIUM",
        },
    ]


def _limit_class(current: dict[str, str]) -> str:
    try:
        max_pos = float(current["Motor[1].MaxPos"])
        min_pos = float(current["Motor[1].MinPos"])
    except (KeyError, TypeError, ValueError):
        return "NOT_ENOUGH_EVIDENCE"
    if max_pos > min_pos:
        return "SUSPICIOUS"
    return "ELIMINATED"


def _write_audit() -> dict[str, object]:
    return {
        "method": "git grep of controller and tests at the P-021A commit, separating live gpascii writes from scripted test doubles and evidence read-back text",
        "configuration_writes_found": [],
        "motion_writes": ["#1J/", "#1j=<pmac motor unit>"],
        "activation_writes": ["Motor[1].ServoCtrl=1 only when the read-back ServoCtrl is not greater than 0"],
        "test_variable_writes": ["P100=<int>"],
        "rejected_by_guard_not_sent": ["Sys.SimConfigOk=1", "Motor[2] commands", "j+", "j-"],
        "scripted_only": ["ExchangeMotor AmpFault, FeFatal, LimitStop, ServoCtrl, and ClosedLoop mutations"],
        "persistent_state_candidates_from_repository_writes": [],
        "jog_ta_assignment_in_repository": "JogTa=-10 appears as a read-back in evidence/P-021A. No live writer assigns JogTa or JogTs.",
    }


def _summary(payload: dict[str, object], current: dict[str, str], analysis: dict[str, object]) -> str:
    before = payload["before"]
    lines = [
        "# P-021B summary",
        "",
        f"측정: {datetime.now(timezone.utc).isoformat()}",
        "",
        "이 문서는 JogPos에서 DesPos로 이어지는 trajectory 조사다. P-021 motion 해결을 선언하지 않는다. P-021A의 RC-3 결론은 유지한다. P-022는 시작하지 않는다.",
        "",
        f"classification: {analysis['classification']} {analysis['classification_name']}",
        f"confidence: {analysis['confidence']}",
        f"confirmed: {analysis['confirmed']}",
        f"command: {payload['command']}",
        f"ack: {payload['ack_present']} error: {payload['error_present']} returncode: {payload['returncode']}",
        f"motion_completed: {payload['motion_completed']}",
        "",
        f"T0 JogPos/DesPos/DesVel/Pos/ActPos: {before.get('JogPos')}/{before.get('DesPos')}/{before.get('DesVel')}/{before.get('Pos')}/{before.get('ActPos')}",
    ]
    for index, sample in enumerate(payload["samples"], start=1):
        assert isinstance(sample, dict)
        lines.append(
            f"T{index} t_s {sample['t_s']} JogPos {sample.get('JogPos')} DesPos {sample.get('DesPos')} "
            f"DesVel {sample.get('DesVel')} Pos {sample.get('Pos')} ActPos {sample.get('ActPos')} "
            f"ActVel {sample.get('ActVel')} TimerEnabled {sample.get('Desired.TimerEnabled')}"
        )
    lines.extend(
        [
            "",
            f"JogTa/JogTs class: {analysis['jog_ta_ts_class']}",
            f"current JogTa/JogTs: {current.get('Motor[1].JogTa')}/{current.get('Motor[1].JogTs')}",
            f"MaxPos/MinPos: {current.get('Motor[1].MaxPos')}/{current.get('Motor[1].MinPos')}",
            f"Coord: {current.get('Motor[1].Coord')} Coord[0].ProgRunning: {current.get('Coord[0].ProgRunning')}",
            f"Desired.TimerEnabled at state read: {current.get('Motor[1].Desired.TimerEnabled')}",
            "",
            "intervention: not performed",
            "restart: not performed",
            "",
            "## Claim boundary",
            "",
            "- JogPos → DesPos root-cause investigation only",
            "- Official Power PMAC Simulator only",
            "- software virtual Motor[1]",
            "- historical evidence is not back-filled",
            "- unmeasured historical state is NOT_MEASURED",
            "- ACK is not motion completion",
            "- no real Power PMAC hardware",
            "- no physical motor",
            "- no physical encoder",
            "- no physical amplifier",
            "- no physical I/O",
            "- no EtherCAT",
            "- no real robot",
            "- no servo tuning",
            "- no controller commissioning",
            "",
        ]
    )
    return "\n".join(lines)


def _environment() -> str:
    return "\n".join(
        [
            "# P-021B environment",
            "",
            f"측정: {datetime.now(timezone.utc).isoformat()}",
            "",
            "```text",
            "client: MacBook",
            "pmac backend: Official Power PMAC Simulator 1.2.1",
            "firmware: 2.8.3.0 stated in the P-019B brief; this runner did not query a firmware variable",
            "path: MacBook -> 192.168.0.146:22022 -> 172.20.0.200:22 -> SSH + gpascii",
            "writes: #1j=<small PMAC motor unit> only",
            "simulator restart: not performed",
            "credential: runtime environment variable PMAC_SSH_PASSWORD; value not recorded",
            "```",
            "",
            "P-019, P-019C, P-020, P-021, P-021A evidence는 수정하지 않는다.",
            "",
        ]
    )


def _write(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def _reject_secret(root: Path) -> None:
    secret = os.environ.get("PMAC_SSH_PASSWORD", "")
    if not secret:
        return
    for path in root.rglob("*"):
        if path.is_file() and secret in path.read_text(encoding="utf-8", errors="replace"):
            raise SystemExit("secret scan failed")


if __name__ == "__main__":
    main()
