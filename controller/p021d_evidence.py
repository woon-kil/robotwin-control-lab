"""P-021D live #1J/ causal probe.

정상 motion에서 #1J/를 보내지 않는다.
stall을 만드는 configuration write와 Simulator restart는 하지 않는다.
"""

from __future__ import annotations

import json
import os
import re
import time
from datetime import datetime, timezone
from pathlib import Path

from controller.p021a_investigation import parse_snapshot
from controller.p021d_investigation import (
    NOT_AVAILABLE,
    TIMELINE_NAMES,
    activation_flags,
    choose_recovery_target,
    classify_reactivation,
    final_observation,
    next_probe_target,
)
from controller.power_pmac_config import P017_TEST_CONFIG
from controller.power_pmac_transport import PowerPmacError, SshGpasciiTransport

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "evidence" / "P-021D"
STALL_NAMES = (
    "Motor[1].ServoCtrl",
    "Motor[1].ClosedLoop",
    "Motor[1].AmpEna",
    "Motor[1].JogPos",
    "Motor[1].ProgJogPos",
    "Motor[1].JogVel",
    "Motor[1].JogSpeed",
    "Motor[1].JogTa",
    "Motor[1].JogTs",
    "Motor[1].MaxSpeed",
    "Motor[1].DesPos",
    "Motor[1].DesVel",
    "Motor[1].DesVelZero",
    "Motor[1].Desired.Pos",
    "Motor[1].Desired.Vel",
    "Motor[1].Desired.TimerEnabled",
    "Motor[1].Pos",
    "Motor[1].ActPos",
    "Motor[1].ActVel",
    "Motor[1].InPos",
    "Motor[1].FeFatal",
    "Motor[1].FeWarn",
    "Motor[1].AmpFault",
    "Motor[1].LimitStop",
    "Motor[1].PlusLimit",
    "Motor[1].MinusLimit",
    "Motor[1].Coord",
    "Coord[0].ProgRunning",
    "Motor[1].Status[0]",
    "Motor[1].Status[1]",
)
REACTIVATION_NAMES = (
    "Motor[1].ServoCtrl",
    "Motor[1].ClosedLoop",
    "Motor[1].AmpEna",
    "Motor[1].Status[0]",
    "Motor[1].Status[1]",
    "Motor[1].JogPos",
    "Motor[1].DesPos",
    "Motor[1].Pos",
    "Motor[1].ActPos",
    "Motor[1].FeFatal",
    "Motor[1].AmpFault",
    "Motor[1].LimitStop",
)
_JOG_RE = re.compile(r"#1j=[0-9]+")


def read_names(transport: SshGpasciiTransport, names: tuple[str, ...]) -> tuple[dict[str, str], str, int]:
    exchange = transport.execute_lines(list(names))
    parsed = parse_snapshot(exchange.stdout, names)
    raw = exchange.stdout
    for name in [item for item, value in parsed.items() if value == NOT_AVAILABLE]:
        try:
            single = transport.execute_lines([name])
        except PowerPmacError:
            continue
        raw += "\n" + single.stdout
        parsed[name] = parse_snapshot(single.stdout, (name,))[name]
    return parsed, raw, exchange.returncode


def main() -> None:
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    (EVIDENCE / "raw").mkdir(exist_ok=True)
    transport = SshGpasciiTransport(P017_TEST_CONFIG)
    raw_rows: list[dict[str, object]] = []
    try:
        identity = transport.run_remote("hostname; uname -s")
        address = transport.run_remote("ip -4 -o addr show eth0")
    except PowerPmacError as exc:
        _publish(raw_rows, [], [], None, None, None, False, exc.code, "", "", evidence_complete=False)
        return
    identity_text = identity.stdout.strip()
    address_text = address.stdout.strip()
    host_ok = "ppmac" in identity_text and "172.20.0.200" in address_text
    try:
        baseline, baseline_raw, baseline_rc = read_names(transport, STALL_NAMES)
    except PowerPmacError as exc:
        _publish(raw_rows, [], [], None, None, None, host_ok, exc.code, identity_text, address_text, evidence_complete=False)
        return
    raw_rows.append({"step": "BASELINE", "returncode": baseline_rc, "values": baseline})
    _write(EVIDENCE / "baseline_state.json", {"performed": True, "values": baseline, "raw": baseline_raw, "returncode": baseline_rc, "activation": activation_flags(baseline)})
    baseline_complete = all(baseline.get(name, NOT_AVAILABLE) != NOT_AVAILABLE for name in STALL_NAMES)
    if not host_ok or not baseline_complete:
        _publish(raw_rows, [], [], None, None, None, host_ok, "baseline incomplete" if host_ok else "identity mismatch", identity_text, address_text, evidence_complete=False, baseline=baseline)
        return
    probes: list[dict[str, object]] = []
    commands: list[str] = []
    position = baseline["Motor[1].Pos"]
    jog_position = baseline["Motor[1].JogPos"]
    previous: int | None = None
    stall: dict[str, str] | None = None
    reactivation: dict[str, object] | None = None
    recovery: dict[str, object] | None = None
    try:
        for _index in range(3):
            target = next_probe_target(position, jog_position, previous)
            if target is None:
                _publish(raw_rows, probes, commands, None, None, None, True, "target selection failed", identity_text, address_text, evidence_complete=False, baseline=baseline)
                return
            probe = _run_timeline(transport, target, raw_rows, "PROBE")
            probes.append(probe)
            commands.append(str(probe["command"]))
            _write(EVIDENCE / "healthy_probe.json", {"performed": True, "probes": probes})
            if probe["observation"] != "COMPLETED":
                break
            last = _last_sample(probe)
            position = str(last["Motor[1].Pos"])
            jog_position = str(last["Motor[1].JogPos"])
            previous = target
        else:
            _publish(raw_rows, probes, commands, None, None, None, True, "", identity_text, address_text, evidence_complete=True, baseline=baseline)
            return
        stall_probe = probes[-1]
        if stall_probe["observation"] != "RB-1":
            _publish(raw_rows, probes, commands, None, None, None, True, "probe was neither completion nor RB-1", identity_text, address_text, evidence_complete=False, baseline=baseline)
            return
        stall, stall_raw, stall_rc = read_names(transport, STALL_NAMES)
        raw_rows.append({"step": "STALL_S0", "returncode": stall_rc, "values": stall})
        flags = activation_flags(stall)
        stall_complete = all(stall.get(name, NOT_AVAILABLE) != NOT_AVAILABLE for name in STALL_NAMES) and flags["complete"] is True
        _write(EVIDENCE / "stall_state.json", {"performed": True, "values": stall, "raw": stall_raw, "returncode": stall_rc, "activation": flags, "failed_command": stall_probe["command"]})
        if not stall_complete or flags["servo_ok"] is not True or flags["closed_loop_ok"] is not True or flags["fault_present"] is True:
            _publish(
                raw_rows,
                probes,
                commands,
                stall,
                None,
                None,
                True,
                "stall precondition was not #1J/ only",
                identity_text,
                address_text,
                evidence_complete=stall_complete is True,
                baseline=baseline,
            )
            return
        jog_stop = transport.execute_lines(["#1J/"])
        commands.append("#1J/")
        raw_rows.append({"step": "REACTIVATION", "command": "#1J/", "returncode": jog_stop.returncode, "stdout": jog_stop.stdout, "stderr": jog_stop.stderr})
        reactivated, reactivated_raw, reactivated_rc = read_names(transport, REACTIVATION_NAMES)
        raw_rows.append({"step": "REACTIVATED_S1", "returncode": reactivated_rc, "values": reactivated})
        reactivation = {
            "performed": True,
            "command": "#1J/",
            "ack_present": "\x06" in jog_stop.stdout,
            "ack_is_reactivation_success": False,
            "returncode": jog_stop.returncode,
            "stdout": jog_stop.stdout,
            "stderr": jog_stop.stderr,
            "readback": reactivated,
            "raw": reactivated_raw,
            "activation": activation_flags(reactivated),
        }
        _write(EVIDENCE / "reactivation.json", reactivation)
        reactivated_flags = activation_flags(reactivated)
        if reactivated_flags["closed_loop_ok"] is not True or reactivated_flags["fault_present"] is not False:
            _publish(raw_rows, probes, commands, stall, reactivation, None, True, "reactivation read-back was not closed-loop and fault-free", identity_text, address_text, evidence_complete=False, baseline=baseline)
            return
        failed_target = int(stall_probe["target"])
        recovery_target = choose_recovery_target(stall["Motor[1].Pos"], stall["Motor[1].JogPos"], failed_target)
        if recovery_target is None or recovery_target == failed_target:
            _publish(raw_rows, probes, commands, stall, reactivation, None, True, "recovery target selection failed", identity_text, address_text, evidence_complete=False, baseline=baseline)
            return
        recovery = _run_timeline(transport, recovery_target, raw_rows, "RECOVERY")
        commands.append(str(recovery["command"]))
        _publish(raw_rows, probes, commands, stall, reactivation, recovery, True, "", identity_text, address_text, evidence_complete=True, baseline=baseline)
    except PowerPmacError as exc:
        raw_rows.append({"step": "ERROR", "error": exc.code})
        _publish(raw_rows, probes, commands, stall, reactivation, recovery, host_ok, exc.code, identity_text, address_text, evidence_complete=False, baseline=baseline)


def _run_timeline(transport: SshGpasciiTransport, target: int, raw_rows: list[dict[str, object]], label: str) -> dict[str, object]:
    before, _raw, _returncode = read_names(transport, TIMELINE_NAMES)
    command = "#1j=" + str(target)
    started = time.monotonic()
    jog = transport.execute_lines([command])
    raw_rows.append({"step": label + "_COMMAND", "command": command, "returncode": jog.returncode, "stdout": jog.stdout, "stderr": jog.stderr})
    samples: list[dict[str, object]] = []
    for index in (1, 2, 3):
        time.sleep(0.5)
        parsed, _raw, returncode = read_names(transport, TIMELINE_NAMES)
        sample: dict[str, object] = {name: parsed[name] for name in TIMELINE_NAMES}
        sample["t_s"] = time.monotonic() - started
        samples.append(sample)
        raw_rows.append({"step": f"{label}_T{index}", "t_s": sample["t_s"], "returncode": returncode, "sample": {name: parsed[name] for name in TIMELINE_NAMES}})
    observed = final_observation([{name: str(sample[name]) for name in TIMELINE_NAMES} for sample in samples], str(target))
    text = jog.stdout + "\n" + jog.stderr
    return {
        "target": target,
        "units": "pmac_motor_unit",
        "command": command,
        "ack_present": "\x06" in jog.stdout,
        "error_present": "error #" in text.lower(),
        "returncode": jog.returncode,
        "ack_is_motion_completion": False,
        "timeline_note": "SSH/gpascii sample times are not a high-frequency servo trace",
        "before": {name: before[name] for name in TIMELINE_NAMES},
        "samples": samples,
        "observation": observed,
        "completed": observed == "COMPLETED",
        "stdout": jog.stdout,
        "stderr": jog.stderr,
    }


def _publish(
    raw_rows: list[dict[str, object]],
    probes: list[dict[str, object]],
    commands: list[str],
    stall: dict[str, str] | None,
    reactivation: dict[str, object] | None,
    recovery: dict[str, object] | None,
    host_ok: bool,
    reason: str,
    identity_text: str,
    address_text: str,
    *,
    evidence_complete: bool,
    baseline: dict[str, str] | None = None,
) -> None:
    flags = activation_flags(stall) if stall is not None else {"servo_ok": None, "closed_loop_ok": None, "fault_present": None, "complete": False}
    forbidden = [command for command in commands if command != "#1J/" and _JOG_RE.fullmatch(command) is None]
    recovery_result = recovery["observation"] if recovery is not None else None
    classification = classify_reactivation(
        evidence_complete=evidence_complete and host_ok and not forbidden,
        unexpected_write=bool(forbidden),
        restart_performed=False,
        probe_results=tuple(str(probe["observation"]) for probe in probes),
        stall_servo_ok=flags["servo_ok"] if isinstance(flags["servo_ok"], bool) else None,
        stall_closed_loop_ok=flags["closed_loop_ok"] if isinstance(flags["closed_loop_ok"], bool) else None,
        stall_fault_present=flags["fault_present"] if isinstance(flags["fault_present"], bool) else None,
        intervention="#1J/" if reactivation is not None else None,
        recovery_result=str(recovery_result) if isinstance(recovery_result, str) else None,
        cycles=1,
    )
    if not probes:
        _write(EVIDENCE / "healthy_probe.json", {"performed": False, "reason": reason})
    if stall is None:
        _write(EVIDENCE / "stall_state.json", {"performed": False})
    if reactivation is None:
        _write(EVIDENCE / "reactivation.json", {"performed": False})
    if recovery is None:
        _write(EVIDENCE / "post_reactivation_motion.json", {"performed": False})
    else:
        _write(EVIDENCE / "post_reactivation_motion.json", recovery)
    stall_probe = next((probe for probe in probes if probe.get("observation") == "RB-1"), None)
    _write(EVIDENCE / "causal_sequence.json", _sequence(baseline, stall, stall_probe, reactivation, recovery, classification))
    _write(EVIDENCE / "classification.json", classification)
    _write(
        EVIDENCE / "write_audit.json",
        {
            "writes_performed": commands,
            "forbidden_writes": forbidden,
            "configuration_modifications": forbidden,
            "restart_performed": False,
            "windows_restart_performed": False,
            "ide_restart_performed": False,
        },
    )
    (EVIDENCE / "environment.md").write_text(_environment(identity_text, address_text, host_ok), encoding="utf-8")
    (EVIDENCE / "summary.md").write_text(_summary(baseline, probes, stall, reactivation, recovery, classification, identity_text, reason), encoding="utf-8")
    (EVIDENCE / "raw" / "investigation.jsonl").write_text("\n".join(json.dumps(row, ensure_ascii=False) for row in raw_rows) + ("\n" if raw_rows else ""), encoding="utf-8")
    _reject_secret(EVIDENCE)
    print(json.dumps({"classification": classification["classification"], "causal_claim": classification["causal_claim"], "confirmed": classification["confirmed"]}, ensure_ascii=False))


def _sequence(
    baseline: dict[str, str] | None,
    stall: dict[str, str] | None,
    stall_probe: dict[str, object] | None,
    reactivation: dict[str, object] | None,
    recovery: dict[str, object] | None,
    classification: dict[str, object],
) -> dict[str, object]:
    source = stall if stall is not None else baseline
    precondition = {"performed": source is not None}
    if source is not None:
        flags = activation_flags(source)
        precondition = {
            "performed": True,
            "ServoCtrl": source.get("Motor[1].ServoCtrl"),
            "ClosedLoop": source.get("Motor[1].ClosedLoop"),
            "faults": 0 if flags["fault_present"] is False else flags["fault_present"],
        }
    stall_record: dict[str, object] = {"performed": stall_probe is not None}
    if stall_probe is not None:
        last = _last_sample(stall_probe)
        stall_record.update(
            {
                "command": stall_probe["command"],
                "JogPos": last.get("Motor[1].JogPos"),
                "DesPos": last.get("Motor[1].DesPos"),
                "Pos": last.get("Motor[1].Pos"),
                "ActPos": last.get("Motor[1].ActPos"),
                "DesVel": last.get("Motor[1].DesVel"),
                "completed": stall_probe["completed"],
            }
        )
    recovery_record: dict[str, object] = {"performed": recovery is not None}
    if recovery is not None:
        last = _last_sample(recovery)
        recovery_record.update(
            {
                "command": recovery["command"],
                "JogPos": last.get("Motor[1].JogPos"),
                "DesPos": last.get("Motor[1].DesPos"),
                "Pos": last.get("Motor[1].Pos"),
                "ActPos": last.get("Motor[1].ActPos"),
                "completed": recovery["completed"],
            }
        )
    return {
        "precondition": precondition,
        "stall": stall_record,
        "intervention": {"performed": reactivation is not None, "command": None if reactivation is None else reactivation["command"]},
        "restart": False,
        "configuration_change": False,
        "recovery": recovery_record,
        "classification": classification["classification"],
    }


def _summary(
    baseline: dict[str, str] | None,
    probes: list[dict[str, object]],
    stall: dict[str, str] | None,
    reactivation: dict[str, object] | None,
    recovery: dict[str, object] | None,
    classification: dict[str, object],
    identity_text: str,
    reason: str,
) -> str:
    lines = [
        "# P-021D summary",
        "",
        f"측정: {datetime.now(timezone.utc).isoformat()}",
        "",
        "Official Power PMAC Simulator #1J/ reactivation causal experiment only. P-021 FINAL PASS를 선언하지 않는다. P-022는 시작하지 않는다. stall은 유도하지 않았다.",
        "",
        f"classification: {classification['classification']} {classification.get('classification_name', '')}".rstrip(),
        f"confidence: {classification['confidence']}",
        f"causal_claim: {classification['causal_claim']}",
        f"confirmed: {classification['confirmed']}",
        f"identity: {identity_text.replace(chr(10), ' / ')}",
    ]
    if reason:
        lines.append(f"stop_reason: {reason}")
    if baseline is not None:
        lines.append(
            "baseline ServoCtrl/ClosedLoop/AmpEna/JogPos/DesPos/Pos/ActPos: "
            f"{baseline.get('Motor[1].ServoCtrl')}/{baseline.get('Motor[1].ClosedLoop')}/{baseline.get('Motor[1].AmpEna')}/"
            f"{baseline.get('Motor[1].JogPos')}/{baseline.get('Motor[1].DesPos')}/{baseline.get('Motor[1].Pos')}/{baseline.get('Motor[1].ActPos')}"
        )
    for index, probe in enumerate(probes, start=1):
        last = _last_sample(probe)
        lines.append(
            f"probe {index} {probe['command']} observation {probe['observation']} ack {probe['ack_present']} error {probe['error_present']} "
            f"JogPos {last.get('Motor[1].JogPos')} DesPos {last.get('Motor[1].DesPos')} Pos {last.get('Motor[1].Pos')} ActPos {last.get('Motor[1].ActPos')}"
        )
    if stall is not None:
        lines.append(
            "STALL_S0 ServoCtrl/ClosedLoop/AmpEna/FeFatal/AmpFault/LimitStop: "
            f"{stall.get('Motor[1].ServoCtrl')}/{stall.get('Motor[1].ClosedLoop')}/{stall.get('Motor[1].AmpEna')}/"
            f"{stall.get('Motor[1].FeFatal')}/{stall.get('Motor[1].AmpFault')}/{stall.get('Motor[1].LimitStop')}"
        )
    lines.append("reactivation: not performed" if reactivation is None else f"reactivation: {reactivation['command']} read-back ClosedLoop {reactivation['readback'].get('Motor[1].ClosedLoop') if isinstance(reactivation.get('readback'), dict) else None}")
    if recovery is None:
        lines.append("new motion: not performed")
    else:
        last = _last_sample(recovery)
        lines.append(f"new motion: {recovery['command']} completed {recovery['completed']} DesPos {last.get('Motor[1].DesPos')} Pos {last.get('Motor[1].Pos')} ActPos {last.get('Motor[1].ActPos')}")
    lines.extend(
        [
            "",
            "## Claim boundary",
            "",
            "- Official Power PMAC Simulator only",
            "- software virtual Motor[1]",
            "- #1J/ reactivation causal experiment only",
            "- PMAC motor units",
            "- ACK is not motion completion",
            "- no real Power PMAC hardware",
            "- no physical motor",
            "- no physical encoder",
            "- no amplifier",
            "- no EtherCAT",
            "- no physical I/O",
            "- no real robot",
            "- no servo tuning",
            "- no industrial commissioning",
            "- no hard-real-time validation",
            "",
        ]
    )
    return "\n".join(lines)


def _environment(identity_text: str, address_text: str, host_ok: bool) -> str:
    return "\n".join(
        [
            "# P-021D environment",
            "",
            f"측정: {datetime.now(timezone.utc).isoformat()}",
            "",
            "```text",
            "client: MacBook",
            "path: MacBook -> 192.168.0.146:22022 -> Windows portproxy -> 172.20.0.200:22",
            f"remote identity: {identity_text.replace(chr(10), ' / ')}",
            f"eth0: {address_text.replace(chr(10), ' / ')}",
            "pmac backend: Official Power PMAC Simulator 1.2.1 CK5M",
            "firmware_if_observed: not queried by this runner",
            "simulator_restart_performed: false",
            "windows_restart_performed: false",
            "ide_restart_performed: false",
            f"runner_host_check: {host_ok}",
            "simulator_IP: 172.20.0.200",
            "credential: runtime environment variable PMAC_SSH_PASSWORD; value not recorded",
            "```",
            "",
            "P-019부터 P-021C evidence는 수정하지 않는다.",
            "",
        ]
    )


def _last_sample(probe: dict[str, object]) -> dict[str, object]:
    samples = probe["samples"]
    assert isinstance(samples, list) and samples
    last = samples[-1]
    assert isinstance(last, dict)
    return last


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
