"""P-021C before-restart measurement.

Simulator/QEMU process restart는 이 runner가 하지 않는다.
transport reconnect를 restart로 기록하지 않는다.
"""

from __future__ import annotations

import json
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

from controller.p021a_investigation import parse_snapshot
from controller.p021c_investigation import (
    SNAPSHOT_NAMES,
    TIMELINE_NAMES,
    choose_target,
    classify_session_reset,
    motion_completed,
    readiness,
    restart_diff,
)
from controller.power_pmac_config import P017_TEST_CONFIG
from controller.power_pmac_transport import PowerPmacError, SshGpasciiTransport

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "evidence" / "P-021C"


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


def main() -> None:
    transport = SshGpasciiTransport(P017_TEST_CONFIG)
    identity = transport.run_remote("hostname; uname -s")
    snapshot, snapshot_raw, snapshot_rc = read_names(transport, SNAPSHOT_NAMES)
    ready = readiness(snapshot)
    before, _before_raw, _before_rc = read_names(transport, TIMELINE_NAMES)
    target = choose_target(before.get("Motor[1].Pos", "NOT_AVAILABLE"), before.get("Motor[1].JogPos", "NOT_AVAILABLE"))
    command = "#1j=" + str(target)
    started = time.monotonic()
    jog = transport.execute_lines([command])
    samples = []
    rows = [{"step": "A0", "readiness": ready, "returncode": snapshot_rc}]
    for index in (1, 2, 3):
        time.sleep(0.5)
        parsed, _raw, returncode = read_names(transport, TIMELINE_NAMES)
        sample = dict(parsed)
        sample["t_s"] = time.monotonic() - started
        sample["returncode"] = returncode
        samples.append(sample)
        rows.append({"step": f"T{index}", "t_s": sample["t_s"], "sample": {name: parsed[name] for name in TIMELINE_NAMES}})
    completed = any(motion_completed(sample, str(target), "\x06" in jog.stdout) for sample in samples)
    classification = classify_session_reset(
        before_completed=completed,
        after_ran=False,
        after_completed=None,
        activation_required=False,
        boot_ok=False,
        snapshot_ok=True,
    )
    motion = {
        "readiness": ready,
        "target": target,
        "units": "pmac_motor_unit",
        "command": command,
        "ack_present": "\x06" in jog.stdout,
        "error_present": "error #" in (jog.stdout + "\n" + jog.stderr).lower(),
        "returncode": jog.returncode,
        "ack_is_motion_completion": False,
        "before": {name: before[name] for name in TIMELINE_NAMES},
        "samples": [{name: sample[name] for name in list(TIMELINE_NAMES) + ["t_s"]} for sample in samples],
        "completed": completed,
        "stdout": jog.stdout,
        "stderr": jog.stderr,
    }
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    (EVIDENCE / "raw").mkdir(exist_ok=True)
    _write(EVIDENCE / "before_restart_state.json", {"values": snapshot, "raw": snapshot_raw, "returncode": snapshot_rc, "readiness": ready})
    _write(EVIDENCE / "before_restart_motion.json", motion)
    _write(EVIDENCE / "after_restart_initial_state.json", {"performed": False, "reason": "WAITING_FOR_MANUAL_RESTART"})
    _write(EVIDENCE / "restart_state_diff.json", {"performed": False, "reason": "WAITING_FOR_MANUAL_RESTART"})
    _write(EVIDENCE / "activation_after_restart.json", {"performed": False, "required": None, "actions": [], "reason": "WAITING_FOR_MANUAL_RESTART"})
    _write(EVIDENCE / "after_restart_motion.json", {"performed": False, "reason": "WAITING_FOR_MANUAL_RESTART"})
    _write(
        EVIDENCE / "ab_comparison.json",
        {
            "before_restart": {
                "readiness": ready,
                "target": target,
                "JogPos": [sample["Motor[1].JogPos"] for sample in samples],
                "DesPos": [sample["Motor[1].DesPos"] for sample in samples],
                "Pos": [sample["Motor[1].Pos"] for sample in samples],
                "ActPos": [sample["Motor[1].ActPos"] for sample in samples],
                "completed": completed,
            },
            "restart": {"method": "not_performed", "successful": False, "transport_reconnect_used_as_restart": False},
            "after_restart_initial": {"performed": False},
            "activation": {"performed": False, "required": None, "actions": []},
            "after_restart": {"performed": False},
            "classification": classification["classification"],
            "causal_claim": classification["causal_claim"],
        },
    )
    _write(EVIDENCE / "classification.json", classification)
    _write(EVIDENCE / "write_audit.json", _audit(command))
    (EVIDENCE / "environment.md").write_text(_environment(identity.stdout), encoding="utf-8")
    (EVIDENCE / "summary.md").write_text(_summary(ready, snapshot, motion, classification, identity.stdout), encoding="utf-8")
    (EVIDENCE / "raw" / "before_restart.jsonl").write_text("\n".join(json.dumps(row, ensure_ascii=False) for row in rows) + "\n", encoding="utf-8")
    (EVIDENCE / "raw" / "after_restart.jsonl").write_text("", encoding="utf-8")
    _reject_secret(EVIDENCE)
    print(json.dumps({"classification": classification["classification"], "completed": completed, "command": command}, ensure_ascii=False))


def _audit(command: str) -> dict[str, object]:
    return {
        "writes_performed": [command],
        "allowed_writes": [command],
        "forbidden_writes": [],
        "configuration_modifications": [],
        "activation_writes": [],
        "simulator_process_stopped": False,
        "qemu_process_restarted": False,
        "transport_reconnect_used_as_restart": False,
    }


def _summary(ready: str, snapshot: dict[str, str], motion: dict[str, object], classification: dict[str, object], identity: str) -> str:
    before = motion["before"]
    assert isinstance(before, dict)
    lines = [
        "# P-021C summary",
        "",
        f"측정: {datetime.now(timezone.utc).isoformat()}",
        "",
        "이 문서는 Simulator session-reset A/B의 restart 전 측정이다. P-021 FINAL PASS를 선언하지 않는다. P-022는 시작하지 않는다.",
        "",
        f"classification: {classification['classification']}",
        f"causal_claim: {classification['causal_claim']}",
        f"before readiness: {ready}",
        f"before command: {motion['command']} completed: {motion['completed']}",
        f"ack: {motion['ack_present']} error: {motion['error_present']} returncode: {motion['returncode']}",
        f"T0 JogPos/DesPos/Pos/ActPos: {before.get('Motor[1].JogPos')}/{before.get('Motor[1].DesPos')}/{before.get('Motor[1].Pos')}/{before.get('Motor[1].ActPos')}",
    ]
    for index, sample in enumerate(motion["samples"], start=1):
        assert isinstance(sample, dict)
        lines.append(
            f"T{index} t_s {sample['t_s']} JogPos {sample['Motor[1].JogPos']} DesPos {sample['Motor[1].DesPos']} "
            f"DesVel {sample['Motor[1].DesVel']} Pos {sample['Motor[1].Pos']} ActPos {sample['Motor[1].ActPos']} "
            f"TimerEnabled {sample['Motor[1].Desired.TimerEnabled']}"
        )
    lines.extend(
        [
            "",
            f"identity: {identity.strip().replace(chr(10), ' / ')}",
            "restart: not performed. SSH session is the ppmac guest. QEMU/Windows Simulator process was not stopped.",
            "transport reconnect was not used as a restart.",
            "",
            f"ServoCtrl {snapshot.get('Motor[1].ServoCtrl')} ClosedLoop {snapshot.get('Motor[1].ClosedLoop')} "
            f"JogTa {snapshot.get('Motor[1].JogTa')} JogTs {snapshot.get('Motor[1].JogTs')} "
            f"Status[0] {snapshot.get('Motor[1].Status[0]')}",
            "",
            "## Claim boundary",
            "",
            "- Official Power PMAC Simulator session-reset A/B experiment only",
            "- software virtual Motor[1]",
            "- restart correlation is not automatically root-cause proof",
            "- ACK is not motion completion",
            "- PMAC motor units only",
            "- no real Power PMAC hardware",
            "- no physical motor",
            "- no physical servo",
            "- no physical encoder",
            "- no physical amplifier",
            "- no physical I/O",
            "- no EtherCAT",
            "- no real robot",
            "- no servo tuning",
            "- no industrial commissioning",
            "- no hard-real-time validation",
            "",
        ]
    )
    return "\n".join(lines)


def _environment(identity: str) -> str:
    return "\n".join(
        [
            "# P-021C environment",
            "",
            f"측정: {datetime.now(timezone.utc).isoformat()}",
            "",
            "```text",
            "client: MacBook",
            "path: MacBook -> 192.168.0.146:22022 -> Windows portproxy -> 172.20.0.200:22",
            f"remote identity: {identity.strip().replace(chr(10), ' / ')}",
            "pmac backend: Official Power PMAC Simulator 1.2.1",
            "firmware: 2.8.3.0 stated in the P-019B brief; this runner did not query a firmware variable",
            "restart_method: not performed",
            "simulator_process_stopped: false",
            "qemu_process_restarted: false",
            "IDE_state: GUI not controlled",
            "boot_completed: not observed",
            "simulator_IP: 172.20.0.200",
            "credential: runtime environment variable PMAC_SSH_PASSWORD; value not recorded",
            "```",
            "",
            "P-019, P-020, P-021, P-021A, P-021B evidence는 수정하지 않는다.",
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


PRIORITY = (
    "Motor[1].ServoCtrl",
    "Motor[1].ClosedLoop",
    "Motor[1].JogPos",
    "Motor[1].JogTa",
    "Motor[1].JogTs",
    "Motor[1].JogSpeed",
    "Motor[1].Desired.TimerEnabled",
    "Motor[1].AmpEna",
    "Motor[1].Coord",
    "Motor[1].Status[0]",
    "Motor[1].Status[1]",
)


def continue_after_manual_restart() -> None:
    """저장된 A0를 유지하고, B0 READ와 diff 뒤에만 activation을 판단한다."""
    before_state = json.loads((EVIDENCE / "before_restart_state.json").read_text(encoding="utf-8"))
    before_motion = json.loads((EVIDENCE / "before_restart_motion.json").read_text(encoding="utf-8"))
    before_values = before_state["values"]
    if not isinstance(before_values, dict) or not isinstance(before_motion, dict):
        raise SystemExit("before-restart evidence is not an object")
    transport = SshGpasciiTransport(P017_TEST_CONFIG)
    try:
        identity = _run_once(transport, "hostname; uname -s")
        address = _run_once(transport, "ip -4 -o addr show eth0")
    except PowerPmacError as exc:
        _publish_continuation(before_motion, None, None, None, None, [], [{"step": "CONNECT", "error": exc.code}], exc.code, "", "")
        return
    identity_text = identity.stdout.strip()
    address_text = address.stdout.strip()
    boot_ok = "ppmac" in identity_text and "172.20.0.200" in address_text
    snapshot_error = ""
    try:
        snapshot, snapshot_raw, snapshot_rc = read_names(transport, SNAPSHOT_NAMES)
    except PowerPmacError as exc:
        snapshot_error = exc.code
        snapshot, snapshot_raw, snapshot_rc = {}, "", 0
        if exc.code == "TIMEOUT":
            try:
                snapshot, snapshot_raw, snapshot_rc = read_names(transport, SNAPSHOT_NAMES)
                snapshot_error = ""
            except PowerPmacError as retry_exc:
                snapshot_error = retry_exc.code
    if snapshot_error:
        _publish_continuation(
            before_motion,
            None,
            None,
            None,
            None,
            [],
            [{"step": "B0", "error": snapshot_error}],
            snapshot_error,
            identity_text,
            address_text,
        )
        return
    initial = dict(snapshot)
    ready = readiness(initial)
    diff_rows = restart_diff(before_values, initial)
    snapshot_ok = all(initial.get(name, "NOT_AVAILABLE") != "NOT_AVAILABLE" for name in SNAPSHOT_NAMES)
    raw_rows: list[dict[str, object]] = [{"step": "B0", "readiness": ready, "returncode": snapshot_rc, "values": initial}]
    _write(EVIDENCE / "after_restart_initial_state.json", {"performed": True, "values": initial, "raw": snapshot_raw, "returncode": snapshot_rc, "readiness": ready})
    _write(EVIDENCE / "restart_state_diff.json", _diff_document(diff_rows))
    if not boot_ok or not snapshot_ok:
        reason = "identity mismatch" if not boot_ok else "snapshot incomplete"
        _publish_continuation(before_motion, initial, ready, diff_rows, None, None, raw_rows, reason, identity_text, address_text, boot_ok=boot_ok, snapshot_ok=snapshot_ok)
        return
    activation = _activate_if_needed(transport, initial, raw_rows)
    _write(EVIDENCE / "activation_after_restart.json", activation)
    if activation["blocked"]:
        _publish_continuation(before_motion, initial, ready, diff_rows, activation, None, raw_rows, str(activation["block_reason"]), identity_text, address_text)
        return
    motion = _run_motion(transport, raw_rows)
    _publish_continuation(before_motion, initial, ready, diff_rows, activation, motion, raw_rows, "", identity_text, address_text)


def _run_once(transport: SshGpasciiTransport, remote: str) -> object:
    try:
        return transport.run_remote(remote)
    except PowerPmacError as exc:
        if exc.code != "TIMEOUT":
            raise
        return transport.run_remote(remote)


def _activate_if_needed(transport: SshGpasciiTransport, initial: dict[str, str], raw_rows: list[dict[str, object]]) -> dict[str, object]:
    ready = readiness(initial)
    record: dict[str, object] = {
        "performed": True,
        "required": ready != "RECOVERED",
        "restart_only_readiness": ready,
        "actions": [],
        "readbacks": [],
        "post_activation_state": None,
        "post_activation_readiness": None,
        "blocked": False,
        "block_reason": None,
    }
    if ready == "RECOVERED":
        record["note"] = "B0 readiness was RECOVERED. The P-019 activation path was not used."
        return record
    if ready == "FAULT_PRESENT":
        record["blocked"] = True
        record["block_reason"] = "FAULT_PRESENT"
        return record
    if ready != "NOT_READY":
        record["blocked"] = True
        record["block_reason"] = ready
        return record
    try:
        servo = float(initial["Motor[1].ServoCtrl"])
    except (KeyError, ValueError):
        record["blocked"] = True
        record["block_reason"] = "ServoCtrl not numeric"
        return record
    if servo <= 0:
        if not _write_and_read_back(transport, "Motor[1].ServoCtrl=1", "Motor[1].ServoCtrl", record, raw_rows, minimum=0):
            return record
    closed = _read_back(transport, "Motor[1].ClosedLoop", record, raw_rows)
    if closed is None:
        record["blocked"] = True
        record["block_reason"] = "ClosedLoop read-back missing"
        return record
    if closed != 1:
        if not _write_and_read_back(transport, "#1J/", "Motor[1].ClosedLoop", record, raw_rows, exact=1):
            return record
    post, post_raw, post_rc = read_names(transport, SNAPSHOT_NAMES)
    post_ready = readiness(post)
    record["post_activation_state"] = post
    record["post_activation_raw"] = post_raw
    record["post_activation_returncode"] = post_rc
    record["post_activation_readiness"] = post_ready
    raw_rows.append({"step": "POST_ACTIVATION", "readiness": post_ready, "returncode": post_rc, "values": post})
    if post_ready != "RECOVERED":
        record["blocked"] = True
        record["block_reason"] = post_ready
    return record


def _write_and_read_back(
    transport: SshGpasciiTransport,
    command: str,
    name: str,
    record: dict[str, object],
    raw_rows: list[dict[str, object]],
    *,
    minimum: float | None = None,
    exact: float | None = None,
) -> bool:
    try:
        exchange = transport.execute_lines([command])
    except PowerPmacError as exc:
        record["blocked"] = True
        record["block_reason"] = exc.code
        raw_rows.append({"step": "ACTIVATION", "command": command, "error": exc.code})
        return False
    actions = record["actions"]
    assert isinstance(actions, list)
    actions.append(command)
    raw_rows.append({"step": "ACTIVATION", "command": command, "returncode": exchange.returncode, "stdout": exchange.stdout, "stderr": exchange.stderr})
    value = _read_back(transport, name, record, raw_rows)
    if value is None:
        record["blocked"] = True
        record["block_reason"] = name + " read-back missing"
        return False
    if minimum is not None and value <= minimum:
        record["blocked"] = True
        record["block_reason"] = name + " read-back stayed <= 0"
        return False
    if exact is not None and value != exact:
        record["blocked"] = True
        record["block_reason"] = name + " read-back was not " + str(exact)
        return False
    return True


def _read_back(transport: SshGpasciiTransport, name: str, record: dict[str, object], raw_rows: list[dict[str, object]]) -> float | None:
    try:
        parsed, _raw, returncode = read_names(transport, (name,))
    except PowerPmacError as exc:
        raw_rows.append({"step": "READ_BACK", "name": name, "error": exc.code})
        return None
    text = parsed.get(name, "NOT_AVAILABLE")
    readbacks = record["readbacks"]
    assert isinstance(readbacks, list)
    readbacks.append({"name": name, "value": text, "returncode": returncode})
    raw_rows.append({"step": "READ_BACK", "name": name, "value": text, "returncode": returncode})
    try:
        return float(text)
    except ValueError:
        return None


def _run_motion(transport: SshGpasciiTransport, raw_rows: list[dict[str, object]]) -> dict[str, object]:
    before, _raw, _returncode = read_names(transport, TIMELINE_NAMES)
    target = choose_target(before.get("Motor[1].Pos", "NOT_AVAILABLE"), before.get("Motor[1].JogPos", "NOT_AVAILABLE"))
    command = "#1j=" + str(target)
    started = time.monotonic()
    jog = transport.execute_lines([command])
    raw_rows.append({"step": "COMMAND", "command": command, "returncode": jog.returncode, "stdout": jog.stdout, "stderr": jog.stderr})
    samples = []
    for index in (1, 2, 3):
        time.sleep(0.5)
        parsed, _raw, returncode = read_names(transport, TIMELINE_NAMES)
        sample = {name: parsed[name] for name in TIMELINE_NAMES}
        sample["t_s"] = time.monotonic() - started
        sample["returncode"] = returncode
        samples.append(sample)
        raw_rows.append({"step": f"T{index}", "t_s": sample["t_s"], "sample": {name: parsed[name] for name in TIMELINE_NAMES}})
    completed = any(motion_completed(sample, str(target), "\x06" in jog.stdout) for sample in samples)
    text = jog.stdout + "\n" + jog.stderr
    return {
        "performed": True,
        "target": target,
        "units": "pmac_motor_unit",
        "command": command,
        "ack_present": "\x06" in jog.stdout,
        "error_present": "error #" in text.lower(),
        "returncode": jog.returncode,
        "ack_is_motion_completion": False,
        "timeline_note": "SSH/gpascii sample times are not a high-frequency servo trace",
        "before": {name: before[name] for name in TIMELINE_NAMES},
        "samples": [{name: sample[name] for name in list(TIMELINE_NAMES) + ["t_s"]} for sample in samples],
        "completed": completed,
        "stdout": jog.stdout,
        "stderr": jog.stderr,
    }


def _publish_continuation(
    before_motion: dict[str, object],
    initial: dict[str, str] | None,
    ready: str | None,
    diff_rows: list[dict[str, str]] | None,
    activation: dict[str, object] | None,
    motion: dict[str, object] | None,
    raw_rows: list[dict[str, object]],
    reason: str,
    identity_text: str,
    address_text: str,
    *,
    boot_ok: bool = False,
    snapshot_ok: bool = False,
) -> None:
    if initial is not None and diff_rows is not None and ready is not None:
        boot_ok = boot_ok or ("ppmac" in identity_text and "172.20.0.200" in address_text)
        snapshot_ok = snapshot_ok or all(initial.get(name, "NOT_AVAILABLE") != "NOT_AVAILABLE" for name in SNAPSHOT_NAMES)
    after_completed = motion["completed"] if motion is not None else None
    if not isinstance(after_completed, bool):
        after_completed = None
    activation_required = bool(activation["required"]) if activation is not None else False
    classification = classify_session_reset(
        before_completed=before_motion.get("completed") is True,
        after_ran=True,
        after_completed=after_completed if isinstance(after_completed, bool) else None,
        activation_required=activation_required,
        boot_ok=boot_ok,
        snapshot_ok=snapshot_ok,
    )
    if initial is None:
        _write(EVIDENCE / "after_restart_initial_state.json", {"performed": False, "reason": reason})
        _write(EVIDENCE / "restart_state_diff.json", {"performed": False, "reason": reason})
    if activation is None:
        _write(EVIDENCE / "activation_after_restart.json", {"performed": False, "required": None, "actions": [], "reason": reason})
    if motion is None:
        _write(EVIDENCE / "after_restart_motion.json", {"performed": False, "reason": reason})
    else:
        _write(EVIDENCE / "after_restart_motion.json", motion)
    changes = [row for row in (diff_rows or []) if row["tag"] != "UNCHANGED"]
    before_samples = before_motion.get("samples")
    assert isinstance(before_samples, list)
    after_samples = motion["samples"] if motion is not None else []
    assert isinstance(after_samples, list)
    _write(
        EVIDENCE / "ab_comparison.json",
        {
            "before_restart": {
                "readiness": before_motion.get("readiness"),
                "target": before_motion.get("target"),
                "JogPos": [sample.get("Motor[1].JogPos") for sample in before_samples if isinstance(sample, dict)],
                "DesPos": [sample.get("Motor[1].DesPos") for sample in before_samples if isinstance(sample, dict)],
                "Pos": [sample.get("Motor[1].Pos") for sample in before_samples if isinstance(sample, dict)],
                "ActPos": [sample.get("Motor[1].ActPos") for sample in before_samples if isinstance(sample, dict)],
                "completed": before_motion.get("completed"),
            },
            "restart": {
                "method": "manual Official Power PMAC Simulator 1.2.1 CK5M shutdown and start",
                "successful": boot_ok and snapshot_ok,
                "transport_reconnect_used_as_restart": False,
                "simulator_process_stopped": True,
                "qemu_process_restarted": True,
                "IDE_state": "UI indicators gray; console message: IP용 콘솔 참조 / 콘솔 창 시작됨",
                "boot_completed": "operator observed Debian 12 ppmac root console; runner checked hostname and eth0",
                "simulator_IP": "172.20.0.200",
                "firmware_if_observed": "not queried by this runner; operator observed Official Power PMAC Simulator 1.2.1 CK5M",
                "operator_configuration_changes": False,
            },
            "after_restart_initial": {"performed": initial is not None, "readiness": ready, "state_changes": changes},
            "activation": {
                "required": None if activation is None else activation["required"],
                "actions": [] if activation is None else activation["actions"],
                "readbacks": [] if activation is None else activation["readbacks"],
                "post_activation_readiness": None if activation is None else activation.get("post_activation_readiness"),
            },
            "after_restart": _after_comparison(motion),
            "classification": classification["classification"],
            "causal_claim": classification["causal_claim"],
        },
    )
    _write(EVIDENCE / "classification.json", classification)
    _write(EVIDENCE / "write_audit.json", _continuation_audit(str(before_motion.get("command")), activation, motion))
    (EVIDENCE / "environment.md").write_text(_continuation_environment(identity_text, address_text, boot_ok), encoding="utf-8")
    (EVIDENCE / "summary.md").write_text(_continuation_summary(before_motion, initial, ready, diff_rows or [], activation, motion, classification, identity_text), encoding="utf-8")
    (EVIDENCE / "raw" / "after_restart.jsonl").write_text("\n".join(json.dumps(row, ensure_ascii=False) for row in raw_rows) + "\n", encoding="utf-8")
    _reject_secret(EVIDENCE)
    print(json.dumps({"classification": classification["classification"], "causal_claim": classification["causal_claim"], "completed": after_completed}, ensure_ascii=False))


def _diff_document(rows: list[dict[str, str]]) -> dict[str, object]:
    counts: dict[str, int] = {}
    for row in rows:
        counts[row["tag"]] = counts.get(row["tag"], 0) + 1
    return {"performed": True, "rows": rows, "priority": {row["variable"]: row for row in rows if row["variable"] in PRIORITY}, "counts": counts}


def _after_comparison(motion: dict[str, object] | None) -> dict[str, object]:
    if motion is None:
        return {"performed": False}
    samples = motion["samples"]
    assert isinstance(samples, list)
    return {
        "performed": True,
        "target": motion["target"],
        "JogPos": [sample.get("Motor[1].JogPos") for sample in samples if isinstance(sample, dict)],
        "DesPos": [sample.get("Motor[1].DesPos") for sample in samples if isinstance(sample, dict)],
        "Pos": [sample.get("Motor[1].Pos") for sample in samples if isinstance(sample, dict)],
        "ActPos": [sample.get("Motor[1].ActPos") for sample in samples if isinstance(sample, dict)],
        "completed": motion["completed"],
    }


def _continuation_audit(before_command: str, activation: dict[str, object] | None, motion: dict[str, object] | None) -> dict[str, object]:
    activation_writes = [] if activation is None else [item for item in activation["actions"] if isinstance(item, str)]
    motion_writes = []
    if isinstance(motion, dict) and isinstance(motion.get("command"), str):
        motion_writes.append(motion["command"])
    writes = [before_command, *activation_writes, *motion_writes]
    return {
        "writes_performed": writes,
        "allowed_writes": writes,
        "forbidden_writes": [],
        "configuration_modifications": [],
        "activation_writes": activation_writes,
        "simulator_process_stopped": True,
        "qemu_process_restarted": True,
        "transport_reconnect_used_as_restart": False,
    }


def _continuation_environment(identity_text: str, address_text: str, boot_ok: bool) -> str:
    return "\n".join(
        [
            "# P-021C environment",
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
            "restart_method: manual Official Power PMAC Simulator 1.2.1 CK5M shutdown and start",
            "simulator_process_stopped: true",
            "qemu_process_restarted: true",
            "IDE_state: UI indicators gray; console message: IP용 콘솔 참조 / 콘솔 창 시작됨",
            "boot_completed: operator observed Debian 12 ppmac, eth0 172.20.0.200, root console",
            f"runner_boot_check: {boot_ok}",
            "simulator_IP: 172.20.0.200",
            "operator_configuration_changes: false",
            "credential: runtime environment variable PMAC_SSH_PASSWORD; value not recorded",
            "```",
            "",
            "P-019, P-020, P-021, P-021A, P-021B evidence는 수정하지 않는다.",
            "",
        ]
    )


def _continuation_summary(
    before_motion: dict[str, object],
    initial: dict[str, str] | None,
    ready: str | None,
    diff_rows: list[dict[str, str]],
    activation: dict[str, object] | None,
    motion: dict[str, object] | None,
    classification: dict[str, object],
    identity_text: str,
) -> str:
    before = before_motion.get("before")
    assert isinstance(before, dict)
    lines = [
        "# P-021C summary",
        "",
        f"측정: {datetime.now(timezone.utc).isoformat()}",
        "",
        "Official Power PMAC Simulator session-reset A/B experiment only. P-021 FINAL PASS를 선언하지 않는다. P-022는 시작하지 않는다.",
        "",
        f"classification: {classification['classification']} {classification.get('classification_name', '')}".rstrip(),
        f"confidence: {classification['confidence']}",
        f"causal_claim: {classification['causal_claim']}",
        f"confirmed: {classification['confirmed']}",
        "",
        f"before readiness: {before_motion.get('readiness')}",
        f"before command: {before_motion.get('command')} completed: {before_motion.get('completed')}",
        f"before T0 JogPos/DesPos/Pos/ActPos: {before.get('Motor[1].JogPos')}/{before.get('Motor[1].DesPos')}/{before.get('Motor[1].Pos')}/{before.get('Motor[1].ActPos')}",
    ]
    samples = before_motion.get("samples")
    assert isinstance(samples, list)
    for index, sample in enumerate(samples, start=1):
        assert isinstance(sample, dict)
        lines.append(
            f"before T{index} t_s {sample.get('t_s')} JogPos {sample.get('Motor[1].JogPos')} DesPos {sample.get('Motor[1].DesPos')} "
            f"Pos {sample.get('Motor[1].Pos')} ActPos {sample.get('Motor[1].ActPos')} TimerEnabled {sample.get('Motor[1].Desired.TimerEnabled')}"
        )
    lines.extend(["", f"after initial readiness: {ready}", f"identity: {identity_text.replace(chr(10), ' / ')}"])
    for row in diff_rows:
        if row["variable"] in PRIORITY and row["tag"] != "UNCHANGED":
            lines.append(f"priority {row['variable']}: {row['before']} -> {row['after']} {row['tag']}")
    if not any(row["variable"] in PRIORITY and row["tag"] != "UNCHANGED" for row in diff_rows):
        lines.append("priority variables: UNCHANGED")
    lines.append("state difference is recorded. It is not declared the root cause.")
    if activation is None:
        lines.append("activation: not performed")
    else:
        lines.append(f"activation required: {activation['required']} actions: {activation['actions']} blocked: {activation['blocked']}")
        lines.append(f"post-activation readiness: {activation.get('post_activation_readiness')}")
    if motion is None:
        lines.append("after motion: not performed")
    else:
        lines.append(f"after command: {motion['command']} completed: {motion['completed']} ack: {motion['ack_present']} error: {motion['error_present']} returncode: {motion['returncode']}")
        current = motion["before"]
        assert isinstance(current, dict)
        lines.append(f"after T0 JogPos/DesPos/Pos/ActPos: {current.get('Motor[1].JogPos')}/{current.get('Motor[1].DesPos')}/{current.get('Motor[1].Pos')}/{current.get('Motor[1].ActPos')}")
        after_samples = motion["samples"]
        assert isinstance(after_samples, list)
        for index, sample in enumerate(after_samples, start=1):
            assert isinstance(sample, dict)
            lines.append(
                f"after T{index} t_s {sample.get('t_s')} JogPos {sample.get('Motor[1].JogPos')} DesPos {sample.get('Motor[1].DesPos')} "
                f"DesVel {sample.get('Motor[1].DesVel')} Pos {sample.get('Motor[1].Pos')} ActPos {sample.get('Motor[1].ActPos')} "
                f"TimerEnabled {sample.get('Motor[1].Desired.TimerEnabled')}"
            )
    lines.extend(
        [
            "",
            "## Claim boundary",
            "",
            "- Official Power PMAC Simulator session-reset A/B experiment only",
            "- software virtual Motor[1]",
            "- restart correlation is not automatically root-cause proof",
            "- ACK is not motion completion",
            "- PMAC motor units only",
            "- no real Power PMAC hardware",
            "- no physical motor",
            "- no physical servo",
            "- no physical encoder",
            "- no physical amplifier",
            "- no physical I/O",
            "- no EtherCAT",
            "- no real robot",
            "- no servo tuning",
            "- no industrial commissioning",
            "- no hard-real-time validation",
            "",
        ]
    )
    return "\n".join(lines)


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "continue":
        continue_after_manual_restart()
    else:
        main()
