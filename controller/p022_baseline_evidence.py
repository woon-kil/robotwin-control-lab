"""P-022 clean-restart baseline probe.

S0 파일을 쓰기 전에는 motor command를 보내지 않는다.
Simulator/QEMU restart는 이 runner가 하지 않는다.
"""

from __future__ import annotations

import json
import os
import time
from datetime import datetime, timezone
from pathlib import Path

from controller.p021a_investigation import parse_snapshot
from controller.p022_baseline import (
    FAULT_NAMES,
    MOTION_NAMES,
    SNAPSHOT_NAMES,
    activation_decision,
    classify_baseline,
    faults_clear,
    gpascii_ok,
    motion_command,
    motion_completed,
    readback_ok,
    ready_state,
    snapshot_complete,
    transport_ok,
)
from controller.power_pmac_config import P017_TEST_CONFIG
from controller.power_pmac_transport import PowerPmacError, SshGpasciiTransport

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "evidence" / "P-022" / "environment-recovery"
IDENTITY_REMOTE = "hostname; echo __P022_UNAME__; uname -s; echo __P022_ETH__; ip -4 -o addr show eth0"
READBACK_NAME = {
    "Motor[1].ServoCtrl=1": "Motor[1].ServoCtrl",
    "#1J/": "Motor[1].ClosedLoop",
}


def main() -> None:
    transport = SshGpasciiTransport(P017_TEST_CONFIG)
    state = _empty_state()
    try:
        _run(transport, state)
    except PowerPmacError as exc:
        state["stop_error"] = exc.code
        if state["phase"] == "transport":
            state["transport_passed"] = False
        elif state["phase"] == "gpascii":
            state["gpascii_passed"] = False
        elif state["phase"] == "activation":
            state["activation_readback_ok"] = False
        elif state["phase"] == "motion":
            state["motion_completed_value"] = False
    _publish(state)
    print(json.dumps({"classification": state["classification"]["classification"]}, ensure_ascii=False))


def _run(transport: SshGpasciiTransport, state: dict[str, object]) -> None:
    state["phase"] = "transport"
    identity = transport.run_remote(IDENTITY_REMOTE)
    state["identity_stdout"] = identity.stdout
    state["identity_stderr"] = identity.stderr
    state["identity_returncode"] = identity.returncode
    hostname, uname, eth0 = _parse_identity(identity.stdout)
    state["hostname"] = hostname
    state["uname"] = uname
    state["eth0"] = eth0
    state["transport_passed"] = transport_ok(hostname, uname, eth0) and identity.returncode == 0
    state["raw_rows"].append({"step": "SSH", "hostname": hostname, "uname": uname, "eth0": eth0, "returncode": identity.returncode})
    if not state["transport_passed"]:
        return

    state["phase"] = "gpascii"
    servo = transport.execute_lines(["Sys.ServoCount"])
    parsed_count = parse_snapshot(servo.stdout, ("Sys.ServoCount",))["Sys.ServoCount"]
    state["servo_count"] = parsed_count
    state["servo_stdout"] = servo.stdout
    state["servo_stderr"] = servo.stderr
    state["servo_returncode"] = servo.returncode
    timed_out = "timed out" in (servo.stderr or "").lower()
    state["gpascii_passed"] = gpascii_ok(parsed_count) and not timed_out
    state["raw_rows"].append({"step": "GPASCII", "Sys.ServoCount": parsed_count, "returncode": servo.returncode})
    if not state["gpascii_passed"]:
        return

    state["phase"] = "snapshot"
    snapshot, raw, returncode = _read_names(transport, SNAPSHOT_NAMES)
    state["s0"] = snapshot
    state["s0_raw"] = raw
    state["s0_returncode"] = returncode
    state["snapshot_recorded"] = True
    state["snapshot_complete"] = snapshot_complete(snapshot)
    state["fault_present"] = not faults_clear(snapshot) if state["snapshot_complete"] else False
    state["raw_rows"].append({"step": "CLEAN_RESTART_S0", "returncode": returncode, "values": snapshot})
    _write(EVIDENCE / "clean_restart_s0.json", _s0_document(state))
    state["s0_persisted"] = True
    if not state["snapshot_complete"]:
        return

    decision = activation_decision(snapshot)
    if decision["status"] == "FAULT":
        state["fault_present"] = True
        return
    if decision["status"] in {"INCOMPLETE", "UNEXPECTED_SERVO"}:
        state["activation_readback_ok"] = False
        state["activation_required"] = decision["status"] != "INCOMPLETE"
        return

    state["phase"] = "activation"
    current = snapshot
    for _step in (1, 2):
        decision = activation_decision(current)
        if decision["status"] == "READY":
            state["ready_snapshot"] = current
            state["ready"] = True
            break
        if decision["status"] == "FAULT":
            state["fault_present"] = True
            return
        command = decision["command"]
        if not isinstance(command, str) or not state["s0_persisted"]:
            state["activation_readback_ok"] = False
            return
        state["activation_required"] = True
        state["allowed_writes"].append(command)
        if not _activate_once(transport, state, command):
            return
        current = state["ready_snapshot"]
        assert isinstance(current, dict)
    if not state["ready"]:
        state["activation_readback_ok"] = False
        return

    if not state["ready"] or not isinstance(state["ready_snapshot"], dict):
        state["activation_readback_ok"] = False
        return

    state["phase"] = "motion"
    command = motion_command(state["ready_snapshot"], ready=True)
    if command is None:
        state["motion_completed_value"] = None
        return
    state["allowed_writes"].append(command)
    _probe(transport, state, command)


def _activate_once(transport: SshGpasciiTransport, state: dict[str, object], command: str) -> bool:
    if not state["s0_persisted"]:
        state["writes_before_snapshot"] = True
        return False
    performed = state["writes_performed"]
    assert isinstance(performed, list)
    performed.append(command)
    exchange = transport.execute_lines([command])
    actions = state["activation_actions"]
    assert isinstance(actions, list)
    actions.append(command)
    rows = state["raw_rows"]
    assert isinstance(rows, list)
    rows.append({"step": "ACTIVATION", "command": command, "returncode": exchange.returncode, "stdout": exchange.stdout, "stderr": exchange.stderr})
    name = READBACK_NAME[command]
    parsed, _raw, returncode = _read_names(transport, (name,))
    value = parsed.get(name, "NOT_AVAILABLE")
    readbacks = state["readbacks"]
    assert isinstance(readbacks, list)
    readbacks.append({"command": command, "name": name, "value": value, "returncode": returncode})
    if command == "Motor[1].ServoCtrl=1":
        state["servo_write"] = True
        state["servo_readback"] = value
    if command == "#1J/":
        state["close_command"] = True
        state["closed_readback"] = value
    if not readback_ok(command, value):
        state["activation_readback_ok"] = False
        return False
    fresh, fresh_raw, fresh_rc = _read_names(transport, SNAPSHOT_NAMES)
    state["ready_snapshot"] = fresh
    state["ready_raw"] = fresh_raw
    state["ready_returncode"] = fresh_rc
    state["ready"] = ready_state(fresh)
    rows.append({"step": "READY_READ", "returncode": fresh_rc, "values": fresh, "ready": state["ready"]})
    if command == "#1J/" and not state["ready"]:
        state["activation_readback_ok"] = False
        return False
    return True


def _probe(transport: SshGpasciiTransport, state: dict[str, object], command: str) -> None:
    if not state["s0_persisted"]:
        state["writes_before_snapshot"] = True
        return
    before, _raw, _returncode = _read_names(transport, MOTION_NAMES)
    performed = state["writes_performed"]
    assert isinstance(performed, list)
    performed.append(command)
    started = time.monotonic()
    jog = transport.execute_lines([command])
    rows = state["raw_rows"]
    assert isinstance(rows, list)
    rows.append({"step": "COMMAND", "command": command, "returncode": jog.returncode, "stdout": jog.stdout, "stderr": jog.stderr})
    samples = []
    for index in (1, 2, 3):
        time.sleep(0.5)
        parsed, _raw, returncode = _read_names(transport, MOTION_NAMES)
        sample = {name: parsed[name] for name in MOTION_NAMES}
        sample["t_s"] = time.monotonic() - started
        sample["returncode"] = returncode
        samples.append(sample)
        rows.append({"step": f"T{index}", "t_s": sample["t_s"], "sample": sample})
    faults, _raw, fault_rc = _read_names(transport, FAULT_NAMES)
    rows.append({"step": "FAULTS", "returncode": fault_rc, "values": faults})
    last = samples[-1]
    text = jog.stdout + "\n" + jog.stderr
    positions_ok = motion_completed(last, command.split("=", 1)[1], "\x06" in jog.stdout)
    state["motion"] = {
        "performed": True,
        "target": int(command.split("=", 1)[1]),
        "units": "pmac_motor_unit",
        "command": command,
        "ack_present": "\x06" in jog.stdout,
        "error_present": "error #" in text.lower(),
        "returncode": jog.returncode,
        "ack_is_motion_completion": False,
        "timeline_note": "SSH/gpascii sample times are not a high-frequency servo trace",
        "before": before,
        "samples": samples,
        "faults": faults,
        "completed": positions_ok and faults_clear(faults) and "error #" not in text.lower(),
    }
    state["motion_completed_value"] = state["motion"]["completed"]


def _read_names(transport: SshGpasciiTransport, names: tuple[str, ...]) -> tuple[dict[str, str], str, int]:
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


def _parse_identity(stdout: str) -> tuple[str, str, str]:
    host, _, rest = stdout.partition("__P022_UNAME__")
    uname, _, eth = rest.partition("__P022_ETH__")
    return host.strip(), uname.strip(), eth.strip()


def _empty_state() -> dict[str, object]:
    return {
        "phase": "transport",
        "transport_passed": False,
        "gpascii_passed": False,
        "hostname": "",
        "uname": "",
        "eth0": "",
        "identity_stdout": "",
        "identity_stderr": "",
        "identity_returncode": None,
        "servo_count": "NOT_AVAILABLE",
        "servo_stdout": "",
        "servo_stderr": "",
        "servo_returncode": None,
        "s0": None,
        "s0_raw": "",
        "s0_returncode": None,
        "s0_persisted": False,
        "snapshot_recorded": False,
        "snapshot_complete": False,
        "fault_present": False,
        "activation_required": False,
        "activation_readback_ok": True,
        "activation_actions": [],
        "readbacks": [],
        "servo_write": False,
        "servo_readback": None,
        "close_command": False,
        "closed_readback": None,
        "ready": False,
        "ready_snapshot": None,
        "ready_raw": "",
        "ready_returncode": None,
        "motion": None,
        "motion_completed_value": None,
        "writes_before_snapshot": False,
        "writes_performed": [],
        "allowed_writes": [],
        "unexpected_write": False,
        "stop_error": None,
        "raw_rows": [],
        "classification": {},
    }


def _publish(state: dict[str, object]) -> None:
    performed = state["writes_performed"]
    allowed = state["allowed_writes"]
    assert isinstance(performed, list) and isinstance(allowed, list)
    forbidden = [item for item in performed if item not in allowed]
    if state["writes_before_snapshot"] or forbidden:
        state["unexpected_write"] = True
    motion = state["motion"]
    motion_value = state["motion_completed_value"]
    if not isinstance(motion_value, bool):
        motion_value = None
    classification = classify_baseline(
        transport_ok=bool(state["transport_passed"]),
        gpascii_ok=bool(state["gpascii_passed"]),
        snapshot_recorded=bool(state["snapshot_recorded"]),
        snapshot_complete=bool(state["snapshot_complete"]),
        writes_before_snapshot=bool(state["writes_before_snapshot"]),
        fault_present=bool(state["fault_present"]),
        activation_required=bool(state["activation_required"]),
        activation_readback_ok=bool(state["activation_readback_ok"]),
        ready=bool(state["ready"]),
        motion_completed=motion_value,
        unexpected_write=bool(state["unexpected_write"]),
    )
    if state["stop_error"]:
        classification["stop_error"] = state["stop_error"]
    state["classification"] = classification
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    (EVIDENCE / "raw").mkdir(exist_ok=True)
    if not (EVIDENCE / "clean_restart_s0.json").exists():
        _write(EVIDENCE / "clean_restart_s0.json", _s0_document(state))
    _write(EVIDENCE / "transport_health.json", _transport_document(state))
    _write(EVIDENCE / "gpascii_health.json", _gpascii_document(state))
    _write(EVIDENCE / "activation.json", _activation_document(state))
    _write(EVIDENCE / "ready_s1.json", _ready_document(state))
    _write(EVIDENCE / "motion_probe.json", motion if isinstance(motion, dict) else {"performed": False})
    _write(
        EVIDENCE / "write_audit.json",
        {
            "writes_before_snapshot": ["recorded"] if state["writes_before_snapshot"] else [],
            "writes_performed": performed,
            "allowed_writes": allowed,
            "forbidden_writes": forbidden,
            "configuration_writes": [],
            "p100_write": False,
        },
    )
    _write(EVIDENCE / "classification.json", classification)
    rows = state["raw_rows"]
    assert isinstance(rows, list)
    (EVIDENCE / "raw" / "probe.jsonl").write_text("".join(json.dumps(row, ensure_ascii=False) + "\n" for row in rows), encoding="utf-8")
    (EVIDENCE / "summary.md").write_text(_summary(state, classification), encoding="utf-8")
    _reject_secret(EVIDENCE)


def _s0_document(state: dict[str, object]) -> dict[str, object]:
    if not state["snapshot_recorded"]:
        return {"performed": False, "label": "CLEAN_RESTART_S0"}
    return {
        "performed": True,
        "label": "CLEAN_RESTART_S0",
        "values": state["s0"],
        "raw": state["s0_raw"],
        "returncode": state["s0_returncode"],
        "activation_performed_before_snapshot": False,
    }


def _transport_document(state: dict[str, object]) -> dict[str, object]:
    return {
        "ssh": "PASS" if state["transport_passed"] else "FAIL",
        "hostname": state["hostname"],
        "os": state["uname"],
        "eth0": state["eth0"],
        "returncode": state["identity_returncode"],
        "stderr": state["identity_stderr"],
        "path": "MacBook -> 192.168.0.146:22022 -> Windows portproxy -> 172.20.0.200:22",
    }


def _gpascii_document(state: dict[str, object]) -> dict[str, object]:
    if state["servo_returncode"] is None and not state["transport_passed"]:
        return {"performed": False}
    return {
        "performed": state["servo_returncode"] is not None,
        "variable": "Sys.ServoCount",
        "value": state["servo_count"],
        "numeric": gpascii_ok(str(state["servo_count"])),
        "timeout": "timed out" in str(state["servo_stderr"]).lower(),
        "protocol_error": not gpascii_ok(str(state["servo_count"])),
        "returncode": state["servo_returncode"],
        "stdout": state["servo_stdout"],
        "stderr": state["servo_stderr"],
        "servo_state_claim": False,
    }


def _activation_document(state: dict[str, object]) -> dict[str, object]:
    actions = state["activation_actions"]
    if not isinstance(actions, list) or not actions:
        return {
            "performed": False,
            "required": state["activation_required"],
            "actions": [],
            "readbacks": state["readbacks"],
        }
    return {
        "performed": True,
        "required": True,
        "actions": actions,
        "readbacks": state["readbacks"],
        "servo_write": state["servo_write"],
        "servo_readback": state["servo_readback"],
        "close_command": state["close_command"],
        "closed_loop_readback": state["closed_readback"],
    }


def _ready_document(state: dict[str, object]) -> dict[str, object]:
    snapshot = state["ready_snapshot"]
    if not state["ready"] or not isinstance(snapshot, dict):
        return {"performed": False, "label": "CLEAN_RESTART_READY_S1", "ready": False}
    return {
        "performed": True,
        "label": "CLEAN_RESTART_READY_S1",
        "ready": True,
        "values": {name: snapshot.get(name) for name in ("Motor[1].ServoCtrl", "Motor[1].ClosedLoop", "Motor[1].AmpEna", *FAULT_NAMES)},
        "raw": state["ready_raw"],
        "returncode": state["ready_returncode"],
    }


def _summary(state: dict[str, object], classification: dict[str, object]) -> str:
    snapshot = state["s0"] if isinstance(state["s0"], dict) else {}
    ready = state["ready_snapshot"] if isinstance(state["ready_snapshot"], dict) else {}
    motion = state["motion"] if isinstance(state["motion"], dict) else {}
    code = classification["classification"]
    if code == "BR-1":
        gate = "P-022 ENVIRONMENT RECOVERY PASS. P-022 main integration may resume. P-022 FINAL PASS는 선언하지 않는다."
    else:
        gate = "P-022 PAUSED"
    lines = [
        "# P-022 environment recovery",
        "",
        f"측정: {datetime.now(timezone.utc).isoformat()}",
        "",
        f"classification: {code} {classification['classification_name']}",
        f"confidence: {classification['confidence']}",
        f"confirmed: {classification['confirmed']}",
        f"p022_final_pass_declared: {classification['p022_final_pass_declared']}",
        "",
        "## Incident",
        "",
        "Before clean restart:",
        "",
        'QEMU repeatedly displayed: "Xenomai: suspending kernel thread ... (\'rtitask\')"',
        "",
        "Existing SSH path became unreachable.",
        "",
        "P-022 live execution was stopped.",
        "",
        "No attempt was made to classify the failure as a RoboTwin, adapter, motion, or Digital Twin defect.",
        "",
        "The Simulator was cleanly restarted manually.",
        "",
        "Post-restart baseline was then revalidated independently.",
        "",
        "incident: SIMULATOR_RUNTIME_FAILURE",
        "root_cause: NOT_CONFIRMED",
        "",
        "## Transport",
        "",
        f"SSH: {'PASS' if state['transport_passed'] else 'FAIL'}",
        f"hostname: {state['hostname']}",
        f"OS: {state['uname']}",
        f"eth0: {state['eth0']}",
        f"gpascii: {'PASS' if state['gpascii_passed'] else 'FAIL'}",
        f"Sys.ServoCount: {state['servo_count']}",
        "",
        "## CLEAN_RESTART_S0",
        "",
        f"ServoCtrl: {snapshot.get('Motor[1].ServoCtrl', 'NOT_AVAILABLE')}",
        f"ClosedLoop: {snapshot.get('Motor[1].ClosedLoop', 'NOT_AVAILABLE')}",
        f"AmpEna: {snapshot.get('Motor[1].AmpEna', 'NOT_AVAILABLE')}",
        f"JogPos: {snapshot.get('Motor[1].JogPos', 'NOT_AVAILABLE')}",
        f"DesPos: {snapshot.get('Motor[1].DesPos', 'NOT_AVAILABLE')}",
        f"Pos: {snapshot.get('Motor[1].Pos', 'NOT_AVAILABLE')}",
        f"ActPos: {snapshot.get('Motor[1].ActPos', 'NOT_AVAILABLE')}",
        f"FeFatal: {snapshot.get('Motor[1].FeFatal', 'NOT_AVAILABLE')}",
        f"AmpFault: {snapshot.get('Motor[1].AmpFault', 'NOT_AVAILABLE')}",
        f"LimitStop: {snapshot.get('Motor[1].LimitStop', 'NOT_AVAILABLE')}",
        f"Status0: {snapshot.get('Motor[1].Status[0]', 'NOT_AVAILABLE')}",
        f"Status1: {snapshot.get('Motor[1].Status[1]', 'NOT_AVAILABLE')}",
        "",
        "## Activation",
        "",
        f"required: {state['activation_required']}",
        f"ServoCtrl write: {state['servo_write']}",
        f"ServoCtrl read-back: {state['servo_readback']}",
        f"#1J/: {state['close_command']}",
        f"ClosedLoop read-back: {state['closed_readback']}",
        "",
        "## READY_S1",
        "",
        f"ready: {state['ready']}",
        f"ServoCtrl: {ready.get('Motor[1].ServoCtrl', 'NOT_AVAILABLE')}",
        f"ClosedLoop: {ready.get('Motor[1].ClosedLoop', 'NOT_AVAILABLE')}",
        f"AmpEna: {ready.get('Motor[1].AmpEna', 'NOT_AVAILABLE')}",
        f"FeFatal: {ready.get('Motor[1].FeFatal', 'NOT_AVAILABLE')}",
        f"AmpFault: {ready.get('Motor[1].AmpFault', 'NOT_AVAILABLE')}",
        f"LimitStop: {ready.get('Motor[1].LimitStop', 'NOT_AVAILABLE')}",
        "",
        "## Motion probe",
        "",
        f"target: {motion.get('target', 'NOT_RUN')}",
        f"command: {motion.get('command', 'NOT_RUN')}",
    ]
    before = motion.get("before")
    if isinstance(before, dict):
        lines.append(
            "T0 JogPos/DesPos/Pos/ActPos: "
            f"{before.get('Motor[1].JogPos')}/{before.get('Motor[1].DesPos')}/{before.get('Motor[1].Pos')}/{before.get('Motor[1].ActPos')}"
        )
    samples = motion.get("samples")
    if isinstance(samples, list):
        for index, sample in enumerate(samples, start=1):
            if isinstance(sample, dict):
                lines.append(
                    f"T{index} t_s {sample.get('t_s')} JogPos/DesPos/Pos/ActPos: "
                    f"{sample.get('Motor[1].JogPos')}/{sample.get('Motor[1].DesPos')}/{sample.get('Motor[1].Pos')}/{sample.get('Motor[1].ActPos')}"
                )
    faults = motion.get("faults")
    if isinstance(faults, dict):
        lines.append(
            "faults after T3: "
            f"FeFatal {faults.get('Motor[1].FeFatal')}, AmpFault {faults.get('Motor[1].AmpFault')}, LimitStop {faults.get('Motor[1].LimitStop')}"
        )
    if motion:
        lines.append(f"ack: {motion.get('ack_present')}")
        lines.append(f"error: {motion.get('error_present')}")
        lines.append(f"returncode: {motion.get('returncode')}")
    lines.extend(
        [
        f"completed: {motion.get('completed', False)}",
        "",
        "## Claim boundary",
        "",
        "- Official Power PMAC Simulator 1.2.1 only",
        "- software virtual Motor[1]",
        "- PMAC motor units",
        "- ActPos is simulator virtual feedback",
        "- ACK is not motion completion",
        "- Sys.ServoCount is not a servo-state claim",
        "- Xenomai message root cause is NOT_CONFIRMED",
        "- no real Power PMAC hardware",
        "- no physical motor, encoder, amplifier, I/O, or EtherCAT",
        "- no real robot",
        "- no hard-real-time validation",
        "- no industrial safety claim",
        "",
        f"Gate: {gate}",
        "P-023 started: NO",
        "",
        ]
    )
    return "\n".join(lines)


def _write(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
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
