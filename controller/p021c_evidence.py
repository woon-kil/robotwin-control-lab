"""P-021C before-restart measurement.

Simulator/QEMU process restart는 이 runner가 하지 않는다.
transport reconnect를 restart로 기록하지 않는다.
"""

from __future__ import annotations

import json
import os
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


if __name__ == "__main__":
    main()
