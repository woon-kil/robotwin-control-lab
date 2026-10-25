"""P-021 final live recovery retest.

Simulator process는 멈추지 않는다.
fresh read가 RECOVERED이면 #1J/를 보내지 않는다.
ACK는 motion completion이 아니다.
"""

from __future__ import annotations

import json
import os
import re
import time
from datetime import datetime, timezone
from pathlib import Path

from controller.p021_final_investigation import (
    NOT_AVAILABLE,
    choose_blocked_target,
    choose_new_target,
    choose_precondition_target,
    classify_final_lifecycle,
    is_rb1,
    motion_completed,
    readiness_of,
    values_equal,
)
from controller.p021_recovery import (
    recovery_leaves_validator_closed,
    scripted_fault_blocks,
    scripted_fault_cleared_but_not_ready,
    scripted_fault_recovery,
    scripted_fault_remains,
)
from controller.p021a_investigation import parse_snapshot
from controller.power_pmac_adapter import PowerPmacControllerAdapter
from controller.power_pmac_config import P017_TEST_CONFIG
from controller.power_pmac_transport import GpasciiExchange, PowerPmacError, SshGpasciiTransport

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "evidence" / "P-021-FINAL"
STATE_NAMES = (
    "Motor[1].ServoCtrl",
    "Motor[1].ClosedLoop",
    "Motor[1].AmpEna",
    "Motor[1].JogPos",
    "Motor[1].DesPos",
    "Motor[1].Pos",
    "Motor[1].ActPos",
    "Motor[1].DesVel",
    "Motor[1].DesVelZero",
    "Motor[1].Desired.Pos",
    "Motor[1].Desired.Vel",
    "Motor[1].Desired.TimerEnabled",
    "Motor[1].InPos",
    "Motor[1].FeFatal",
    "Motor[1].FeWarn",
    "Motor[1].AmpFault",
    "Motor[1].LimitStop",
    "Motor[1].Status[0]",
    "Motor[1].Status[1]",
)
TIMELINE_NAMES = (
    "Motor[1].JogPos",
    "Motor[1].DesPos",
    "Motor[1].DesVel",
    "Motor[1].Desired.Pos",
    "Motor[1].Desired.Vel",
    "Motor[1].Desired.TimerEnabled",
    "Motor[1].Pos",
    "Motor[1].ActPos",
    "Motor[1].ActVel",
    "Motor[1].InPos",
)
_JOG_RE = re.compile(r"#1j=[0-9]+")
_ALLOWED = {"Motor[1].ServoCtrl=1", "#1J/"}


class CountingTransport:
    def __init__(self, inner: SshGpasciiTransport) -> None:
        self.inner = inner
        self.calls: list[tuple[str, object]] = []

    def read_variable(self, name: str, timeout_s: float | None = None) -> str:
        self.calls.append(("read", name))
        return self.inner.read_variable(name, timeout_s)

    def write_and_read(self, name: str, value: int, timeout_s: float | None = None) -> str:
        self.calls.append(("write", f"{name}={value}"))
        return self.inner.write_and_read(name, value, timeout_s)

    def close(self) -> None:
        self.calls.append(("close", None))
        self.inner.close()

    def execute_lines(self, lines: list[str], timeout_s: float | None = None) -> GpasciiExchange:
        self.calls.append(("execute", list(lines)))
        return self.inner.execute_lines(lines, timeout_s)

    def run_remote(self, remote: str, timeout_s: float | None = None) -> GpasciiExchange:
        self.calls.append(("remote", remote))
        return self.inner.run_remote(remote, timeout_s)


def read_names(transport: CountingTransport, names: tuple[str, ...]) -> tuple[dict[str, str], str, int]:
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
    ctx: dict[str, object] = {
        "commands": [],
        "raw_rows": [],
        "evidence_complete": True,
        "restart_performed": False,
        "blind_reactivation": False,
        "precondition_ready": False,
        "precondition_completed": None,
        "disconnect_verified": False,
        "blocked_result": None,
        "blocked_error": None,
        "command_sent": None,
        "simulator_alive": False,
        "reconnect_healthy": False,
        "fresh_read": False,
        "readiness": None,
        "new_target_distinct": False,
        "motion_attempted": False,
        "motion_accepted": False,
        "motion_completed_value": None,
        "rb1": None,
        "faults_clear": None,
        "reason": "",
        "identity": "",
        "address": "",
        "baseline": None,
        "precondition_motion": None,
        "disconnect": None,
        "blocked": None,
        "reconnect": None,
        "post_reconnect": None,
        "recovery": None,
        "post_motion": None,
    }
    transport = CountingTransport(SshGpasciiTransport(P017_TEST_CONFIG))
    adapter = PowerPmacControllerAdapter(transport)
    try:
        _run_live(adapter, transport, ctx)
    except PowerPmacError as exc:
        rows = ctx["raw_rows"]
        assert isinstance(rows, list)
        rows.append({"step": "ERROR", "error": exc.code})
        ctx["evidence_complete"] = False
        ctx["reason"] = exc.code
    scripted = _scripted_report()
    validator = recovery_leaves_validator_closed()
    _publish(ctx, scripted, validator)


def _run_live(adapter: PowerPmacControllerAdapter, transport: CountingTransport, ctx: dict[str, object]) -> None:
    identity = transport.run_remote("hostname; uname -s")
    address = transport.run_remote("ip -4 -o addr show eth0")
    ctx["identity"] = identity.stdout.strip()
    ctx["address"] = address.stdout.strip()
    if "ppmac" not in str(ctx["identity"]) or "172.20.0.200" not in str(ctx["address"]):
        ctx["evidence_complete"] = False
        ctx["reason"] = "identity mismatch"
        return
    adapter.connect()
    if adapter.communication_state != "HEALTHY":
        ctx["evidence_complete"] = False
        ctx["reason"] = "precondition connect failed"
        return
    baseline, raw, returncode = read_names(transport, STATE_NAMES)
    ctx["baseline"] = {"values": baseline, "raw": raw, "returncode": returncode, "communication": adapter.communication_state, "readiness": readiness_of(baseline)}
    _rows(ctx).append({"step": "PRECONDITION_STATE", "readiness": readiness_of(baseline), "values": baseline})
    if readiness_of(baseline) != "RECOVERED" or not _complete(baseline):
        ctx["reason"] = "precondition state was not motion-ready"
        return
    ctx["precondition_ready"] = True
    precondition_target = choose_precondition_target(baseline["Motor[1].Pos"], baseline["Motor[1].JogPos"])
    if precondition_target is None:
        ctx["evidence_complete"] = False
        ctx["reason"] = "precondition target selection failed"
        return
    blocked_target = choose_blocked_target(precondition_target)
    motion = _run_timeline(transport, precondition_target, ctx, "PRECONDITION")
    ctx["precondition_motion"] = motion
    _commands(ctx).append(str(motion["command"]))
    ctx["precondition_completed"] = motion["completed"] is True
    if motion["completed"] is not True:
        ctx["reason"] = "PRECONDITION_MOTION_FAILED"
        return
    adapter.disconnect()
    ctx["disconnect"] = {"performed": True, "communication": adapter.communication_state, "simulator_process_stopped": False}
    ctx["disconnect_verified"] = adapter.communication_state == "DISCONNECTED"
    mark = len(transport.calls)
    blocked = adapter.move_test_motor(blocked_target, 1)
    during = transport.calls[mark:]
    jog_sent = any(item[0] == "execute" and isinstance(item[1], list) and any(str(line).startswith("#1j=") for line in item[1]) for item in during)
    ctx["blocked"] = {
        "performed": True,
        "requested_target": blocked_target,
        "result": blocked.result,
        "error_code": blocked.error_code,
        "command_sent": blocked.command_sent,
        "transport_calls": during,
        "jog_observed_on_transport": jog_sent,
    }
    ctx["blocked_result"] = blocked.result
    ctx["blocked_error"] = blocked.error_code
    ctx["command_sent"] = blocked.command_sent is True or jog_sent
    _rows(ctx).append({"step": "BLOCK", "result": blocked.result, "error_code": blocked.error_code, "command_sent": blocked.command_sent, "calls": during})
    if blocked.result != "REJECTED" or blocked.error_code != "DISCONNECTED" or blocked.command_sent is not False or jog_sent:
        ctx["reason"] = "disconnect block failed"
        return
    alive = transport.run_remote("hostname")
    ctx["simulator_alive"] = "ppmac" in alive.stdout and adapter.communication_state == "DISCONNECTED"
    _rows(ctx).append({"step": "SIMULATOR_ALIVE", "stdout": alive.stdout, "adapter": adapter.communication_state})
    if ctx["simulator_alive"] is not True:
        ctx["reason"] = "simulator liveness was not observed"
        return
    try:
        adapter.connect()
    except PowerPmacError as exc:
        ctx["reconnect"] = {"performed": True, "communication": adapter.communication_state, "error": exc.code}
        ctx["reason"] = exc.code
        return
    ctx["reconnect"] = {"performed": True, "communication": adapter.communication_state}
    ctx["reconnect_healthy"] = adapter.communication_state == "HEALTHY"
    if not ctx["reconnect_healthy"]:
        ctx["reason"] = "reconnect was not HEALTHY"
        return
    seam = adapter.read_test_motor_state()
    fresh, fresh_raw, fresh_rc = read_names(transport, STATE_NAMES)
    ctx["post_reconnect"] = {"performed": True, "values": fresh, "raw": fresh_raw, "returncode": fresh_rc, "seam_complete": seam.complete, "seam_fault": seam.fault_code, "readiness": readiness_of(fresh)}
    ctx["fresh_read"] = _complete(fresh)
    _rows(ctx).append({"step": "POST_RECONNECT_S0", "readiness": readiness_of(fresh), "values": fresh})
    if not ctx["fresh_read"]:
        ctx["evidence_complete"] = False
        ctx["reason"] = "fresh snapshot incomplete"
        return
    recovery, ready_snapshot = _activate_if_needed(transport, fresh, ctx)
    ctx["recovery"] = recovery
    ctx["readiness"] = recovery["post_readiness"]
    if recovery["post_readiness"] != "RECOVERED":
        ctx["reason"] = "not ready after fresh read"
        return
    new_target = choose_new_target(ready_snapshot["Motor[1].Pos"], ready_snapshot["Motor[1].JogPos"], precondition_target, blocked_target)
    if new_target is None:
        ctx["evidence_complete"] = False
        ctx["reason"] = "new target selection failed"
        return
    ctx["new_target_distinct"] = new_target not in {precondition_target, blocked_target}
    post = _run_timeline(transport, new_target, ctx, "POST_RECOVERY")
    ctx["post_motion"] = post
    ctx["motion_attempted"] = True
    _commands(ctx).append(str(post["command"]))
    faults, _fault_raw, _fault_rc = read_names(transport, ("Motor[1].FeFatal", "Motor[1].AmpFault", "Motor[1].LimitStop"))
    ctx["faults_clear"] = all(values_equal(faults.get(name, NOT_AVAILABLE), "0") for name in ("Motor[1].FeFatal", "Motor[1].AmpFault", "Motor[1].LimitStop"))
    final = _last(post)
    before = post["before"]
    assert isinstance(before, dict)
    final_text = {name: str(final[name]) for name in TIMELINE_NAMES}
    before_text = {name: str(before[name]) for name in TIMELINE_NAMES}
    ctx["rb1"] = is_rb1(before_text, final_text, str(new_target))
    ctx["motion_accepted"] = post["error_present"] is False
    ctx["motion_completed_value"] = (
        post["completed"] is True
        and values_equal(str(final["Motor[1].JogPos"]), str(new_target))
        and ctx["faults_clear"] is True
        and ctx["motion_accepted"] is True
    )


def _activate_if_needed(transport: CountingTransport, snapshot: dict[str, str], ctx: dict[str, object]) -> tuple[dict[str, object], dict[str, str]]:
    ready = readiness_of(snapshot)
    record: dict[str, object] = {
        "performed": True,
        "required": ready != "RECOVERED",
        "candidate": "READINESS_CANDIDATE" if ready == "RECOVERED" else ready,
        "actions": [],
        "readbacks": [],
        "post_readiness": ready,
        "post_state": None,
    }
    if ready == "RECOVERED":
        record["note"] = "Fresh read is a readiness candidate. #1J/ was not sent before the new motion."
        return record, snapshot
    if ready != "NOT_READY":
        return record, snapshot
    actions = record["actions"]
    readbacks = record["readbacks"]
    assert isinstance(actions, list) and isinstance(readbacks, list)
    servo = _float(snapshot.get("Motor[1].ServoCtrl", NOT_AVAILABLE))
    if servo is None:
        record["post_readiness"] = "INCOMPLETE"
        return record, snapshot
    if servo <= 0:
        exchange = transport.execute_lines(["Motor[1].ServoCtrl=1"])
        actions.append("Motor[1].ServoCtrl=1")
        _commands(ctx).append("Motor[1].ServoCtrl=1")
        _rows(ctx).append({"step": "ACTIVATION", "command": "Motor[1].ServoCtrl=1", "returncode": exchange.returncode, "stdout": exchange.stdout})
        value = _read_one(transport, "Motor[1].ServoCtrl", readbacks)
        if value is None or value <= 0:
            record["post_readiness"] = "NOT_READY"
            return record, snapshot
    closed = _read_one(transport, "Motor[1].ClosedLoop", readbacks)
    if closed != 1:
        exchange = transport.execute_lines(["#1J/"])
        actions.append("#1J/")
        _commands(ctx).append("#1J/")
        _rows(ctx).append({"step": "ACTIVATION", "command": "#1J/", "returncode": exchange.returncode, "stdout": exchange.stdout})
        closed = _read_one(transport, "Motor[1].ClosedLoop", readbacks)
        if closed != 1:
            record["post_readiness"] = "NOT_READY"
            return record, snapshot
    post, raw, returncode = read_names(transport, STATE_NAMES)
    record["post_state"] = post
    record["post_raw"] = raw
    record["post_returncode"] = returncode
    record["post_readiness"] = readiness_of(post)
    return record, post


def _run_timeline(transport: CountingTransport, target: int, ctx: dict[str, object], label: str) -> dict[str, object]:
    before, _raw, _returncode = read_names(transport, TIMELINE_NAMES)
    command = "#1j=" + str(target)
    started = time.monotonic()
    jog = transport.execute_lines([command])
    _rows(ctx).append({"step": label + "_COMMAND", "command": command, "returncode": jog.returncode, "stdout": jog.stdout, "stderr": jog.stderr})
    samples: list[dict[str, object]] = []
    for index in (1, 2, 3):
        time.sleep(0.5)
        parsed, _raw, returncode = read_names(transport, TIMELINE_NAMES)
        sample: dict[str, object] = {name: parsed[name] for name in TIMELINE_NAMES}
        sample["t_s"] = time.monotonic() - started
        samples.append(sample)
        _rows(ctx).append({"step": f"{label}_T{index}", "t_s": sample["t_s"], "returncode": returncode, "sample": {name: parsed[name] for name in TIMELINE_NAMES}})
    text_samples = [{name: str(sample[name]) for name in TIMELINE_NAMES} for sample in samples]
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
        "completed": motion_completed(text_samples[-1], str(target), True) if text_samples else False,
        "stdout": jog.stdout,
        "stderr": jog.stderr,
    }


def _scripted_report() -> dict[str, object]:
    faults = [scripted_fault_blocks(name) for name in ("AmpFault", "FeFatal", "LimitStop")]
    recovery = scripted_fault_recovery()
    remains = scripted_fault_remains()
    not_ready = scripted_fault_cleared_but_not_ready()
    detection = all(row["result"] == "FAULTED" and row["command_sent"] is False and row["jog_count"] == 0 for row in faults)
    recovery_ok = recovery["blocked_sent"] is False and recovery["verification"] == "RECOVERED" and recovery["post_result"] == "COMPLETED" and recovery["live_fault_injection"] == "NOT_TESTED"
    failed_ok = remains["command_sent"] is False and remains["jog_count"] == 0 and not_ready["command_sent"] is False and not_ready["jog_count"] == 0
    return {
        "live_fault_injection": "NOT_TESTED",
        "fault_detection": detection,
        "blocked": detection,
        "recovery": recovery_ok,
        "failed_recovery": failed_ok,
        "faults": faults,
        "scripted_recovery": recovery,
        "fault_remains": remains,
        "not_ready": not_ready,
    }


def _publish(ctx: dict[str, object], scripted: dict[str, object], validator: dict[str, object]) -> None:
    commands = _commands(ctx)
    forbidden = [command for command in commands if command not in _ALLOWED and _JOG_RE.fullmatch(command) is None]
    classification = classify_final_lifecycle(
        evidence_complete=ctx["evidence_complete"] is True,
        unexpected_write=bool(forbidden),
        restart_performed=ctx["restart_performed"] is True,
        blind_reactivation=ctx["blind_reactivation"] is True,
        precondition_ready=ctx["precondition_ready"] is True,
        precondition_completed=ctx["precondition_completed"] if isinstance(ctx["precondition_completed"], bool) else None,
        disconnect_verified=ctx["disconnect_verified"] is True,
        blocked_result=ctx["blocked_result"] if isinstance(ctx["blocked_result"], str) else None,
        blocked_error=ctx["blocked_error"] if isinstance(ctx["blocked_error"], str) else None,
        command_sent=ctx["command_sent"] if isinstance(ctx["command_sent"], bool) else None,
        simulator_alive=ctx["simulator_alive"] is True,
        reconnect_healthy=ctx["reconnect_healthy"] is True,
        fresh_read=ctx["fresh_read"] is True,
        readiness=ctx["readiness"] if isinstance(ctx["readiness"], str) else None,
        new_target_distinct=ctx["new_target_distinct"] is True,
        motion_attempted=ctx["motion_attempted"] is True,
        motion_accepted=ctx["motion_accepted"] is True,
        motion_completed_value=ctx["motion_completed_value"] if isinstance(ctx["motion_completed_value"], bool) else None,
        rb1=ctx["rb1"] if isinstance(ctx["rb1"], bool) else None,
        faults_clear=ctx["faults_clear"] if isinstance(ctx["faults_clear"], bool) else None,
    )
    validator_ok = validator.get("validator_result") == "REJECT" and validator.get("virtual_backend_calls") == 0 and validator.get("pmac_backend_calls") == 0 and validator.get("jog_count") == 0
    post = ctx["post_motion"] if isinstance(ctx["post_motion"], dict) else None
    final = _last(post) if post is not None else {}
    target = None if post is None else post.get("target")
    checklist = {
        "precondition_live_motion_completed": ctx["precondition_completed"] is True,
        "adapter_disconnect_verified": ctx["disconnect_verified"] is True,
        "disconnected_motion_rejected": ctx["blocked_result"] == "REJECTED" and ctx["blocked_error"] == "DISCONNECTED",
        "command_sent_false": ctx["command_sent"] is False,
        "simulator_process_remained_alive": ctx["simulator_alive"] is True,
        "reconnect_successful": ctx["reconnect_healthy"] is True,
        "fresh_motor_read_back_performed": ctx["fresh_read"] is True,
        "readiness_recovery_verified": ctx["readiness"] == "RECOVERED",
        "post_recovery_new_target_used": ctx["new_target_distinct"] is True and ctx["motion_attempted"] is True,
        "jogpos_reached_new_target": target is not None and values_equal(str(final.get("Motor[1].JogPos", NOT_AVAILABLE)), str(target)),
        "despos_reached_new_target": target is not None and values_equal(str(final.get("Motor[1].DesPos", NOT_AVAILABLE)), str(target)),
        "pos_reached_new_target": target is not None and values_equal(str(final.get("Motor[1].Pos", NOT_AVAILABLE)), str(target)),
        "actpos_reached_new_target": target is not None and values_equal(str(final.get("Motor[1].ActPos", NOT_AVAILABLE)), str(target)),
        "final_result_completed": ctx["motion_completed_value"] is True,
        "no_fault": ctx["faults_clear"] is True,
        "scripted_fault_regression": scripted.get("fault_detection") is True and scripted.get("recovery") is True,
        "failed_recovery_blocking": scripted.get("failed_recovery") is True,
        "e_fixed_1_virtual_calls_0": validator.get("virtual_backend_calls") == 0,
        "e_fixed_1_pmac_calls_0": validator.get("pmac_backend_calls") == 0,
        "no_forbidden_writes": not forbidden,
        "full_regression": None,
    }
    classification["acceptance_checklist"] = checklist
    classification["acceptance_complete"] = False
    classification["validator_ok"] = validator_ok
    _write_stage("precondition_state.json", ctx["baseline"])
    _write_stage("precondition_motion.json", ctx["precondition_motion"])
    _write_stage("disconnect.json", ctx["disconnect"])
    _write_stage("blocked_motion.json", ctx["blocked"])
    _write_stage("reconnect.json", ctx["reconnect"])
    _write_stage("post_reconnect_state.json", ctx["post_reconnect"])
    _write_stage("recovery.json", ctx["recovery"])
    _write_stage("post_recovery_motion.json", ctx["post_motion"])
    _write(EVIDENCE / "lifecycle.json", _lifecycle(ctx, classification))
    _write(EVIDENCE / "classification.json", classification)
    _write(EVIDENCE / "scripted_fault_regression.json", scripted)
    _write(EVIDENCE / "validator_invariant.json", validator)
    _write(EVIDENCE / "write_audit.json", {"writes_performed": commands, "forbidden_writes": forbidden, "configuration_modifications": forbidden, "simulator_restart_performed": False, "live_fault_injection": "NOT_TESTED"})
    (EVIDENCE / "environment.md").write_text(_environment(str(ctx["identity"]), str(ctx["address"])), encoding="utf-8")
    (EVIDENCE / "summary.md").write_text(_summary(ctx, classification, scripted, validator), encoding="utf-8")
    rows = _rows(ctx)
    (EVIDENCE / "raw" / "live.jsonl").write_text("\n".join(json.dumps(row, ensure_ascii=False) for row in rows) + ("\n" if rows else ""), encoding="utf-8")
    _reject_secret(EVIDENCE)
    print(json.dumps({"classification": classification["classification"], "causal_claim": classification["causal_claim"], "final_pass_declared": False}, ensure_ascii=False))


def _lifecycle(ctx: dict[str, object], classification: dict[str, object]) -> dict[str, object]:
    baseline = ctx["baseline"] if isinstance(ctx["baseline"], dict) else None
    motion = ctx["precondition_motion"] if isinstance(ctx["precondition_motion"], dict) else None
    blocked = ctx["blocked"] if isinstance(ctx["blocked"], dict) else None
    fresh = ctx["post_reconnect"] if isinstance(ctx["post_reconnect"], dict) else None
    recovery = ctx["recovery"] if isinstance(ctx["recovery"], dict) else None
    post = ctx["post_motion"] if isinstance(ctx["post_motion"], dict) else None
    final = _last(post) if post is not None else {}
    return {
        "PRECONDITION": {"performed": baseline is not None, "communication": None if baseline is None else baseline.get("communication"), "readiness": None if baseline is None else baseline.get("readiness"), "motion_completed": None if motion is None else motion.get("completed")},
        "DISCONNECT": {"performed": isinstance(ctx["disconnect"], dict), "adapter_disconnected": ctx["disconnect_verified"] is True},
        "BLOCK": {"performed": blocked is not None, "requested_target": None if blocked is None else blocked.get("requested_target"), "result": None if blocked is None else blocked.get("result"), "command_sent": None if blocked is None else blocked.get("command_sent")},
        "RECONNECT": {"performed": isinstance(ctx["reconnect"], dict), "communication": None if not isinstance(ctx["reconnect"], dict) else ctx["reconnect"].get("communication")},
        "READBACK": {"performed": fresh is not None, "fresh_state": None if fresh is None else fresh.get("values"), "readiness": None if fresh is None else fresh.get("readiness")},
        "RECOVERY": {"performed": recovery is not None, "activation_required": None if recovery is None else recovery.get("required"), "actions": [] if recovery is None else recovery.get("actions"), "verification": None if recovery is None else recovery.get("post_readiness")},
        "NEW_COMMAND": {"performed": post is not None, "target": None if post is None else post.get("target")},
        "EXECUTION": {"performed": post is not None, "JogPos": final.get("Motor[1].JogPos"), "DesPos": final.get("Motor[1].DesPos"), "Pos": final.get("Motor[1].Pos"), "ActPos": final.get("Motor[1].ActPos")},
        "FINAL": classification["classification"],
    }


def _summary(ctx: dict[str, object], classification: dict[str, object], scripted: dict[str, object], validator: dict[str, object]) -> str:
    lines = [
        "# P-021 final summary",
        "",
        f"측정: {datetime.now(timezone.utc).isoformat()}",
        "",
        "Official Power PMAC Simulator adapter disconnect/reconnect software recovery lifecycle. P-021 FINAL PASS를 선언하지 않는다. P-022는 시작하지 않는다.",
        "",
        f"classification: {classification['classification']} {classification.get('classification_name', '')}".rstrip(),
        f"confidence: {classification['confidence']}",
        f"causal_claim: {classification['causal_claim']}",
        f"confirmed: {classification['confirmed']}",
        f"final_pass_declared: {classification['final_pass_declared']}",
        f"identity: {str(ctx['identity']).replace(chr(10), ' / ')}",
    ]
    if ctx["reason"]:
        lines.append(f"stop_reason: {ctx['reason']}")
    baseline = ctx["baseline"] if isinstance(ctx["baseline"], dict) else None
    if baseline is not None:
        values = baseline["values"]
        assert isinstance(values, dict)
        lines.append(
            "precondition ServoCtrl/ClosedLoop/AmpEna/JogPos/DesPos/Pos/ActPos: "
            f"{values.get('Motor[1].ServoCtrl')}/{values.get('Motor[1].ClosedLoop')}/{values.get('Motor[1].AmpEna')}/"
            f"{values.get('Motor[1].JogPos')}/{values.get('Motor[1].DesPos')}/{values.get('Motor[1].Pos')}/{values.get('Motor[1].ActPos')}"
        )
    _append_motion(lines, "precondition motion", ctx["precondition_motion"])
    blocked = ctx["blocked"] if isinstance(ctx["blocked"], dict) else None
    if blocked is not None:
        lines.append(f"blocked target {blocked.get('requested_target')} result {blocked.get('result')} error {blocked.get('error_code')} command_sent {blocked.get('command_sent')} transport_jog {blocked.get('jog_observed_on_transport')}")
    lines.append(f"simulator_alive: {ctx['simulator_alive']}")
    lines.append(f"reconnect: {ctx['reconnect_healthy']}")
    fresh = ctx["post_reconnect"] if isinstance(ctx["post_reconnect"], dict) else None
    if fresh is not None:
        values = fresh["values"]
        assert isinstance(values, dict)
        lines.append(f"fresh readiness: {fresh.get('readiness')} ServoCtrl {values.get('Motor[1].ServoCtrl')} ClosedLoop {values.get('Motor[1].ClosedLoop')}")
    recovery = ctx["recovery"] if isinstance(ctx["recovery"], dict) else None
    if recovery is not None:
        lines.append(f"recovery required {recovery.get('required')} actions {recovery.get('actions')} verification {recovery.get('post_readiness')}")
    _append_motion(lines, "post-recovery motion", ctx["post_motion"])
    lines.append(f"scripted fault detection {scripted.get('fault_detection')} recovery {scripted.get('recovery')} failed_recovery_blocked {scripted.get('failed_recovery')} live_fault_injection {scripted.get('live_fault_injection')}")
    lines.append(f"E-fixed-1 {validator.get('validator_result')} virtual {validator.get('virtual_backend_calls')} pmac {validator.get('pmac_backend_calls')}")
    lines.extend(["", "## Claim boundary", "", *_claims(), ""])
    return "\n".join(lines)


def _append_motion(lines: list[str], label: str, motion: object) -> None:
    if not isinstance(motion, dict):
        lines.append(f"{label}: not performed")
        return
    last = _last(motion)
    lines.append(
        f"{label}: {motion.get('command')} completed {motion.get('completed')} ack {motion.get('ack_present')} error {motion.get('error_present')} "
        f"JogPos {last.get('Motor[1].JogPos')} DesPos {last.get('Motor[1].DesPos')} Pos {last.get('Motor[1].Pos')} ActPos {last.get('Motor[1].ActPos')}"
    )
    for index, sample in enumerate(motion["samples"], start=1):
        assert isinstance(sample, dict)
        lines.append(
            f"{label} T{index} t_s {sample.get('t_s')} JogPos {sample.get('Motor[1].JogPos')} DesPos {sample.get('Motor[1].DesPos')} "
            f"DesVel {sample.get('Motor[1].DesVel')} Pos {sample.get('Motor[1].Pos')} ActPos {sample.get('Motor[1].ActPos')} "
            f"TimerEnabled {sample.get('Motor[1].Desired.TimerEnabled')}"
        )


def _claims() -> list[str]:
    return [
        "- Official Power PMAC Simulator 1.2.1 only",
        "- software virtual Motor[1]",
        "- PMAC motor units",
        "- adapter disconnect/reconnect software recovery lifecycle",
        "- ACK is not motion completion",
        "- ActPos is Simulator virtual feedback",
        "- LIVE_FAULT_INJECTION = NOT_TESTED",
        "- no real Power PMAC hardware",
        "- no physical motor",
        "- no physical encoder",
        "- no physical amplifier",
        "- no physical I/O",
        "- no EtherCAT",
        "- no real robot",
        "- no hard-real-time validation",
        "- no controller commissioning",
        "- no industrial safety claim",
    ]


def _environment(identity: str, address: str) -> str:
    return "\n".join(
        [
            "# P-021 final environment",
            "",
            f"측정: {datetime.now(timezone.utc).isoformat()}",
            "",
            "```text",
            "client: MacBook",
            "path: MacBook -> 192.168.0.146:22022 -> Windows portproxy -> 172.20.0.200:22",
            f"remote identity: {identity.replace(chr(10), ' / ')}",
            f"eth0: {address.replace(chr(10), ' / ')}",
            "pmac backend: Official Power PMAC Simulator 1.2.1",
            "firmware_if_observed: not queried by this runner",
            "disconnect_scope: PowerPmacControllerAdapter communication lifecycle",
            "simulator_process_stopped: false",
            "qemu_process_restarted: false",
            "simulator_IP: 172.20.0.200",
            "live_fault_injection: NOT_TESTED",
            "credential: runtime environment variable PMAC_SSH_PASSWORD; value not recorded",
            "```",
            "",
            "evidence/P-021, P-021A, P-021B, P-021C, P-021D는 수정하지 않는다.",
            "",
        ]
    )


def _write_stage(name: str, value: object) -> None:
    _write(EVIDENCE / name, {"performed": False} if value is None else value)


def _rows(ctx: dict[str, object]) -> list[dict[str, object]]:
    rows = ctx["raw_rows"]
    assert isinstance(rows, list)
    return rows


def _commands(ctx: dict[str, object]) -> list[str]:
    commands = ctx["commands"]
    assert isinstance(commands, list)
    return commands


def _complete(snapshot: dict[str, str]) -> bool:
    return all(snapshot.get(name, NOT_AVAILABLE) != NOT_AVAILABLE for name in STATE_NAMES)


def _read_one(transport: CountingTransport, name: str, readbacks: list[object]) -> float | None:
    parsed, _raw, returncode = read_names(transport, (name,))
    text = parsed.get(name, NOT_AVAILABLE)
    readbacks.append({"name": name, "value": text, "returncode": returncode})
    try:
        return float(text)
    except ValueError:
        return None


def _float(value: str) -> float | None:
    try:
        return float(value)
    except ValueError:
        return None


def _last(motion: dict[str, object]) -> dict[str, object]:
    samples = motion["samples"]
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
