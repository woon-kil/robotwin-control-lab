"""P-023 reliability measurement.

100회의 #1j와 5회의 disconnect/reconnect만 보낸다.
ServoCtrl write, #1J/, Simulator restart는 하지 않는다.
JSONL은 cycle마다 append한다.
"""

from __future__ import annotations

import json
import os
import time
from datetime import datetime, timezone
from pathlib import Path

from controller.p021a_investigation import parse_snapshot
from controller.p022_baseline import gpascii_ok, transport_ok
from controller.p022_integration import precondition_ready
from controller.p023_reliability import (
    alternate_targets,
    apply_event,
    classify_reliability,
    disconnect_blocked,
    empty_counters,
    evaluate_observation,
    metric_summary,
    reconnect_pair,
    reconnect_plan,
    state_leakage,
)
from controller.power_pmac_adapter import PowerPmacControllerAdapter
from controller.power_pmac_config import P017_TEST_CONFIG
from controller.power_pmac_transport import GpasciiExchange, PowerPmacError, SshGpasciiTransport
from digital_twin.controller_axis import (
    MAPPING_BOUNDARY,
    ControllerObservationBridge,
    DomainBoundaryError,
    accept_tracking,
    normalize_motor_snapshot,
)

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "evidence" / "P-023"
READ_NAMES = (
    "Motor[1].ServoCtrl",
    "Motor[1].ClosedLoop",
    "Motor[1].AmpEna",
    "Motor[1].JogPos",
    "Motor[1].DesPos",
    "Motor[1].Pos",
    "Motor[1].ActPos",
    "Motor[1].DesVel",
    "Motor[1].InPos",
    "Motor[1].FeFatal",
    "Motor[1].FeWarn",
    "Motor[1].AmpFault",
    "Motor[1].LimitStop",
)
FINAL_NAMES = READ_NAMES + ("Motor[1].Status[0]", "Motor[1].Status[1]")
IDENTITY = "hostname; echo __P023_UNAME__; uname -s; echo __P023_ETH__; ip -4 -o addr show eth0"
ELAPSED_MEANING = "SSH + Windows portproxy + gpascii + polling을 포함한 observed integration elapsed time. servo latency, controller cycle time, real-time latency, PMAC execution latency가 아니다."
CONTROLLER_STOPS = {"FAULT", "MALFORMED", "NONFINITE", "CLOSED_LOOP_LOST", "SERVO_LOST"}
TRANSPORT_STOPS = {"TIMEOUT", "DISCONNECT", "PROTOCOL_ERROR"}


class RecordingTransport:
    def __init__(self, inner: SshGpasciiTransport) -> None:
        self._inner = inner
        self.calls: list[tuple[object, ...]] = []

    def execute_lines(self, lines: list[str], timeout_s: float | None = None) -> GpasciiExchange:
        self.calls.append(("execute", tuple(lines)))
        return self._inner.execute_lines(lines, timeout_s)

    def read_variable(self, name: str, timeout_s: float | None = None) -> str:
        self.calls.append(("read", name))
        return self._inner.read_variable(name, timeout_s)

    def write_and_read(self, name: str, value: int, timeout_s: float | None = None) -> str:
        self.calls.append(("write", name, value))
        return self._inner.write_and_read(name, value, timeout_s)

    def run_remote(self, remote: str, timeout_s: float | None = None) -> GpasciiExchange:
        self.calls.append(("remote", remote))
        return self._inner.run_remote(remote, timeout_s)

    def close(self) -> None:
        self.calls.append(("close",))
        self._inner.close()


def main() -> None:
    transport = RecordingTransport(SshGpasciiTransport(P017_TEST_CONFIG))
    state = _empty_state()
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    (EVIDENCE / "raw").mkdir(exist_ok=True)
    try:
        _preflight(transport, state)
        if state["run_motion"]:
            _motion(transport, state)
        if state["run_motion"] and not state["environment_stop"]:
            _reconnect(transport, state)
    except PowerPmacError as exc:
        state["transport_stop"] = True
        state["stop_error"] = exc.code
        _incident(state, exc.code)
    _final_health(transport, state)
    state["negative"] = _scripted_negative()
    state["transport_calls"] = transport.calls
    _publish(state)
    print(json.dumps({"classification": state["classification"]["classification"], "attempted": state["counters"]["attempted_cycles"]}, ensure_ascii=False), flush=True)


def _preflight(transport: RecordingTransport, state: dict[str, object]) -> None:
    identity = transport.run_remote(IDENTITY)
    hostname, uname, eth0 = _parse_identity(identity.stdout)
    state["hostname"] = hostname
    state["uname"] = uname
    state["eth0"] = eth0
    state["preflight_transport"] = transport_ok(hostname, uname, eth0) and identity.returncode == 0
    if not state["preflight_transport"]:
        state["stop_reason"] = "P023_PRECONDITION_NOT_READY"
        _write(EVIDENCE / "preflight.json", _preflight_document(state))
        return
    servo = transport.execute_lines(["Sys.ServoCount"])
    value = parse_snapshot(servo.stdout, ("Sys.ServoCount",))["Sys.ServoCount"]
    state["servo_count"] = value
    state["gpascii"] = gpascii_ok(value) and "timed out" not in servo.stderr.lower()
    if not state["gpascii"]:
        state["preflight_transport"] = False
        state["stop_reason"] = "P023_PRECONDITION_NOT_READY"
        _write(EVIDENCE / "preflight.json", _preflight_document(state))
        return
    snapshot, _raw, returncode = _read_names(transport, READ_NAMES)
    state["initial"] = snapshot
    state["preflight_returncode"] = returncode
    state["preflight_ready"] = precondition_ready(snapshot)
    _write(EVIDENCE / "preflight.json", _preflight_document(state))
    if not state["preflight_ready"]:
        state["stop_reason"] = "P023_PRECONDITION_NOT_READY"
        return
    state["run_motion"] = True


def _motion(transport: RecordingTransport, state: dict[str, object]) -> None:
    initial = state["initial"]
    assert isinstance(initial, dict)
    targets = alternate_targets(str(initial.get("Motor[1].Pos", "NOT_AVAILABLE")), 100)
    if targets is None:
        state["stop_reason"] = "TARGET_SELECTION_FAILED"
        state["evidence_complete"] = False
        return
    state["targets"] = targets
    previous: dict[str, str] | None = None
    for index, target in enumerate(targets, start=1):
        cycle, stop_event = _one_motion(transport, state, index, target, previous)
        previous_post = cycle.get("post")
        previous = previous_post if isinstance(previous_post, dict) else previous
        if index % 10 == 0 or stop_event:
            print(f"cycle {index}/100", flush=True)
        if stop_event:
            state["environment_stop"] = True
            state["stop_reason"] = stop_event
            if stop_event in CONTROLLER_STOPS:
                state["controller_stop"] = True
            if stop_event in TRANSPORT_STOPS:
                state["transport_stop"] = True
            if stop_event == "RUNTIME":
                state["runtime_failure"] = True
            return


def _one_motion(
    transport: RecordingTransport,
    state: dict[str, object],
    index: int,
    target: int,
    previous: dict[str, str] | None,
    *,
    record: bool = True,
) -> tuple[dict[str, object], str | None]:
    started_at = _now()
    pre, _raw, _rc = _read_names(transport, READ_NAMES)
    counters = state["counters"]
    assert isinstance(counters, dict)
    if record and state_leakage(previous, pre):
        counters["state_leakage_or_external_change"] = int(counters["state_leakage_or_external_change"]) + 1
    command = "#1j=" + str(target)
    _allow(state, command)
    monotonic = time.monotonic()
    try:
        jog = transport.execute_lines([command])
    except PowerPmacError as exc:
        event = "TIMEOUT" if exc.code == "TIMEOUT" else "DISCONNECT"
        cycle = _failed_cycle(index, target, started_at, pre, event, exc.code)
        if record:
            _store_cycle(state, cycle, event)
        return cycle, event
    text = jog.stdout + "\n" + jog.stderr
    samples: list[dict[str, object]] = []
    post: dict[str, str] | None = None
    evaluated: dict[str, object] | None = None
    for sample_index in (1, 2, 3):
        time.sleep(0.5)
        try:
            parsed, _raw, returncode = _read_names(transport, READ_NAMES)
        except PowerPmacError as exc:
            event = "TIMEOUT" if exc.code == "TIMEOUT" else "DISCONNECT"
            cycle = _failed_cycle(index, target, started_at, pre, event, exc.code)
            cycle["command_ack"] = "\x06" in jog.stdout
            cycle["command_returncode"] = jog.returncode
            if record:
                _store_cycle(state, cycle, event)
            return cycle, event
        sample = {name: parsed[name] for name in READ_NAMES}
        sample["t_s"] = time.monotonic() - monotonic
        sample["returncode"] = returncode
        samples.append(sample)
        post = parsed
        evaluated = evaluate_observation(
            parsed,
            target=str(target),
            ack_present="\x06" in jog.stdout,
            error_present="error #" in text.lower(),
            fresh=True,
            communication_state="HEALTHY",
            observed_at=_now(),
        )
        if evaluated["completed"] or evaluated["stop"]:
            break
    assert post is not None and evaluated is not None
    event = str(evaluated["event"])
    cycle = {
        "cycle_id": index,
        "target": target,
        "started_at": started_at,
        "completed_at": _now(),
        "pre": {name: pre[name] for name in ("Motor[1].JogPos", "Motor[1].DesPos", "Motor[1].Pos", "Motor[1].ActPos")},
        "post": {name: post[name] for name in READ_NAMES},
        "ClosedLoop": post.get("Motor[1].ClosedLoop"),
        "InPos": post.get("Motor[1].InPos"),
        "FeFatal": post.get("Motor[1].FeFatal"),
        "FeWarn": post.get("Motor[1].FeWarn"),
        "AmpFault": post.get("Motor[1].AmpFault"),
        "LimitStop": post.get("Motor[1].LimitStop"),
        "communication_state": "HEALTHY",
        "command_ack": "\x06" in jog.stdout,
        "command_error": "error #" in text.lower(),
        "command_returncode": jog.returncode,
        "ack_is_completion": False,
        "tracking_error": evaluated["tracking_error"],
        "tracking_error_unit": "pmac_motor_unit",
        "elapsed_s": time.monotonic() - monotonic,
        "elapsed_meaning": ELAPSED_MEANING,
        "samples": samples,
        "observation": evaluated["observation"],
        "result": "COMPLETED" if evaluated["completed"] else event,
        "error_code": None if evaluated["completed"] else event,
        "fresh_pre": True,
        "fresh_post": True,
    }
    if record:
        _store_cycle(state, cycle, "COMPLETED" if evaluated["completed"] else event)
    if evaluated["stop"]:
        return cycle, event
    return cycle, None


def _reconnect(transport: RecordingTransport, state: dict[str, object]) -> None:
    state["reconnect_ran"] = True
    adapter = PowerPmacControllerAdapter(transport)
    trials: list[dict[str, object]] = []
    for trial in range(1, 6):
        snapshot, _raw, _rc = _read_names(transport, READ_NAMES)
        if not precondition_ready(snapshot):
            trials.append({"trial": trial, "result": "RECONNECT_NOT_READY", "activation": []})
            state["controller_stop"] = True
            break
        position = int(float(snapshot["Motor[1].Pos"]))
        blocked, renewed = reconnect_pair(position)
        adapter.connect()
        adapter.disconnect()
        mark = len(transport.calls)
        blocked_result = adapter.move_test_motor(blocked, 1)
        jog_observed = _jog_since(transport, mark)
        blocked_ok = disconnect_blocked(
            result=blocked_result.result,
            error_code=blocked_result.error_code,
            command_sent=blocked_result.command_sent,
            jog_observed=jog_observed,
        )
        metrics = state["reconnect_metrics"]
        assert isinstance(metrics, dict)
        metrics["reconnect_trials"] = int(metrics["reconnect_trials"]) + 1
        if blocked_ok:
            metrics["blocked_while_disconnected"] = int(metrics["blocked_while_disconnected"]) + 1
        else:
            metrics["false_submission_count"] = int(metrics["false_submission_count"]) + 1
        try:
            adapter.connect()
        except PowerPmacError as exc:
            metrics["reconnect_failure"] = int(metrics["reconnect_failure"]) + 1
            trials.append({"trial": trial, "blocked": blocked_ok, "result": exc.code, "activation": []})
            state["transport_stop"] = True
            break
        fresh, _raw, _rc = _read_names(transport, READ_NAMES)
        plan = reconnect_plan(fresh_read=True, ready=precondition_ready(fresh))
        record: dict[str, object] = {
            "trial": trial,
            "blocked_target": blocked,
            "blocked": blocked_ok,
            "command_sent": blocked_result.command_sent,
            "fresh_read": True,
            "plan": plan["reason"],
            "activation": plan["activation"],
        }
        if plan["motion"] is not True:
            metrics["reconnect_failure"] = int(metrics["reconnect_failure"]) + 1
            metrics["post_reconnect_motion_failed"] = int(metrics["post_reconnect_motion_failed"]) + 1
            record["result"] = plan["reason"]
            trials.append(record)
            if plan["reason"] == "RECONNECT_NOT_READY":
                state["controller_stop"] = True
            break
        outcome, stop_event = _one_motion(transport, state, 100 + trial, renewed, fresh, record=False)
        record["renewed_target"] = renewed
        record["motion"] = outcome.get("result")
        record["result"] = outcome.get("result")
        if outcome.get("result") == "COMPLETED" and blocked_ok:
            metrics["reconnect_success"] = int(metrics["reconnect_success"]) + 1
            metrics["post_reconnect_motion_completed"] = int(metrics["post_reconnect_motion_completed"]) + 1
        else:
            metrics["reconnect_failure"] = int(metrics["reconnect_failure"]) + 1
            metrics["post_reconnect_motion_failed"] = int(metrics["post_reconnect_motion_failed"]) + 1
        trials.append(record)
        _append(EVIDENCE / "raw" / "reconnect.jsonl", record)
        if stop_event:
            state["environment_stop"] = True
            if stop_event in CONTROLLER_STOPS:
                state["controller_stop"] = True
            if stop_event in TRANSPORT_STOPS:
                state["transport_stop"] = True
            break
    state["reconnect_trials"] = trials
    _write(EVIDENCE / "reconnect_trials.json", {"trials": trials})
    _write(EVIDENCE / "reconnect_metrics.json", state["reconnect_metrics"])


def _final_health(transport: RecordingTransport, state: dict[str, object]) -> None:
    try:
        snapshot, raw, returncode = _read_names(transport, FINAL_NAMES)
    except PowerPmacError as exc:
        state["final_ready"] = False
        state["transport_stop"] = True
        state["stop_error"] = exc.code
        _write(EVIDENCE / "final_health.json", {"performed": False, "error": exc.code})
        return
    state["final"] = snapshot
    state["final_ready"] = precondition_ready(snapshot)
    initial = state["initial"] if isinstance(state["initial"], dict) else {}
    _write(
        EVIDENCE / "final_health.json",
        {
            "performed": True,
            "values": snapshot,
            "raw": raw,
            "returncode": returncode,
            "ready": state["final_ready"],
            "initial_pos": initial.get("Motor[1].Pos"),
            "final_pos": snapshot.get("Motor[1].Pos"),
        },
    )


def _store_cycle(state: dict[str, object], cycle: dict[str, object], event: str) -> None:
    cycles = state["cycles"]
    counters = state["counters"]
    assert isinstance(cycles, list) and isinstance(counters, dict)
    cycles.append(cycle)
    state["counters"] = apply_event(counters, event)
    if cycle.get("tracking_error") is not None:
        errors = state["tracking_errors"]
        assert isinstance(errors, list)
        errors.append(float(cycle["tracking_error"]))
    if cycle.get("elapsed_s") is not None:
        elapsed = state["elapsed"]
        assert isinstance(elapsed, list)
        elapsed.append(float(cycle["elapsed_s"]))
    _append(EVIDENCE / "raw" / "motion_cycles.jsonl", {key: cycle[key] for key in cycle if key != "samples"})
    if int(cycle["cycle_id"]) % 10 == 0:
        _write(EVIDENCE / "motion_cycles.json", {"cycles": cycles})


def _failed_cycle(index: int, target: int, started_at: str, pre: dict[str, str], event: str, code: str) -> dict[str, object]:
    return {
        "cycle_id": index,
        "target": target,
        "started_at": started_at,
        "completed_at": _now(),
        "pre": {name: pre.get(name) for name in ("Motor[1].JogPos", "Motor[1].DesPos", "Motor[1].Pos", "Motor[1].ActPos")},
        "post": None,
        "communication_state": "DISCONNECTED" if event == "DISCONNECT" else "HEALTHY",
        "command_ack": False,
        "command_error": False,
        "command_returncode": None,
        "ack_is_completion": False,
        "tracking_error": None,
        "result": event,
        "error_code": code,
        "fresh_post": False,
    }


def _scripted_negative() -> dict[str, object]:
    observed = _now()
    base = {
        "Motor[1].DesPos": "1",
        "Motor[1].Pos": "1",
        "Motor[1].ActPos": "1",
        "Motor[1].ClosedLoop": "1",
        "Motor[1].InPos": "1",
        "Motor[1].FeFatal": "0",
        "Motor[1].AmpFault": "0",
        "Motor[1].LimitStop": "0",
    }
    disconnected = normalize_motor_snapshot(base, communication_state="DISCONNECTED", command_target=1, observed_at=observed, fresh=True)
    faulted = normalize_motor_snapshot({**base, "Motor[1].AmpFault": "1"}, communication_state="HEALTHY", command_target=1, observed_at=observed, fresh=True)
    malformed = normalize_motor_snapshot({**base, "Motor[1].ActPos": "malformed"}, communication_state="HEALTHY", command_target=1, observed_at=observed, fresh=True)
    incomplete_snapshot = dict(base)
    del incomplete_snapshot["Motor[1].DesPos"]
    incomplete = normalize_motor_snapshot(incomplete_snapshot, communication_state="HEALTHY", command_target=1, observed_at=observed, fresh=True)
    stale = normalize_motor_snapshot(base, communication_state="HEALTHY", command_target=1, observed_at=observed, fresh=False)
    bridge = ControllerObservationBridge()
    rejected = False
    try:
        bridge.request_unit_conversion("pmac_motor_unit", "rad")
    except DomainBoundaryError:
        rejected = True
    cases = {
        "disconnected": disconnected.valid is False,
        "fault": faulted.faulted is True and faulted.tracking_success is False and accept_tracking(faulted) is False,
        "malformed": malformed.valid is False,
        "incomplete": incomplete.valid is False,
        "stale": stale.valid is False,
        "unit_mismatch": rejected,
    }
    return {"live_fault_injection": "NOT_TESTED", "scripted": True, "cases": cases, "passed": all(cases.values())}


def _publish(state: dict[str, object]) -> None:
    counters = state["counters"]
    assert isinstance(counters, dict)
    cycles = state["cycles"]
    assert isinstance(cycles, list)
    performed = _performed_writes(state["transport_calls"])
    allowed = state["allowed_writes"]
    assert isinstance(allowed, list)
    forbidden = [item for item in performed if item not in allowed]
    activation = [item for item in performed if item in {"Motor[1].ServoCtrl=1", "#1J/"}]
    defect_fields = (
        "timeout_count",
        "disconnect_count",
        "protocol_error_count",
        "fault_count",
        "nonfinite_count",
        "malformed_count",
        "tracking_mismatch_count",
        "stale_observation_count",
        "unexpected_state_count",
        "state_leakage_or_external_change",
        "runtime_failure_count",
    )
    defect_count = sum(int(counters.get(name, 0)) for name in defect_fields)
    metrics = state["reconnect_metrics"]
    assert isinstance(metrics, dict)
    reconnect_acceptance = (
        int(metrics["reconnect_trials"]) == 5
        and int(metrics["blocked_while_disconnected"]) == 5
        and int(metrics["false_submission_count"]) == 0
        and int(metrics["reconnect_success"]) == 5
        and int(metrics["post_reconnect_motion_completed"]) == 5
    )
    negative = state["negative"]
    assert isinstance(negative, dict)
    observations_ok = all(
        isinstance(cycle.get("observation"), dict) and cycle["observation"].get("unit") == "pmac_motor_unit"
        for cycle in cycles
        if isinstance(cycle, dict) and isinstance(cycle.get("observation"), dict)
    )
    classification = classify_reliability(
        preflight_transport=bool(state["preflight_transport"]),
        preflight_ready=bool(state["preflight_ready"]),
        motion_attempted=int(counters["attempted_cycles"]),
        motion_completed=int(counters["completed_cycles"]),
        motion_defect_count=defect_count,
        reconnect_ran=bool(state["reconnect_ran"]),
        reconnect_acceptance=reconnect_acceptance,
        controller_stop=bool(state["controller_stop"]),
        transport_stop=bool(state["transport_stop"]),
        runtime_failure=bool(state["runtime_failure"]),
        final_ready=bool(state["final_ready"]),
        negative_pass=bool(negative.get("passed")),
        domain_ok=observations_ok and MAPPING_BOUNDARY["robot_joint_mapping"] == "NOT_DEFINED",
        forbidden_write=bool(forbidden or activation),
        regression_passed=None,
        evidence_complete=bool(state["evidence_complete"]),
    )
    if state["stop_reason"]:
        classification["stop_reason"] = state["stop_reason"]
    if state["stop_error"]:
        classification["stop_error"] = state["stop_error"]
    state["classification"] = classification
    attempted = int(counters["attempted_cycles"])
    completed = int(counters["completed_cycles"])
    tracking = state["tracking_errors"]
    elapsed = state["elapsed"]
    assert isinstance(tracking, list) and isinstance(elapsed, list)
    _write(EVIDENCE / "motion_cycles.json", {"cycles": cycles})
    _write(
        EVIDENCE / "motion_metrics.json",
        {
            "attempted": attempted,
            "completed": completed,
            "failed": int(counters["failed_cycles"]),
            "completion_rate": None if attempted == 0 else completed / attempted,
            "tracking_error": metric_summary([float(item) for item in tracking]),
            "tracking_error_unit": "pmac_motor_unit",
            "tracking_error_meaning": "controller-domain virtual tracking error",
            "elapsed": metric_summary([float(item) for item in elapsed]),
            "elapsed_meaning": ELAPSED_MEANING,
            "timeouts": counters["timeout_count"],
            "disconnects": counters["disconnect_count"],
            "protocol_errors": counters["protocol_error_count"],
            "faults": counters["fault_count"],
            "nonfinite": counters["nonfinite_count"],
            "malformed": counters["malformed_count"],
            "tracking_mismatch": counters["tracking_mismatch_count"],
            "stale_observation": counters["stale_observation_count"],
            "unexpected_state": counters["unexpected_state_count"],
            "runtime_failures": counters["runtime_failure_count"],
            "state_leakage_or_external_change": counters["state_leakage_or_external_change"],
            "industrial_reliability_claimed": False,
        },
    )
    _write(EVIDENCE / "reconnect_trials.json", {"trials": state["reconnect_trials"]})
    _write(EVIDENCE / "reconnect_metrics.json", metrics)
    _write(EVIDENCE / "negative_regression.json", negative)
    _write(EVIDENCE / "domain_boundary.json", dict(MAPPING_BOUNDARY))
    _write(EVIDENCE / "runtime_incidents.json", state["runtime_incidents"])
    _write(
        EVIDENCE / "write_audit.json",
        {
            "writes_performed": performed,
            "allowed_writes": allowed,
            "activation_writes": activation,
            "forbidden_writes": forbidden,
        },
    )
    _write(EVIDENCE / "regression_results.json", {"full_regression": None, "p023_unit": None, "p022_unit": None, "p021_final_unit": None})
    _write(EVIDENCE / "classification.json", classification)
    (EVIDENCE / "environment.md").write_text(_environment(state), encoding="utf-8")
    (EVIDENCE / "summary.md").write_text(_summary(state, classification), encoding="utf-8")
    (EVIDENCE / "raw" / "pytest.txt").write_text("", encoding="utf-8")
    _reject_secret(EVIDENCE)


def _performed_writes(calls: object) -> list[str]:
    if not isinstance(calls, list):
        return []
    writes: list[str] = []
    for call in calls:
        if not isinstance(call, tuple) or not call:
            continue
        if call[0] == "write":
            writes.append(str(call[1]) + "=" + str(call[2]))
        if call[0] == "execute" and isinstance(call[1], tuple):
            for line in call[1]:
                if _is_write(str(line)):
                    writes.append(str(line))
    return writes


def _is_write(line: str) -> bool:
    return line.startswith("#1") or "=" in line


def _jog_since(transport: RecordingTransport, mark: int) -> bool:
    for call in transport.calls[mark:]:
        if call[0] == "execute" and isinstance(call[1], tuple):
            if any(str(line).startswith("#1j=") for line in call[1]):
                return True
    return False


def _allow(state: dict[str, object], command: str) -> None:
    allowed = state["allowed_writes"]
    assert isinstance(allowed, list)
    allowed.append(command)


def _incident(state: dict[str, object], code: str) -> None:
    incidents = state["runtime_incidents"]
    assert isinstance(incidents, dict)
    incidents["this_run"].append({"code": code, "root_cause": "NOT_CONFIRMED"})


def _empty_state() -> dict[str, object]:
    return {
        "hostname": "",
        "uname": "",
        "eth0": "",
        "servo_count": "NOT_AVAILABLE",
        "gpascii": False,
        "preflight_transport": False,
        "preflight_ready": False,
        "preflight_returncode": None,
        "initial": None,
        "final": None,
        "final_ready": False,
        "run_motion": False,
        "environment_stop": False,
        "controller_stop": False,
        "transport_stop": False,
        "runtime_failure": False,
        "reconnect_ran": False,
        "stop_reason": None,
        "stop_error": None,
        "targets": [],
        "cycles": [],
        "counters": empty_counters(),
        "tracking_errors": [],
        "elapsed": [],
        "allowed_writes": [],
        "transport_calls": [],
        "reconnect_trials": [],
        "reconnect_metrics": {
            "reconnect_trials": 0,
            "reconnect_success": 0,
            "reconnect_failure": 0,
            "blocked_while_disconnected": 0,
            "false_submission_count": 0,
            "post_reconnect_motion_completed": 0,
            "post_reconnect_motion_failed": 0,
        },
        "negative": {"passed": False},
        "runtime_incidents": {
            "prior": "SIMULATOR_RUNTIME_FAILURE",
            "root_cause": "NOT_CONFIRMED",
            "qemu_console_visible_to_runner": False,
            "this_run": [],
        },
        "evidence_complete": True,
        "classification": {},
    }


def _read_names(transport: RecordingTransport, names: tuple[str, ...]) -> tuple[dict[str, str], str, int]:
    exchange = transport.execute_lines(list(names))
    parsed = parse_snapshot(exchange.stdout, names)
    raw = exchange.stdout
    for name in [item for item, value in parsed.items() if value == "NOT_AVAILABLE"]:
        single = transport.execute_lines([name])
        raw += "\n" + single.stdout
        parsed[name] = parse_snapshot(single.stdout, (name,))[name]
    return parsed, raw, exchange.returncode


def _parse_identity(stdout: str) -> tuple[str, str, str]:
    host, _, rest = stdout.partition("__P023_UNAME__")
    uname, _, eth = rest.partition("__P023_ETH__")
    return host.strip(), uname.strip(), eth.strip()


def _preflight_document(state: dict[str, object]) -> dict[str, object]:
    initial = state["initial"] if isinstance(state["initial"], dict) else {}
    return {
        "ssh": "PASS" if state["preflight_transport"] else "FAIL",
        "hostname": state["hostname"],
        "os": state["uname"],
        "eth0": state["eth0"],
        "gpascii": "PASS" if state["gpascii"] else "FAIL",
        "Sys.ServoCount": state["servo_count"],
        "ready": state["preflight_ready"],
        "values": initial,
        "activation_performed": False,
    }


def _environment(state: dict[str, object]) -> str:
    return "\n".join(
        [
            "# P-023 environment",
            "",
            f"측정: {_now()}",
            "",
            "client: MacBook",
            "path: MacBook -> 192.168.0.146:22022 -> Windows portproxy -> 172.20.0.200:22",
            f"hostname: {state['hostname']}",
            f"os: {state['uname']}",
            f"eth0: {state['eth0']}",
            "pmac backend: Official Power PMAC Simulator 1.2.1",
            "axis: software virtual Motor[1]",
            "unit: pmac_motor_unit",
            "credential: runtime environment variable PMAC_SSH_PASSWORD; value not recorded",
            "",
            ELAPSED_MEANING,
            "",
            "P-011 500-cycle evidence와 이 측정은 별개다.",
            "industrial reliability는 주장하지 않는다.",
            "",
        ]
    )


def _summary(state: dict[str, object], classification: dict[str, object]) -> str:
    counters = state["counters"]
    assert isinstance(counters, dict)
    initial = state["initial"] if isinstance(state["initial"], dict) else {}
    final = state["final"] if isinstance(state["final"], dict) else {}
    metrics = state["reconnect_metrics"]
    assert isinstance(metrics, dict)
    return "\n".join(
        [
            "# P-023 reliability",
            "",
            f"측정: {_now()}",
            "",
            f"classification: {classification.get('classification')} {classification.get('classification_name')}",
            f"p023_final_pass_declared: {classification.get('p023_final_pass_declared')}",
            f"industrial_reliability_claimed: false",
            "",
            f"preflight transport: {state['preflight_transport']}",
            f"preflight ready: {state['preflight_ready']}",
            f"initial Pos: {initial.get('Motor[1].Pos', 'NOT_AVAILABLE')}",
            f"attempted: {counters['attempted_cycles']}",
            f"completed: {counters['completed_cycles']}",
            f"failed: {counters['failed_cycles']}",
            f"reconnect trials: {metrics['reconnect_trials']}",
            f"reconnect success: {metrics['reconnect_success']}",
            f"blocked while disconnected: {metrics['blocked_while_disconnected']}",
            f"false submissions: {metrics['false_submission_count']}",
            f"post reconnect completed: {metrics['post_reconnect_motion_completed']}",
            f"final ready: {state['final_ready']}",
            f"final Pos: {final.get('Motor[1].Pos', 'NOT_AVAILABLE')}",
            "",
            ELAPSED_MEANING,
            "",
            "robot_joint_mapping: NOT_DEFINED",
            "unit_conversion: NOT_DEFINED",
            "",
            "Gate: P-023 REVIEW REQUIRED",
            "P-024 started: NO",
            "",
        ]
    )


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _append(path: Path, row: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(row, ensure_ascii=False) + "\n")
        handle.flush()


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
