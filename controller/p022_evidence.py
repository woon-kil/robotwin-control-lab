"""P-022 live controller-domain observation.

READY session에서 #1j 세 번만 보낸다.
ServoCtrl write와 #1J/로 복구하지 않는다.
"""

from __future__ import annotations

import json
import os
import time
from datetime import datetime, timezone
from pathlib import Path

from controller.p021a_investigation import parse_snapshot
from controller.p022_integration import (
    choose_cycle_targets,
    classify_integration,
    motion_completed,
    precondition_ready,
    values_equal,
)
from controller.power_pmac_config import P017_TEST_CONFIG
from controller.power_pmac_transport import PowerPmacError, SshGpasciiTransport
from digital_twin.controller_axis import (
    MAPPING_BOUNDARY,
    ControllerObservationBridge,
    DomainBoundaryError,
    accept_tracking,
    normalize_motor_snapshot,
)

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "evidence" / "P-022" / "main"
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
FAULT_NAMES = ("Motor[1].FeFatal", "Motor[1].AmpFault", "Motor[1].LimitStop")


def main() -> None:
    transport = SshGpasciiTransport(P017_TEST_CONFIG)
    state = _empty_state()
    try:
        _run(transport, state)
    except PowerPmacError as exc:
        state["stop_error"] = exc.code
    _publish(state)
    print(json.dumps({"classification": state["classification"]["classification"]}, ensure_ascii=False))


def _run(transport: SshGpasciiTransport, state: dict[str, object]) -> None:
    snapshot, raw, returncode = _read_names(transport, READ_NAMES)
    state["precondition"] = snapshot
    state["precondition_raw"] = raw
    state["precondition_returncode"] = returncode
    state["raw_rows"].append({"step": "PRECONDITION", "returncode": returncode, "values": snapshot})
    ready = precondition_ready(snapshot)
    state["precondition_ready"] = ready
    if not ready:
        state["stop_reason"] = "PRECONDITION_NOT_READY"
        return
    targets = choose_cycle_targets(snapshot.get("Motor[1].Pos", "NOT_AVAILABLE"))
    if targets is None:
        state["stop_reason"] = "TARGET_SELECTION_FAILED"
        return
    state["targets"] = list(targets)
    for index, target in enumerate(targets, start=1):
        if not _cycle(transport, state, index, target):
            return


def _cycle(transport: SshGpasciiTransport, state: dict[str, object], index: int, target: int) -> bool:
    pre, _raw, pre_rc = _read_names(transport, READ_NAMES)
    pre_at = _now()
    rows = state["raw_rows"]
    assert isinstance(rows, list)
    rows.append({"step": f"C{index}_PRE", "returncode": pre_rc, "observed_at": pre_at, "values": pre})
    command = "#1j=" + str(target)
    allowed = state["allowed_writes"]
    performed = state["writes_performed"]
    assert isinstance(allowed, list) and isinstance(performed, list)
    allowed.append(command)
    performed.append(command)
    started = time.monotonic()
    jog = transport.execute_lines([command])
    rows.append({"step": f"C{index}_COMMAND", "command": command, "returncode": jog.returncode, "stdout": jog.stdout, "stderr": jog.stderr})
    samples = []
    for sample_index in (1, 2, 3):
        time.sleep(0.5)
        parsed, _raw, returncode = _read_names(transport, READ_NAMES)
        sample = {name: parsed[name] for name in READ_NAMES}
        sample["t_s"] = time.monotonic() - started
        sample["returncode"] = returncode
        samples.append(sample)
        rows.append({"step": f"C{index}_T{sample_index}", "t_s": sample["t_s"], "values": sample})
    post = samples[-1]
    post_at = _now()
    text = jog.stdout + "\n" + jog.stderr
    positions_ok = motion_completed(post, str(target), "\x06" in jog.stdout)
    faults_ok = all(values_equal(post.get(name, "NOT_AVAILABLE"), "0") for name in FAULT_NAMES)
    jog_ok = values_equal(post.get("Motor[1].JogPos", "NOT_AVAILABLE"), str(target))
    completed = positions_ok and faults_ok and jog_ok and "error #" not in text.lower()
    observation = normalize_motor_snapshot(
        post,
        communication_state="HEALTHY",
        command_target=float(target),
        observed_at=post_at,
        fresh=True,
    )
    bridge = state["bridge"]
    assert isinstance(bridge, ControllerObservationBridge)
    bridge.publish(observation)
    cycle = {
        "index": index,
        "pre": {name: pre[name] for name in READ_NAMES},
        "pre_observed_at": pre_at,
        "pre_returncode": pre_rc,
        "target": target,
        "units": "pmac_motor_unit",
        "command": command,
        "ack_present": "\x06" in jog.stdout,
        "error_present": "error #" in text.lower(),
        "returncode": jog.returncode,
        "ack_is_motion_completion": False,
        "timeline_note": "SSH/gpascii sample times are not a high-frequency servo trace",
        "samples": samples,
        "post": {name: post[name] for name in READ_NAMES},
        "post_observed_at": post_at,
        "fresh_post": True,
        "observation": observation.to_dict(),
        "tracking_error": observation.tracking_error,
        "completed": completed,
    }
    cycles = state["cycles"]
    assert isinstance(cycles, list)
    cycles.append(cycle)
    if completed and observation.valid and observation.tracking_success and observation.tracking_error == 0:
        done = state["cycles_completed"]
        assert isinstance(done, int)
        state["cycles_completed"] = done + 1
        return True
    state["stop_reason"] = "CYCLE_FAILED"
    return False


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


def _empty_state() -> dict[str, object]:
    return {
        "precondition": None,
        "precondition_raw": "",
        "precondition_returncode": None,
        "precondition_ready": False,
        "targets": [],
        "cycles": [],
        "cycles_completed": 0,
        "writes_performed": [],
        "allowed_writes": [],
        "stop_reason": None,
        "stop_error": None,
        "raw_rows": [],
        "bridge": ControllerObservationBridge(),
        "negative": _scripted_negative(),
        "classification": {},
    }


def _scripted_negative() -> dict[str, object]:
    observed = "2026-10-26T00:00:00+00:00"
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
    malformed = normalize_motor_snapshot({**base, "Motor[1].ActPos": "malformed"}, communication_state="HEALTHY", command_target=1, observed_at=observed, fresh=True)
    incomplete_snapshot = dict(base)
    del incomplete_snapshot["Motor[1].DesPos"]
    incomplete = normalize_motor_snapshot(incomplete_snapshot, communication_state="HEALTHY", command_target=1, observed_at=observed, fresh=True)
    faulted = normalize_motor_snapshot({**base, "Motor[1].AmpFault": "1"}, communication_state="HEALTHY", command_target=1, observed_at=observed, fresh=True)
    stale = normalize_motor_snapshot(base, communication_state="HEALTHY", command_target=1, observed_at=observed, fresh=False)
    bridge = ControllerObservationBridge()
    conversion_rejected = False
    try:
        bridge.request_unit_conversion("pmac_motor_unit", "rad")
    except DomainBoundaryError:
        conversion_rejected = True
    cases = {
        "disconnected": disconnected.valid is False and disconnected.invalid_reason == "DISCONNECTED",
        "malformed": malformed.valid is False and malformed.invalid_reason == "MALFORMED",
        "incomplete": incomplete.valid is False and incomplete.invalid_reason == "INCOMPLETE",
        "fault": faulted.faulted is True and faulted.tracking_success is False and accept_tracking(faulted) is False,
        "stale": stale.valid is False and stale.invalid_reason == "STALE",
        "unit_mismatch": conversion_rejected,
    }
    return {
        "live_fault_injection": "NOT_TESTED",
        "scripted": True,
        "cases": cases,
        "passed": all(cases.values()),
    }


def _publish(state: dict[str, object]) -> None:
    performed = state["writes_performed"]
    allowed = state["allowed_writes"]
    assert isinstance(performed, list) and isinstance(allowed, list)
    forbidden = [item for item in performed if item not in allowed or not str(item).startswith("#1j=")]
    activation = [item for item in performed if item in {"Motor[1].ServoCtrl=1", "#1J/"}]
    negative = state["negative"]
    assert isinstance(negative, dict)
    cycles = state["cycles"]
    assert isinstance(cycles, list)
    observation_ok = bool(cycles) and all(
        cycle["completed"] is True
        and cycle["fresh_post"] is True
        and cycle["observation"]["valid"] is True
        and cycle["observation"]["tracking_success"] is True
        and cycle["tracking_error"] == 0
        for cycle in cycles
        if isinstance(cycle, dict)
    )
    observation_ok = observation_ok and len(cycles) == 3
    domain_violation = any(value != MAPPING_BOUNDARY[key] for key, value in _expected_boundary().items())
    classification = classify_integration(
        precondition_ready=bool(state["precondition_ready"]),
        cycles_completed=int(state["cycles_completed"]),
        observation_ok=observation_ok,
        domain_violation=domain_violation,
        fail_closed_ok=bool(negative["passed"]),
        regression_passed=None,
        forbidden_write=bool(forbidden or activation),
    )
    if state["stop_reason"]:
        classification["stop_reason"] = state["stop_reason"]
    if state["stop_error"]:
        classification["stop_error"] = state["stop_error"]
    state["classification"] = classification
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    (EVIDENCE / "raw").mkdir(exist_ok=True)
    _write(EVIDENCE / "architecture.json", _architecture())
    _write(EVIDENCE / "mapping_boundary.json", dict(MAPPING_BOUNDARY))
    _write(EVIDENCE / "observation_contract.json", _contract())
    _write(EVIDENCE / "live_cycles.json", {"cycles": cycles, "precondition": state["precondition"], "precondition_ready": state["precondition_ready"]})
    _write(EVIDENCE / "target_actual.json", _target_actual(cycles))
    _write(EVIDENCE / "negative_tests.json", negative)
    _write(
        EVIDENCE / "write_audit.json",
        {
            "writes_performed": performed,
            "allowed_writes": allowed,
            "activation_writes": activation,
            "forbidden_writes": forbidden,
            "configuration_writes": [],
        },
    )
    _write(EVIDENCE / "regression_results.json", {"full_regression": None, "p022_unit": None, "p021_final_unit": None, "environment_recovery_unit": None})
    _write(EVIDENCE / "classification.json", classification)
    rows = state["raw_rows"]
    assert isinstance(rows, list)
    (EVIDENCE / "raw" / "live.jsonl").write_text("".join(json.dumps(row, ensure_ascii=False) + "\n" for row in rows), encoding="utf-8")
    (EVIDENCE / "raw" / "pytest.txt").write_text("", encoding="utf-8")
    (EVIDENCE / "summary.md").write_text(_summary(state, classification), encoding="utf-8")
    _reject_secret(EVIDENCE)


def _architecture() -> dict[str, object]:
    return {
        "flow": [
            "PowerPmacControllerAdapter",
            "Official Power PMAC Simulator",
            "Motor[1]",
            "ControllerAxisObservation",
            "ControllerObservationBridge",
            "RoboTwin Digital Twin Observation",
            "Controller Target vs Actual",
        ],
        "robot_domain": "rad / meter",
        "controller_domain": "pmac_motor_unit",
        "robot_domain_monitor": "DigitalTwinMonitor remains joint-radian only",
        "bridge_actions": ["read", "normalize", "publish"],
        "bridge_excluded": ["unit conversion", "robot-joint mapping", "motion generation", "activation", "fault reset"],
    }


def _contract() -> dict[str, object]:
    return {
        "fields": [
            "controller_id",
            "axis_id",
            "command_target",
            "desired_position",
            "actual_position",
            "unit",
            "closed_loop",
            "in_position",
            "faulted",
            "fault_code",
            "communication_state",
            "observed_at",
            "source",
            "valid",
            "invalid_reason",
        ],
        "controller_id": "power_pmac_simulator",
        "axis_id": "Motor[1]",
        "unit": "pmac_motor_unit",
        "source": "official_power_pmac_simulator",
    }


def _expected_boundary() -> dict[str, str]:
    return dict(MAPPING_BOUNDARY)


def _target_actual(cycles: list[object]) -> dict[str, object]:
    rows = []
    for cycle in cycles:
        if not isinstance(cycle, dict):
            continue
        observation = cycle["observation"]
        assert isinstance(observation, dict)
        rows.append(
            {
                "cycle": cycle["index"],
                "command_target": observation["command_target"],
                "desired_position": observation["desired_position"],
                "actual_position": observation["actual_position"],
                "tracking_error": cycle["tracking_error"],
                "unit": "pmac_motor_unit",
                "robot_accuracy_claim": False,
            }
        )
    return {"domain": "controller", "rows": rows}


def _summary(state: dict[str, object], classification: dict[str, object]) -> str:
    precondition = state["precondition"] if isinstance(state["precondition"], dict) else {}
    cycles = state["cycles"] if isinstance(state["cycles"], list) else []
    lines = [
        "# P-022 main integration",
        "",
        f"측정: {_now()}",
        "",
        f"classification: {classification['classification']} {classification['classification_name']}",
        f"p022_final_pass_declared: {classification['p022_final_pass_declared']}",
        "",
        "prior environment incident: SIMULATOR_RUNTIME_FAILURE",
        "root cause: NOT_CONFIRMED",
        "clean restart recovery: BR-1 BASELINE_READY",
        "",
        "## Precondition",
        "",
        f"ready: {state['precondition_ready']}",
        f"ServoCtrl: {precondition.get('Motor[1].ServoCtrl', 'NOT_AVAILABLE')}",
        f"ClosedLoop: {precondition.get('Motor[1].ClosedLoop', 'NOT_AVAILABLE')}",
        f"Pos: {precondition.get('Motor[1].Pos', 'NOT_AVAILABLE')}",
        f"FeFatal: {precondition.get('Motor[1].FeFatal', 'NOT_AVAILABLE')}",
        f"AmpFault: {precondition.get('Motor[1].AmpFault', 'NOT_AVAILABLE')}",
        f"LimitStop: {precondition.get('Motor[1].LimitStop', 'NOT_AVAILABLE')}",
        "",
        "## Cycles",
        "",
    ]
    for cycle in cycles:
        if not isinstance(cycle, dict):
            continue
        post = cycle["post"]
        assert isinstance(post, dict)
        lines.append(
            f"C{cycle['index']} target {cycle['target']} command {cycle['command']} "
            f"JogPos {post.get('Motor[1].JogPos')} DesPos {post.get('Motor[1].DesPos')} "
            f"Pos {post.get('Motor[1].Pos')} ActPos {post.get('Motor[1].ActPos')} "
            f"tracking_error {cycle['tracking_error']} completed {cycle['completed']}"
        )
        for index, sample in enumerate(cycle["samples"], start=1):
            if isinstance(sample, dict):
                lines.append(
                    f"  T{index} t_s {sample.get('t_s')} JogPos {sample.get('Motor[1].JogPos')} "
                    f"DesPos {sample.get('Motor[1].DesPos')} Pos {sample.get('Motor[1].Pos')} ActPos {sample.get('Motor[1].ActPos')}"
                )
    lines.extend(
        [
            "",
            "## Claim boundary",
            "",
            "- Official Power PMAC Simulator 1.2.1",
            "- software virtual Motor[1]",
            "- controller-domain Target vs Actual",
            "- PMAC motor units",
            "- ActPos is Simulator virtual feedback",
            "- robot_joint_mapping = NOT_DEFINED",
            "- unit_conversion = NOT_DEFINED",
            "- no real Power PMAC hardware",
            "- no physical motor",
            "- no physical encoder",
            "- no amplifier",
            "- no physical I/O",
            "- no EtherCAT",
            "- no real robot",
            "- no robot joint accuracy claim",
            "- no physical positioning accuracy claim",
            "- no hard-real-time validation",
            "- no controller commissioning",
            "- no industrial safety claim",
            "",
            "Gate: P-022 REVIEW REQUIRED",
            "P-023 started: NO",
            "",
        ]
    )
    return "\n".join(lines)


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


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
