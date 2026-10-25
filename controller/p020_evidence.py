"""P-020 evidence. credential 값은 기록하지 않는다.

실행: python3 -m controller.p020_evidence
"""

from __future__ import annotations

import json
import os
import platform
from datetime import datetime, timezone
from pathlib import Path

from controller.p020_cross_controller import (
    build_matrix,
    e_fixed_boundary,
    negative_matrix,
    pmac_move_observation,
    state_mapping,
)
from controller.power_pmac_adapter import PowerPmacControllerAdapter
from controller.power_pmac_config import P017_TEST_CONFIG
from controller.power_pmac_motor import choose_targets
from controller.power_pmac_transport import PowerPmacError, SshGpasciiTransport

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "evidence" / "P-020"
RAW = EVIDENCE / "raw"


def main() -> None:
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    RAW.mkdir(parents=True, exist_ok=True)
    started_at = datetime.now(timezone.utc).isoformat()
    live = _live()
    boundary = e_fixed_boundary()
    observation = live["observation"]
    assert isinstance(observation, dict)
    virtual, rows = build_matrix(observation, boundary)
    _annotate(rows, live["ok"] is True)
    payload = {
        "virtual": virtual,
        "live": live,
        "rows": rows,
        "boundary": boundary,
    }
    _write_json(EVIDENCE / "comparison_matrix.json", {"rows": rows})
    _write_json(EVIDENCE / "positive_live.json", {"virtual": virtual, "power_pmac": live})
    _write_json(EVIDENCE / "negative_matrix.json", negative_matrix(rows))
    _write_json(EVIDENCE / "validator_boundary.json", boundary)
    _write_json(EVIDENCE / "state_mapping.json", state_mapping(virtual, observation, rows))
    _write_jsonl(RAW / "live_session.jsonl", live["rows"])
    (EVIDENCE / "environment.md").write_text(_environment(started_at), encoding="utf-8")
    (EVIDENCE / "summary.md").write_text(_summary(started_at, payload), encoding="utf-8")
    _reject_secret(EVIDENCE)
    print(json.dumps({"live_ok": live["ok"], "target": live.get("target"), "result": live.get("result"), "rows": len(rows)}, indent=2))


def _live() -> dict[str, object]:
    transport = SshGpasciiTransport(P017_TEST_CONFIG)
    adapter = PowerPmacControllerAdapter(transport)
    rows: list[dict[str, object]] = []
    try:
        adapter.connect()
        rows.append({"step": "connect", "communication_state": adapter.communication_state})
        enabled = adapter.enable_test_motor()
        rows.append({"step": "enable", "result": enabled.result, "error_code": enabled.error_code, "activation_write": enabled.activation_write, "enable_command_sent": enabled.enable_command_sent})
        if adapter.communication_state != "HEALTHY" or enabled.result != "COMPLETED":
            return _failed("enable", enabled.error_code or enabled.result, rows)
        pre = adapter.read_test_motor_state()
        current = pre.normalized.get("pos")
        target, _other = choose_targets(current if isinstance(current, (int, float)) and not isinstance(current, bool) else None)
        result = adapter.move_test_motor(target)
        observed = pmac_move_observation(adapter.communication_state, result)
        observed["ready"] = pre.normalized.get("servo_ctrl") not in (None, 0) and pre.normalized.get("closed_loop") is True
        observed["servo_ctrl"] = pre.normalized.get("servo_ctrl")
        observed["closed_loop"] = result.state.normalized.get("closed_loop") if result.state is not None else pre.normalized.get("closed_loop")
        rows.append(
            {
                "step": "move",
                "target": target,
                "pre_pos": pre.raw.get("Motor[1].Pos"),
                "pre_act": pre.raw.get("Motor[1].ActPos"),
                "pre_des": pre.raw.get("Motor[1].DesPos"),
                "command": result.command,
                "result": result.result,
                "response": result.response_text,
                "poll_count": result.poll_count,
                "elapsed_s": result.elapsed_s,
            }
        )
        return {
            "ok": result.result == "COMPLETED",
            "source": "official_simulator",
            "target": target,
            "pre": {
                "pos": pre.raw.get("Motor[1].Pos"),
                "actual_pos": pre.raw.get("Motor[1].ActPos"),
                "desired_pos": pre.raw.get("Motor[1].DesPos"),
            },
            "result": result.result,
            "error_code": result.error_code,
            "command": result.command,
            "response_text": result.response_text,
            "poll_count": result.poll_count,
            "elapsed_s": result.elapsed_s,
            "elapsed_note": "SSH + portproxy + gpascii + polling. Not controller or servo latency.",
            "observation": observed,
            "rows": rows,
            "error": None,
        }
    except PowerPmacError as exc:
        rows.append({"step": "error", "code": exc.code})
        return _failed(exc.code, exc.code, rows)


def _failed(step: str, code: str | None, rows: list[dict[str, object]]) -> dict[str, object]:
    return {
        "ok": False,
        "source": "official_simulator",
        "target": None,
        "result": code,
        "error_code": code,
        "command": None,
        "response_text": "",
        "poll_count": 0,
        "elapsed_s": None,
        "observation": {
            "communication_state": None,
            "ready": False,
            "result": code,
            "error_code": code,
            "stages": [],
            "accepted": False,
            "completed": False,
            "command_sent": False,
            "command": None,
            "target_observable": False,
            "actual_observable": False,
            "fault_free": False,
            "desired_pos": None,
            "pos": None,
            "actual_pos": None,
            "units": "pmac_motor_unit",
            "response_text": "",
            "poll_count": 0,
            "elapsed_s": None,
            "closed_loop": None,
            "in_position": None,
            "fault_code": None,
            "raw_pos": None,
            "raw_act": None,
            "raw_des": None,
            "fe_fatal": None,
            "fe_warn": None,
            "amp_fault": None,
            "limit_stop": None,
            "servo_ctrl": None,
            "activation_write": False,
            "enable_command_sent": False,
        },
        "rows": rows,
        "error": {"step": step, "code": code},
    }


def _annotate(rows: list[dict[str, object]], live_ok: bool) -> None:
    del live_ok
    for row in rows:
        scenario = row.get("scenario")
        if scenario == "positive":
            row["virtual_source"] = "VirtualPmacController"
            row["power_pmac_source"] = "official_simulator"
        elif scenario == "validator_boundary":
            row["virtual_source"] = "ExecutionGate"
            row["power_pmac_source"] = "not_called"
        else:
            row["virtual_source"] = "VirtualPmacController"
            row["power_pmac_source"] = "scripted_exchange"


def _environment(started_at: str) -> str:
    config = P017_TEST_CONFIG
    return "\n".join(
        [
            "# P-020 environment",
            "",
            f"측정: {started_at}",
            "",
            "```text",
            "client: MacBook",
            f"client_os: {platform.platform()}",
            "virtual backend: VirtualPmacController",
            "pmac backend: Official Power PMAC Simulator 1.2.1",
            "firmware: 2.8.3.0 stated in the P-019B brief; this runner did not query a firmware variable",
            f"path: MacBook -> {config.host}:{config.port} -> {config.simulator_address}:22 -> SSH + gpascii",
            "live scope: Motor[1] one small motor-unit target",
            "negative paths: scripted exchange or disconnected adapter; the simulator process is not stopped",
            f"credential: runtime environment variable {config.password_env}; value not recorded",
            "```",
            "",
            "P-017, P-018, P-019, P-019C evidence는 수정하지 않는다.",
            "",
        ]
    )


def _summary(started_at: str, payload: dict[str, object]) -> str:
    virtual = payload["virtual"]
    live = payload["live"]
    boundary = payload["boundary"]
    rows = payload["rows"]
    assert isinstance(virtual, dict)
    assert isinstance(live, dict)
    assert isinstance(boundary, dict)
    assert isinstance(rows, list)
    observed = live.get("observation")
    assert isinstance(observed, dict)
    lines = [
        "# P-020 summary",
        "",
        f"측정: {started_at}",
        "",
        "이 문서는 P-020 PASS 또는 FAIL을 확정하지 않는다. 최종 판정은 별도 review다. P-021은 시작하지 않는다.",
        "",
        "기준선: `bb58ea3a03cb7147fbc14e3ea59f8fe35d79a2c6`",
        "",
        "비교는 semantic이다. Virtual radian과 PMAC motor unit의 숫자는 비교하지 않는다.",
        "",
        "## Virtual",
        "",
        f"operation: {virtual.get('operation')} {virtual.get('configuration')}",
        f"result: submit {virtual.get('submit_result')} execution {virtual.get('execution_state')}",
        f"target observable: {virtual.get('target_observable')} units {virtual.get('units')}",
        f"actual observable: {virtual.get('actual_observable')}",
        f"fault: {virtual.get('fault')}",
        "",
        "## Power PMAC",
        "",
        f"live ok: {live.get('ok')}",
        f"target: {live.get('target')} pmac motor unit",
        f"command: {live.get('command')}",
        f"result: {live.get('result')}",
        f"Pos: {observed.get('raw_pos')} ActPos: {observed.get('raw_act')} DesPos: {observed.get('raw_des')}",
        f"ClosedLoop: {observed.get('closed_loop')} InPos: {observed.get('in_position')}",
        f"FeFatal: {observed.get('fe_fatal')} FeWarn: {observed.get('fe_warn')} AmpFault: {observed.get('amp_fault')} LimitStop: {observed.get('limit_stop')}",
        f"polls: {live.get('poll_count')} elapsed_s: {live.get('elapsed_s')}",
        "",
        "elapsed time은 SSH + portproxy + gpascii + polling이다. controller/servo latency가 아니다.",
        "",
        "## Comparison",
        "",
    ]
    for semantic in ("command_acceptance", "completion", "target_observable", "actual_observable", "fault_free", "numeric_position_equality"):
        row = next(item for item in rows if item.get("scenario") == "positive" and item.get("semantic") == semantic)
        lines.append(f"- {semantic}: {row.get('comparison')}")
    lines.extend(["", "## Validator", "", f"E-fixed-1: {boundary.get('validator_result')}", f"virtual calls: {boundary.get('virtual_backend_calls')}", f"pmac calls: {boundary.get('pmac_backend_calls')}", "", "## Claim boundary", ""])
    lines.extend(
        [
            "- Cross-controller semantic validation only",
            "- VirtualPmacController vs Official Power PMAC Simulator software backend",
            "- not numeric position equivalence",
            "- not radian ↔ PMAC unit calibration",
            "- not trajectory equivalence",
            "- not robot-joint mapping",
            "- not UR10e controller validation",
            "- Official Power PMAC Simulator only",
            "- software virtual Motor[1] only",
            "- no real Power PMAC hardware",
            "- no physical motor/servo",
            "- no physical encoder",
            "- no EtherCAT",
            "- no physical I/O",
            "- no real robot",
            "- no hard-real-time validation",
            "- no industrial commissioning",
            "",
        ]
    )
    return "\n".join(lines)


def _write_json(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _write_jsonl(path: Path, rows: object) -> None:
    if not isinstance(rows, list):
        rows = []
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
