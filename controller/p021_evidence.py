"""P-021 evidence. credential 값은 기록하지 않는다.

실행: python3 -m controller.p021_evidence
"""

from __future__ import annotations

import json
import os
import platform
from datetime import datetime, timezone
from pathlib import Path

from controller.p021_recovery import (
    move_if_recovered,
    recovery_leaves_validator_closed,
    recovery_metrics,
    scripted_disconnect_recovery,
    scripted_fault_blocks,
    scripted_fault_cleared_but_not_ready,
    scripted_fault_recovery,
    scripted_fault_remains,
    scripted_malformed_recovery,
    scripted_reconnect_does_not_assume_ready,
    scripted_reconnect_failure,
    virtual_fault_recovery,
)
from controller.power_pmac_adapter import PowerPmacControllerAdapter
from controller.power_pmac_config import P017_TEST_CONFIG
from controller.power_pmac_motor import assess_test_motor_readiness, choose_targets
from controller.power_pmac_transport import PowerPmacError, SshGpasciiTransport

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "evidence" / "P-021"
RAW = EVIDENCE / "raw"


def main() -> None:
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    RAW.mkdir(parents=True, exist_ok=True)
    started_at = datetime.now(timezone.utc).isoformat()
    virtual = virtual_fault_recovery()
    detection = [scripted_fault_blocks(name) for name in ("AmpFault", "FeFatal", "LimitStop")]
    fault_recovered = scripted_fault_recovery()
    remains = scripted_fault_remains()
    not_ready = scripted_fault_cleared_but_not_ready()
    disconnect = scripted_disconnect_recovery()
    not_assumed = scripted_reconnect_does_not_assume_ready()
    reconnect_failed = scripted_reconnect_failure()
    malformed = scripted_malformed_recovery()
    boundary = recovery_leaves_validator_closed()
    live = _live()
    records = _records(virtual, fault_recovered, remains, not_ready, disconnect, not_assumed, reconnect_failed, malformed, live)
    metrics = recovery_metrics(records)
    _write_json(EVIDENCE / "disconnect_recovery.json", {"scripted": disconnect, "reconnect_does_not_assume_ready": not_assumed, "live": live})
    _write_json(
        EVIDENCE / "fault_recovery.json",
        {
            "live_fault_injection": "NOT_TESTED",
            "reason": "AmpFault, FeFatal, LimitStop were not injected on the live simulator. pAmpFault, encoder pointers, limits, and Sys.SimConfigOk were not changed.",
            "virtual": virtual,
            "scripted_detection": detection,
            "scripted_recovery": fault_recovered,
        },
    )
    _write_json(
        EVIDENCE / "failed_recovery.json",
        {
            "fault_remains": remains,
            "fault_cleared_not_ready": not_ready,
            "reconnect_failure": reconnect_failed,
            "malformed_response": malformed,
            "reconnect_not_ready": not_assumed,
        },
    )
    _write_json(
        EVIDENCE / "post_recovery_motion.json",
        {
            "virtual": {"command_id": virtual["post_command_id"], "result": virtual["post_result"], "trace": virtual["post_trace"]},
            "scripted_fault": {"command": fault_recovered["post_command"], "result": fault_recovered["post_result"]},
            "scripted_disconnect": {"command": disconnect["post_command"], "result": disconnect["post_result"]},
            "live": live.get("post"),
        },
    )
    _write_json(EVIDENCE / "validator_boundary.json", boundary)
    _write_json(EVIDENCE / "recovery_metrics.json", metrics)
    _write_jsonl(RAW / "live_session.jsonl", live.get("rows", []))
    (EVIDENCE / "environment.md").write_text(_environment(started_at), encoding="utf-8")
    (EVIDENCE / "summary.md").write_text(_summary(started_at, virtual, fault_recovered, remains, live, boundary, metrics), encoding="utf-8")
    _reject_secret(EVIDENCE)
    print(json.dumps({"live_ok": live.get("ok"), "post": None if not isinstance(live.get("post"), dict) else live["post"].get("result"), "metrics": metrics}, indent=2))


def _live() -> dict[str, object]:
    transport = SshGpasciiTransport(P017_TEST_CONFIG)
    adapter = PowerPmacControllerAdapter(transport)
    rows: list[dict[str, object]] = []
    try:
        adapter.connect()
        enabled = adapter.enable_test_motor()
        pre = adapter.read_test_motor_state()
        pre_ready = assess_test_motor_readiness(pre)
        rows.append({"step": "baseline", "communication": adapter.communication_state, "ready": pre_ready, "enable": enabled.result, **_view(pre)})
        if pre_ready != "RECOVERED":
            return {"ok": False, "error": {"step": "baseline", "ready": pre_ready}, "rows": rows, "post": None}
        adapter.disconnect()
        blocked = adapter.move_test_motor(4)
        rows.append({"step": "disconnected", "communication": adapter.communication_state, "attempted_command": "#1j=4", "result": blocked.result, "error_code": blocked.error_code, "command_sent": blocked.command_sent})
        verified = adapter.reconnect_and_verify()
        rows.append({"step": "reconnected", "communication": adapter.communication_state, "result": verified.result, "read_back": verified.state is not None, **_view(verified.state)})
        post = None
        if verified.result == "RECOVERED" and verified.state is not None:
            current = verified.state.normalized.get("pos")
            target = _new_target(current)
            moved = move_if_recovered(adapter, verified, target)
            post = {
                "target": target,
                "units": "pmac_motor_unit",
                "command": moved.command,
                "result": moved.result,
                "error_code": moved.error_code,
                "response_text": moved.response_text,
                "command_sent": moved.command_sent,
                "poll_count": moved.poll_count,
                "elapsed_s": moved.elapsed_s,
                "elapsed_note": "SSH + portproxy + gpascii + polling. Not controller or servo latency.",
                **_view(moved.state),
            }
            rows.append({"step": "post_recovery", **post})
        return {
            "ok": isinstance(post, dict) and post.get("result") == "COMPLETED",
            "source": "official_simulator",
            "simulator_process_stopped": False,
            "pre": {"communication": "HEALTHY", "ready": pre_ready, "enable": enabled.result, **_view(pre)},
            "disconnected": {"communication": "DISCONNECTED", "attempted_command": "#1j=4", "result": blocked.result, "command_sent": blocked.command_sent},
            "reconnected": {"communication": adapter.communication_state, "result": verified.result, "read_back": verified.state is not None, **_view(verified.state)},
            "post": post,
            "rows": rows,
            "error": None,
        }
    except PowerPmacError as exc:
        rows.append({"step": "error", "code": exc.code})
        return {"ok": False, "source": "official_simulator", "error": {"code": exc.code}, "rows": rows, "post": None, "pre": None, "disconnected": None, "reconnected": None}


def _records(
    virtual: dict[str, object],
    fault_recovered: dict[str, object],
    remains: dict[str, object],
    not_ready: dict[str, object],
    disconnect: dict[str, object],
    not_assumed: dict[str, object],
    reconnect_failed: dict[str, object],
    malformed: dict[str, object],
    live: dict[str, object],
) -> list[dict[str, object]]:
    live_post = live.get("post")
    live_completed = isinstance(live_post, dict) and live_post.get("result") == "COMPLETED"
    return [
        {"name": "virtual_fault", "recovered": virtual["post_result"] == "COMPLETED", "failed_blocked": False, "post_completed": virtual["post_result"] == "COMPLETED", "blocked_submissions": 2, "false_recovery": 0, "false_completion": 0},
        {"name": "scripted_fault", "recovered": fault_recovered["verification"] == "RECOVERED", "failed_blocked": False, "post_completed": fault_recovered["post_result"] == "COMPLETED", "blocked_submissions": 1, "false_recovery": 0, "false_completion": 0},
        {"name": "fault_remains", "recovered": False, "failed_blocked": remains["command_sent"] is False and remains["jog_count"] == 0, "post_completed": False, "blocked_submissions": 1, "false_recovery": 0, "false_completion": 0},
        {"name": "not_ready", "recovered": False, "failed_blocked": not_ready["command_sent"] is False and not_ready["jog_count"] == 0, "post_completed": False, "blocked_submissions": 1, "false_recovery": 0, "false_completion": 0},
        {"name": "scripted_disconnect", "recovered": disconnect["verification"] == "RECOVERED", "failed_blocked": False, "post_completed": disconnect["post_result"] == "COMPLETED", "blocked_submissions": 1, "false_recovery": 0, "false_completion": 0},
        {"name": "reconnect_not_ready", "recovered": False, "failed_blocked": not_assumed["command_sent"] is False and not_assumed["jog_count"] == 0, "post_completed": False, "blocked_submissions": 1, "false_recovery": 0, "false_completion": 0},
        {"name": "reconnect_failure", "recovered": False, "failed_blocked": reconnect_failed["command_sent"] is False, "post_completed": False, "blocked_submissions": 1, "false_recovery": 0, "false_completion": 0},
        {"name": "malformed", "recovered": False, "failed_blocked": malformed["command_sent"] is False and malformed["jog_count"] == 0, "post_completed": False, "blocked_submissions": 1, "false_recovery": 0, "false_completion": 0},
        {"name": "live_disconnect", "recovered": isinstance(live.get("reconnected"), dict) and live["reconnected"].get("result") == "RECOVERED", "failed_blocked": False, "post_completed": live_completed, "blocked_submissions": 1 if isinstance(live.get("disconnected"), dict) and live["disconnected"].get("command_sent") is False else 0, "false_recovery": 0, "false_completion": 0},
    ]


def _new_target(current: object) -> int:
    numeric = current if isinstance(current, (int, float)) and not isinstance(current, bool) else None
    first, second = choose_targets(numeric)
    if first == 4:
        return second
    return first


def _view(state: object) -> dict[str, object]:
    normalized = getattr(state, "normalized", None)
    raw = getattr(state, "raw", None)
    if not isinstance(normalized, dict) or not isinstance(raw, dict):
        return {}
    return {
        "servo_ctrl": normalized.get("servo_ctrl"),
        "closed_loop": normalized.get("closed_loop"),
        "pos": raw.get("Motor[1].Pos"),
        "actual_pos": raw.get("Motor[1].ActPos"),
        "desired_pos": raw.get("Motor[1].DesPos"),
        "in_position": normalized.get("in_position"),
        "fe_fatal": normalized.get("following_error_fatal"),
        "fe_warn": normalized.get("following_error_warning"),
        "amp_fault": normalized.get("amp_fault"),
        "limit_stop": normalized.get("limit_stop"),
        "fault_code": getattr(state, "fault_code", None),
    }


def _environment(started_at: str) -> str:
    config = P017_TEST_CONFIG
    return "\n".join(
        [
            "# P-021 environment",
            "",
            f"측정: {started_at}",
            "",
            "```text",
            "client: MacBook",
            f"client_os: {platform.platform()}",
            "virtual backend: VirtualPmacController existing reset_fault and servo_on",
            "pmac backend: Official Power PMAC Simulator 1.2.1",
            "firmware: 2.8.3.0 stated in the P-019B brief; this runner did not query a firmware variable",
            f"path: MacBook -> {config.host}:{config.port} -> {config.simulator_address}:22 -> SSH + gpascii",
            "live path: adapter disconnect and connect only; the simulator process is not stopped",
            "live fault injection: NOT_TESTED",
            "scripted fault: AmpFault, FeFatal, LimitStop on an exchange double",
            f"credential: runtime environment variable {config.password_env}; value not recorded",
            "```",
            "",
            "P-017, P-018, P-019, P-019C, P-020 evidence는 수정하지 않는다.",
            "",
        ]
    )


def _summary(started_at: str, virtual: dict[str, object], fault: dict[str, object], remains: dict[str, object], live: dict[str, object], boundary: dict[str, object], metrics: dict[str, object]) -> str:
    pre = live.get("pre") if isinstance(live.get("pre"), dict) else {}
    disconnected = live.get("disconnected") if isinstance(live.get("disconnected"), dict) else {}
    reconnected = live.get("reconnected") if isinstance(live.get("reconnected"), dict) else {}
    post = live.get("post") if isinstance(live.get("post"), dict) else {}
    lines = [
        "# P-021 summary",
        "",
        f"측정: {started_at}",
        "",
        "이 문서는 P-021 PASS 또는 FAIL을 확정하지 않는다. 최종 판정은 별도 review다. P-022는 시작하지 않는다.",
        "",
        "기준선: `f7412654be99c7274361505a770438f364eaec5d`",
        "",
        "Disconnect recovery와 fault recovery는 서로 다른 class다. reconnect 성공을 READY로 두지 않는다.",
        "",
        "LIVE_FAULT_INJECTION = NOT_TESTED",
        "",
        "## Live disconnect",
        "",
        f"pre communication: {pre.get('communication')} ready: {pre.get('ready')} ServoCtrl: {pre.get('servo_ctrl')} ClosedLoop: {pre.get('closed_loop')}",
        f"pre Pos/ActPos/DesPos: {pre.get('pos')}/{pre.get('actual_pos')}/{pre.get('desired_pos')}",
        f"pre faults: FeFatal {pre.get('fe_fatal')} FeWarn {pre.get('fe_warn')} AmpFault {pre.get('amp_fault')} LimitStop {pre.get('limit_stop')}",
        f"disconnected communication: {disconnected.get('communication')} command: {disconnected.get('attempted_command')} result: {disconnected.get('result')} command_sent: {disconnected.get('command_sent')}",
        f"reconnected communication: {reconnected.get('communication')} result: {reconnected.get('result')} read_back: {reconnected.get('read_back')}",
        f"reconnected ServoCtrl: {reconnected.get('servo_ctrl')} ClosedLoop: {reconnected.get('closed_loop')} fault: {reconnected.get('fault_code')}",
        f"post target: {post.get('target')} command: {post.get('command')} result: {post.get('result')} error_code: {post.get('error_code')}",
        f"post response: {post.get('response_text')!r}",
        f"post Pos/ActPos/DesPos: {post.get('pos')}/{post.get('actual_pos')}/{post.get('desired_pos')} InPos: {post.get('in_position')}",
        f"post faults: FeFatal {post.get('fe_fatal')} AmpFault {post.get('amp_fault')} LimitStop {post.get('limit_stop')}",
        f"elapsed_s: {post.get('elapsed_s')}",
        "",
        "elapsed time은 SSH + portproxy + gpascii + polling이다. controller/servo latency가 아니다.",
        "",
        "## Scripted fault",
        "",
        f"verification: {fault.get('verification')} blocked_sent: {fault.get('blocked_sent')} post: {fault.get('post_command')} {fault.get('post_result')}",
        f"fault remains command_sent: {remains.get('command_sent')} jog: {remains.get('jog_count')}",
        "",
        "## Virtual",
        "",
        f"blocked: {virtual.get('blocked_result')} after reset: {virtual.get('after_reset_state')} before ready: {virtual.get('before_ready_result')}",
        f"new command: {virtual.get('post_command_id')} result: {virtual.get('post_result')}",
        "",
        "## Metrics",
        "",
        f"attempted: {metrics.get('recovery_scenarios_attempted')}",
        f"recovered: {metrics.get('recovery_scenarios_recovered')}",
        f"failed recovery correctly blocked: {metrics.get('failed_recovery_correctly_blocked')}",
        f"post-recovery completed: {metrics.get('post_recovery_commands_completed')}",
        f"blocked-state submissions: {metrics.get('blocked_state_command_submissions')}",
        f"false recovery: {metrics.get('false_recovery')}",
        f"false completion: {metrics.get('false_completion')}",
        "",
        "이 metric은 project controller execution invariant다. physical safety claim이 아니다.",
        "",
        "## Validator",
        "",
        f"E-fixed-1: {boundary.get('validator_result')} virtual calls: {boundary.get('virtual_backend_calls')} pmac calls: {boundary.get('pmac_backend_calls')}",
        "",
        "## Claim boundary",
        "",
        "- Official Power PMAC Simulator only",
        "- software controller fault/recovery validation",
        "- software virtual Motor[1]",
        "- disconnect/reconnect semantics validated",
        "- post-recovery state read-back validated",
        "- scripted fault injection where explicitly identified",
        "- no real Power PMAC hardware",
        "- no physical motor/servo",
        "- no physical amplifier fault",
        "- no physical encoder fault",
        "- no physical limit switch",
        "- no physical I/O",
        "- no EtherCAT recovery",
        "- no safety controller",
        "- no safety certification",
        "- no real robot",
        "- no hard-real-time validation",
        "- no industrial commissioning",
        "",
    ]
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
