"""P-021A live read-only comparison runner.

#1j= 이외의 configuration write는 보내지 않는다.
ACK를 COMPLETED로 기록하지 않는다.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

from controller.p021a_investigation import (
    SNAPSHOT_VARIABLES,
    build_analysis,
    choose_small_target,
    command_observation,
    explanatory_drift,
    historical_comparison,
    motion_observation,
    parse_snapshot,
    render_environment,
    render_summary,
    snapshot_diff,
    write_evidence,
)
from controller.power_pmac_adapter import PowerPmacControllerAdapter
from controller.power_pmac_config import P017_TEST_CONFIG
from controller.power_pmac_motor import assess_test_motor_readiness
from controller.power_pmac_transport import GpasciiExchange, PowerPmacError, SshGpasciiTransport

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "evidence" / "P-021A"


def read_snapshot(transport: SshGpasciiTransport) -> tuple[dict[str, str], list[str]]:
    exchange = transport.execute_lines(list(SNAPSHOT_VARIABLES))
    parsed = parse_snapshot(exchange.stdout)
    raw = [exchange.stdout]
    for name in [item for item, value in parsed.items() if value == "NOT_AVAILABLE"]:
        try:
            single = transport.execute_lines([name])
        except PowerPmacError:
            continue
        raw.append(single.stdout)
        parsed[name] = parse_snapshot(single.stdout, (name,))[name]
    return parsed, raw


def send_jog(transport: SshGpasciiTransport, target: int) -> tuple[str, dict[str, object]]:
    command = "#1j=" + str(target)
    exchange = transport.execute_lines([command])
    if not isinstance(exchange, GpasciiExchange):
        raise PowerPmacError("PROTOCOL_ERROR", "jog exchange was not parseable")
    return command, command_observation(exchange.stdout, exchange.stderr, exchange.returncode)


def run_trial(adapter: PowerPmacControllerAdapter, transport: SshGpasciiTransport, number: int) -> dict[str, object]:
    rows: list[dict[str, object]] = []
    adapter.connect()
    before, before_raw = read_snapshot(transport)
    readiness = assess_test_motor_readiness(adapter.read_test_motor_state())
    target = choose_small_target(before.get("Motor[1].Pos"))
    command, response = send_jog(transport, target)
    after, after_raw = read_snapshot(transport)
    motion = motion_observation(before, after, target)
    fresh = {
        "readiness": readiness,
        "target": target,
        "command": command,
        "response": response,
        "before": before,
        "after": after,
        "before_raw": before_raw,
        "after_raw": after_raw,
        "motion": motion,
        "configuration_write": False,
    }
    rows.append({"step": "fresh", "command": command, "readiness": readiness, "motion": motion, "response": response})
    reconnect: dict[str, object] | None = None
    drift: list[str] = []
    reconnect_reached: bool | None = None
    if motion["reached"] is True:
        adapter.disconnect()
        blocked = adapter.move_test_motor(4)
        rows.append(
            {
                "step": "disconnected_command",
                "result": blocked.result,
                "error_code": blocked.error_code,
                "command_sent": blocked.command_sent,
            }
        )
        adapter.connect()
        reconnect_before, reconnect_before_raw = read_snapshot(transport)
        reconnect_ready = assess_test_motor_readiness(adapter.read_test_motor_state())
        reconnect_target = choose_small_target(reconnect_before.get("Motor[1].Pos"), avoid=(target,))
        reconnect_command, reconnect_response = send_jog(transport, reconnect_target)
        reconnect_after, reconnect_after_raw = read_snapshot(transport)
        reconnect_motion = motion_observation(reconnect_before, reconnect_after, reconnect_target)
        drift = explanatory_drift(before, reconnect_before)
        reconnect_reached = bool(reconnect_motion["reached"])
        reconnect = {
            "disconnect": "adapter.disconnect",
            "simulator_process_stopped": False,
            "disconnected_result": blocked.result,
            "disconnected_command_sent": blocked.command_sent,
            "readiness": reconnect_ready,
            "target": reconnect_target,
            "command": reconnect_command,
            "response": reconnect_response,
            "before": reconnect_before,
            "after": reconnect_after,
            "before_raw": reconnect_before_raw,
            "after_raw": reconnect_after_raw,
            "motion": reconnect_motion,
            "explanatory_drift": drift,
            "configuration_write": False,
        }
        rows.append(
            {
                "step": "reconnect",
                "command": reconnect_command,
                "readiness": reconnect_ready,
                "motion": reconnect_motion,
                "response": reconnect_response,
                "drift": drift,
            }
        )
    return {
        "trial": number,
        "fresh_motion_reached": bool(motion["reached"]),
        "reconnect_ran": reconnect is not None,
        "reconnect_motion_reached": reconnect_reached,
        "explanatory_drift": drift,
        "fresh": fresh,
        "reconnect": reconnect,
        "rows": rows,
        "configuration_writes": [],
    }


def main() -> None:
    transport = SshGpasciiTransport(P017_TEST_CONFIG)
    adapter = PowerPmacControllerAdapter(transport)
    trials: list[dict[str, object]] = []
    for number in (1, 2):
        try:
            trials.append(run_trial(adapter, transport, number))
        except PowerPmacError as exc:
            trials.append(
                {
                    "trial": number,
                    "error_code": exc.code,
                    "fresh_motion_reached": None,
                    "reconnect_ran": False,
                    "reconnect_motion_reached": None,
                    "explanatory_drift": [],
                    "fresh": {},
                    "reconnect": None,
                    "rows": [{"step": "error", "error_code": exc.code}],
                    "configuration_writes": [],
                }
            )
    analysis = build_analysis(trials, None)
    matrix = []
    diffs = []
    for trial in trials:
        fresh = trial.get("fresh")
        reconnect = trial.get("reconnect")
        if not isinstance(fresh, dict) or "before" not in fresh:
            continue
        entry = {
            "trial": trial["trial"],
            "a_before": fresh["before"],
            "a_after": fresh["after"],
            "b_before": None if not isinstance(reconnect, dict) else reconnect["before"],
            "b_after": None if not isinstance(reconnect, dict) else reconnect["after"],
        }
        matrix.append(entry)
        diffs.append(
            {
                "trial": trial["trial"],
                "rows": snapshot_diff(
                    fresh["before"],
                    fresh["after"],
                    None if not isinstance(reconnect, dict) else reconnect["before"],
                    None if not isinstance(reconnect, dict) else reconnect["after"],
                ),
            }
        )
    history = historical_comparison()
    history["P-021A"] = [
        {
            "trial": trial["trial"],
            "fresh_command": None if not isinstance(trial.get("fresh"), dict) else trial["fresh"].get("command"),
            "fresh_reached": trial.get("fresh_motion_reached"),
            "reconnect_command": None if not isinstance(trial.get("reconnect"), dict) else trial["reconnect"].get("command"),
            "reconnect_reached": trial.get("reconnect_motion_reached"),
        }
        for trial in trials
    ]
    payload = {
        "trials": trials,
        "snapshot_matrix": matrix,
        "snapshot_diff": diffs,
        "historical_comparison": history,
        "root_cause_analysis": analysis,
        "environment_md": render_environment(),
        "summary_md": render_summary(analysis, [trial for trial in trials if isinstance(trial.get("fresh"), dict) and trial["fresh"]]),
    }
    write_evidence(EVIDENCE, payload)
    _reject_secret(EVIDENCE)
    print(json.dumps({"classification": analysis["classification"], "confidence": analysis["confidence"]}, ensure_ascii=False))


def _reject_secret(root: Path) -> None:
    secret = os.environ.get("PMAC_SSH_PASSWORD", "")
    if not secret:
        return
    for path in root.rglob("*"):
        if path.is_file() and secret in path.read_text(encoding="utf-8", errors="replace"):
            raise SystemExit("secret scan failed")


if __name__ == "__main__":
    main()
