"""한 컨트롤러 세션에서 canonical workload를 반복한다."""

from __future__ import annotations

import json
import math
import os
import subprocess
import time
from pathlib import Path

from controller.evidence import DT_S
from controller.virtual_pmac import COMPLETION_TOLERANCE_RAD, JointCommand, VirtualPmacController
from core.model.ur10e import load_ur10e_model
from digital_twin.monitor import DigitalTwinMonitor, load_monitor_config
from digital_twin.session import vector
from hmi.model import HmiSnapshot
from hmi.view_model import initial_snapshot, snapshot_from_run, snapshot_from_validation
from reliability.metrics import classify_cycle
from reliability.model import EXPECTED, NONZERO_SAFE, load_reliability_config
from validator.gate import ExecutionGate
from validator.validate import load_validator_config
from workcell.layout import resolve_workcell

ZERO = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


class ReliabilitySession:
    def __init__(self, config: dict | None = None) -> None:
        self.config = load_reliability_config() if config is None else config
        self.model = load_ur10e_model()
        self.workcell = resolve_workcell()
        self.validator_config = load_validator_config()
        self.monitor_config = load_monitor_config()
        if float(self.monitor_config["sample_period_sec"]) != DT_S:
            raise ValueError("monitor sample period must stay on the P-006 project clock")
        if float(self.config["sample_period_sec"]) != DT_S:
            raise ValueError("reliability sample period must stay on the P-006 project clock")
        self.controller = VirtualPmacController(self.model)
        self.controller.connect()
        self.controller.servo_on()
        self.gate = ExecutionGate(self.controller)
        self.view = initial_snapshot()
        self.last_execute_enabled = False
        self.last_case: str | None = None
        self.rows: list[dict] = []
        self.recovery: list[dict] = []
        self.checkpoints: list[dict] = []
        self.stopped = False
        self.stop_reason: str | None = None
        self.first_failure_cycle: int | None = None
        self.first_failure_category: str | None = None
        self.first_failure_details: dict | None = None

    def run_primary(self, jsonl_path: Path | None = None) -> None:
        iterations = int(self.config["iterations"])
        sequence = tuple(self.config["sequence"])
        started = time.perf_counter()
        self._checkpoint(0, started)
        cycle_id = 0
        handle = jsonl_path.open("a", encoding="utf-8") if jsonl_path is not None else None
        try:
            for iteration in range(1, iterations + 1):
                for case_name in sequence:
                    cycle_id += 1
                    row = self._primary_cycle(cycle_id, iteration, case_name)
                    self.rows.append(row)
                    if handle is not None:
                        handle.write(_json_line(row))
                        handle.flush()
                    if row["result"] != "PASS":
                        self._note_failure(row)
                    if self.stopped:
                        break
                    if cycle_id % 100 == 0:
                        self._checkpoint(cycle_id, started)
                if self.stopped:
                    self._checkpoint(cycle_id, started)
                    break
        finally:
            if handle is not None:
                handle.close()
        self.wall_runtime_sec = time.perf_counter() - started

    def run_recovery(self) -> None:
        attempts = int(self.config["recovery_attempts"])
        case_name = str(self.config["recovery_case"])
        for index in range(1, attempts + 1):
            if self.stopped and self.stop_reason == "UNSAFE_SUBMISSION":
                self.recovery.append({"attempt": index, "result": "FAIL", "reason": "primary run stopped before recovery"})
                continue
            self.recovery.append(self._recovery_attempt(index, case_name))

    def _primary_cycle(self, cycle_id: int, iteration: int, case_name: str) -> dict:
        stale = bool(self.last_execute_enabled and self.last_case not in (None, case_name))
        self.view = initial_snapshot()
        self.view.case_name = case_name
        self.view.command_id = case_name
        pre = self._capture("pre")
        leak = self._precondition_leak(pre)
        if leak is not None:
            self.stopped = True
            self.stop_reason = "STATE_LEAK"
            return self._failed_row(cycle_id, iteration, case_name, pre, leak, stale)
        outcome = self._run_move(f"p011-{cycle_id}-{case_name}", case_name, vector(case_name), None)
        post = self._capture("post")
        row = self._row_from_move(cycle_id, iteration, case_name, pre, post, outcome, stale)
        self.last_execute_enabled = bool(row["hmi_execute_enabled"])
        self.last_case = case_name
        if row["result"] != "PASS" and row["failure_category"] in ("UNSAFE_SUBMISSION", "STATE_LEAK"):
            self.stopped = True
            self.stop_reason = row["failure_category"]
            return row
        restore = self._restore_known_start(cycle_id)
        row["reset_required"] = restore["required"]
        row["reset_result"] = restore["result"]
        if restore["result"] == "FAILED":
            row["failure_category"] = "RESET_FAILURE"
            row["result"] = "FAIL"
            row["details"] = restore
            self.stopped = True
            self.stop_reason = "RESET_FAILURE"
        return row

    def _run_move(self, command_id: str, case_name: str, targets: tuple[float, ...], fault_after_steps: int | None) -> dict:
        state = self.controller.get_state()
        command = JointCommand(command_id, "MOVE_JOINT", targets)
        before = self.gate.submit_count
        try:
            report = self.gate.consider(command, state, self.workcell, self.validator_config)
        except Exception as exc:
            return {"internal_error": str(exc), "validation_decision": "REJECT", "controller_submitted": False}
        monitor = DigitalTwinMonitor(self.model, self.monitor_config)
        timed_out = False
        if report["decision"] != "SAFE_TO_ATTEMPT" or report["controller_result"]["result"] != "ACCEPTED":
            if report["decision"] == "SAFE_TO_ATTEMPT":
                report["controller_submitted"] = True
            monitor.record_rejection(command_id)
            finished = monitor.finish(self.controller.trace(command_id))
        else:
            monitor.begin(command_id)
            _observe(monitor, self.controller.get_state())
            steps = 0
            limit = int(self.monitor_config["max_execution_steps"])
            while steps < limit and monitor.status == "TRACKING":
                if fault_after_steps is not None and steps == fault_after_steps:
                    self.controller.inject_fault("TEST_FAULT")
                    _observe(monitor, self.controller.get_state())
                    break
                _observe(monitor, self.controller.step(DT_S))
                steps += 1
            finished = monitor.finish(self.controller.trace(command_id))
            timed_out = monitor.status == "TRACKING"
        self.view = _view_for(case_name, report, finished, timed_out)
        return {
            "report": report,
            "finished": finished,
            "timed_out": timed_out,
            "submit_count_before": before,
            "submit_count_after": self.gate.submit_count,
            "validation_decision": report["decision"],
            "controller_submitted": bool(report.get("controller_submitted")),
        }

    def _restore_known_start(self, cycle_id: int) -> dict:
        state = self.controller.get_state()
        if _at_known_start(state) and state.controller_state == "READY" and state.fault is None:
            return {"required": False, "result": "NOT_REQUIRED"}
        if state.controller_state != "READY" or state.fault is not None:
            return {"required": True, "result": "FAILED", "reason": "controller was not READY before restore"}
        outcome = self._run_move(f"p011-restore-{cycle_id}", "A-zero", ZERO, None)
        restored = self.controller.get_state()
        accepted = outcome.get("validation_decision") == "SAFE_TO_ATTEMPT" and outcome.get("controller_submitted")
        finished = outcome.get("finished") or {}
        if not accepted or not finished.get("settled") or finished.get("fault_occurred") or not _at_known_start(restored):
            return {"required": True, "result": "FAILED", "decision": outcome.get("validation_decision")}
        self.view = initial_snapshot()
        return {"required": True, "result": "KNOWN_START_RESTORED"}

    def _recovery_attempt(self, index: int, case_name: str) -> dict:
        restore = self._restore_known_start(10_000 + index)
        if restore["result"] == "FAILED":
            return {"attempt": index, "result": "FAIL", "reason": "known start restore failed", "reset": restore}
        faulted = self._run_move(
            f"p011-recovery-{index}-fault",
            case_name,
            vector(case_name),
            int(self.config["recovery_fault_after_steps"]),
        )
        finished = faulted.get("finished") or {}
        observation = finished.get("fault_observation") or {}
        held = self.controller.get_state()
        held_joints = list(held.joint_position)
        fault_detected = bool(finished.get("fault_occurred")) and held.controller_state == "ERROR"
        completed = finished.get("completion_state") == "COMPLETED"
        settled = bool(finished.get("settled"))
        try:
            self.controller.reset()
            self.controller.servo_on()
        except Exception as exc:
            return {"attempt": index, "result": "FAIL", "reason": str(exc), "fault_detected": fault_detected}
        ready = self.controller.get_state()
        preserved = list(ready.joint_position) == held_joints and ready.controller_state == "READY" and ready.fault is None
        nxt = self._run_move(f"p011-recovery-{index}-next", case_name, vector(case_name), None)
        next_finished = nxt.get("finished") or {}
        next_completed = next_finished.get("completion_state") == "COMPLETED" and bool(next_finished.get("settled"))
        passed = fault_detected and not completed and not settled and preserved and next_completed
        self._restore_known_start(20_000 + index)
        return {
            "attempt": index,
            "result": "PASS" if passed else "FAIL",
            "fault_detected": fault_detected,
            "completed": completed,
            "settled": settled,
            "reset": ready.controller_state == "READY",
            "state_preserved": preserved,
            "next_execution_completed": next_completed,
            "held_joints": held_joints,
            "joints_after_reset": list(ready.joint_position),
            "fault": observation.get("fault"),
        }

    def _row_from_move(
        self,
        cycle_id: int,
        iteration: int,
        case_name: str,
        pre: dict,
        post: dict,
        outcome: dict,
        stale: bool,
    ) -> dict:
        report = outcome.get("report") or {}
        finished = outcome.get("finished") or {}
        metrics = finished.get("metrics") or {}
        samples = finished.get("samples") or []
        expected = EXPECTED[case_name]
        actual = str(outcome.get("validation_decision") or "REJECT")
        submitted = bool(outcome.get("controller_submitted"))
        tracked = bool(finished.get("tracking_started"))
        completed = finished.get("completion_state") == "COMPLETED" and not finished.get("fault_occurred")
        settled = bool(finished.get("settled")) and not finished.get("fault_occurred")
        separation = _separation(samples)
        reasons = list(report.get("blocking_reason_codes") or [])
        row = {
            "cycle_id": cycle_id,
            "iteration": iteration,
            "case_name": case_name,
            "expected_decision": expected,
            "actual_decision": actual,
            "validation_reason": reasons,
            "controller_submitted": submitted,
            "tracking_started": tracked,
            "completed": completed if expected == "SAFE_TO_ATTEMPT" else False,
            "settled": settled if expected == "SAFE_TO_ATTEMPT" else False,
            "controller_state_before": pre["controller_state"],
            "controller_state_after": post["controller_state"],
            "execution_state": _cycle_execution(finished, expected),
            "sample_count": 0 if not metrics else int(metrics.get("sample_count") or 0),
            "duration_sec": None if not metrics else metrics.get("duration_sec"),
            "settling_time_sec": None if not settled else finished.get("settling_time_s"),
            "final_joint_error_rad": None if not metrics else metrics.get("final_max_joint_error_rad"),
            "final_tcp_position_error_m": None if not metrics else metrics.get("final_tcp_position_error_m"),
            "final_tcp_orientation_error_rad": None if not metrics else metrics.get("final_tcp_orientation_error_rad"),
            "timeout": bool(outcome.get("timed_out")),
            "non_finite": bool(finished.get("non_finite_rejected")) or _record_non_finite(metrics),
            "fault": bool(finished.get("fault_occurred")),
            "reset_required": False,
            "reset_result": "PENDING",
            "separation_observed": separation,
            "state_leak": stale and expected == "REJECT" and (submitted or tracked),
            "stale_execute_enabled_present": stale,
            "pre_cycle_state": pre,
            "post_cycle_state": post,
            "hmi_decision": self.view.validation_decision,
            "hmi_execute_enabled": self.view.execute_enabled,
            "hmi_controller_state": self.view.controller_state,
            "hmi_execution_state": self.view.execution_state,
            "hmi_settled": self.view.settled,
            "hmi_fault": self.view.fault,
            "submit_count_before": outcome.get("submit_count_before"),
            "submit_count_after": outcome.get("submit_count_after"),
            "internal_error": outcome.get("internal_error"),
        }
        if outcome.get("internal_error"):
            row["category_hint"] = "INTERNAL_ERROR"
        if expected == "REJECT" and self.view.execute_enabled:
            row["state_leak"] = True
        if expected == "SAFE_TO_ATTEMPT" and case_name in NONZERO_SAFE and not separation and tracked:
            row["separation_observed"] = False
        category = classify_cycle(row)
        row["failure_category"] = category
        row["result"] = "PASS" if category is None else "FAIL"
        if row["non_finite"]:
            row["final_joint_error_rad"] = _finite_or_none(row["final_joint_error_rad"])
            row["final_tcp_position_error_m"] = _finite_or_none(row["final_tcp_position_error_m"])
            row["final_tcp_orientation_error_rad"] = _finite_or_none(row["final_tcp_orientation_error_rad"])
            row["duration_sec"] = _finite_or_none(row["duration_sec"])
            row["settling_time_sec"] = _finite_or_none(row["settling_time_sec"])
        return row

    def _failed_row(self, cycle_id: int, iteration: int, case_name: str, pre: dict, category: str, stale: bool) -> dict:
        row = {
            "cycle_id": cycle_id,
            "iteration": iteration,
            "case_name": case_name,
            "expected_decision": EXPECTED[case_name],
            "actual_decision": "NOT_RUN",
            "validation_reason": [],
            "controller_submitted": False,
            "tracking_started": False,
            "completed": False,
            "settled": False,
            "controller_state_before": pre["controller_state"],
            "controller_state_after": pre["controller_state"],
            "execution_state": "NOT_STARTED",
            "sample_count": 0,
            "duration_sec": None,
            "settling_time_sec": None,
            "final_joint_error_rad": None,
            "final_tcp_position_error_m": None,
            "final_tcp_orientation_error_rad": None,
            "timeout": False,
            "non_finite": False,
            "fault": False,
            "reset_required": False,
            "reset_result": "NOT_REQUIRED",
            "separation_observed": False,
            "state_leak": True,
            "stale_execute_enabled_present": stale,
            "pre_cycle_state": pre,
            "post_cycle_state": pre,
            "hmi_decision": self.view.validation_decision,
            "hmi_execute_enabled": self.view.execute_enabled,
            "hmi_controller_state": self.view.controller_state,
            "hmi_execution_state": self.view.execution_state,
            "hmi_settled": self.view.settled,
            "hmi_fault": self.view.fault,
            "failure_category": category,
            "result": "FAIL",
        }
        return row

    def _precondition_leak(self, pre: dict) -> str | None:
        if pre["controller_state"] != "READY" or pre["fault"] is not None or pre["active_command_id"] is not None:
            return "STATE_LEAK"
        if not pre["known_start"]:
            return "STATE_LEAK"
        if self.view.execute_enabled or self.view.validation_decision != "NOT RUN":
            return "STATE_LEAK"
        return None

    def _capture(self, label: str) -> dict:
        state = self.controller.get_state()
        return {
            "label": label,
            "controller_state": state.controller_state,
            "execution_state": state.execution_state,
            "joint_position": list(state.joint_position),
            "target_joint_position": list(state.target_joint_position),
            "fault": state.fault,
            "active_command_id": state.active_command_id,
            "known_start": _at_known_start(state),
        }

    def _note_failure(self, row: dict) -> None:
        if self.first_failure_cycle is not None:
            return
        self.first_failure_cycle = row["cycle_id"]
        self.first_failure_category = row["failure_category"]
        self.first_failure_details = {
            "case_name": row["case_name"],
            "expected_decision": row["expected_decision"],
            "actual_decision": row["actual_decision"],
            "controller_submitted": row["controller_submitted"],
            "tracking_started": row["tracking_started"],
        }

    def _checkpoint(self, cycles_completed: int, started: float) -> None:
        rows = self.rows
        point = {
            "cycles_completed": cycles_completed,
            "correct": sum(1 for row in rows if row["result"] == "PASS"),
            "failures": sum(1 for row in rows if row["result"] != "PASS"),
            "unsafe_submissions": sum(1 for row in rows if row.get("failure_category") == "UNSAFE_SUBMISSION"),
            "timeouts": sum(1 for row in rows if row.get("timeout")),
            "non_finite": sum(1 for row in rows if row.get("non_finite")),
            "rss_bytes": current_rss_bytes(),
            "elapsed_sec": time.perf_counter() - started,
        }
        self.checkpoints.append(point)
        print(
            f"{cycles_completed}/500 correct {point['correct']} failures {point['failures']} rss {point['rss_bytes']} elapsed {point['elapsed_sec']:.1f}",
            flush=True,
        )


def current_rss_bytes() -> int | None:
    completed = subprocess.run(
        ["ps", "-o", "rss=", "-p", str(os.getpid())],
        text=True,
        capture_output=True,
        check=False,
    )
    text = completed.stdout.strip()
    if not text:
        return None
    return int(text.split()[0]) * 1024


def _observe(monitor: DigitalTwinMonitor, state) -> None:
    monitor.observe(
        state.target_joint_position,
        state.joint_position,
        state.controller_state,
        state.execution_state,
        state.timestamp_s,
        state.fault,
    )


def _at_known_start(state) -> bool:
    return max(abs(value) for value in state.joint_position) <= COMPLETION_TOLERANCE_RAD


def _separation(samples: list[dict]) -> bool:
    for sample in samples:
        target = sample.get("target_q") or []
        actual = sample.get("actual_q") or []
        if any(left != right for left, right in zip(target, actual, strict=True)):
            return True
    return False


def _cycle_execution(finished: dict, expected: str) -> str:
    if finished.get("fault_occurred"):
        return "FAULTED"
    trace = finished.get("controller_trace") or []
    if trace == ["ACCEPTED", "EXECUTING", "COMPLETED"]:
        return "COMPLETED"
    if expected == "REJECT":
        return "NOT_STARTED"
    if not finished.get("tracking_started"):
        return "NOT_STARTED"
    return "NOT_COMPLETED"


def _view_for(case_name: str, report: dict, finished: dict, timed_out: bool) -> HmiSnapshot:
    if report.get("decision") != "SAFE_TO_ATTEMPT":
        return snapshot_from_validation(case_name, report)
    metrics = finished.get("metrics") or {}
    result = {
        "case": case_name,
        "validation_decision": report.get("decision"),
        "blocking_reason_codes": list(report.get("blocking_reason_codes") or []),
        "controller_submitted": bool(report.get("controller_submitted")),
        "tracking_started": bool(finished.get("tracking_started")),
        "completed": finished.get("completion_state") == "COMPLETED",
        "settled": bool(finished.get("settled")),
        "samples": finished.get("samples") or [],
        "metrics": metrics,
        "settling_time_s": finished.get("settling_time_s"),
        "fault_occurred": bool(finished.get("fault_occurred")),
        "fault_observation": finished.get("fault_observation"),
        "timed_out": timed_out,
    }
    return snapshot_from_run(result)


def case_name_from_command(command_id: str) -> str:
    if "-restore-" in command_id:
        return "A-zero"
    for name in EXPECTED:
        if command_id.endswith(name) or f"-{name}" in command_id:
            return name
    return "A-zero"


def _record_non_finite(metrics: dict) -> bool:
    for key in (
        "final_max_joint_error_rad",
        "final_tcp_position_error_m",
        "final_tcp_orientation_error_rad",
        "duration_sec",
        "settling_time_sec",
    ):
        value = metrics.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            continue
        if not math.isfinite(float(value)):
            return True
    return False


def _finite_or_none(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    if not math.isfinite(float(value)):
        return None
    return float(value)


def _json_line(row: dict) -> str:
    return json.dumps(_json_safe(row), sort_keys=True, allow_nan=False) + "\n"


def _json_safe(value: object) -> object:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    return value
