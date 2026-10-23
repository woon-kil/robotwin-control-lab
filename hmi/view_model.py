"""P-008과 P-009 결과를 화면 값으로 옮긴다. Tracking 식은 만들지 않는다."""

from __future__ import annotations

import math

from core.model.ur10e import ACTUATED_JOINT_ORDER
from digital_twin.session import KNOWN_COLLISION, vector
from hmi.model import HmiSnapshot

CASES = ("A-zero", "B-positive-1", "C-mixed-1", KNOWN_COLLISION, "E-fixed-3")
DEMO_SAFE = "B-positive-1"
JOINT_NAMES = tuple(ACTUATED_JOINT_ORDER)


def initial_snapshot() -> HmiSnapshot:
    return HmiSnapshot(joint_names=list(JOINT_NAMES))


def snapshot_from_validation(case_name: str, report: dict) -> HmiSnapshot:
    decision = str(report["decision"])
    pair, sample = _collision_view(report)
    snapshot = initial_snapshot()
    snapshot.command_id = case_name
    snapshot.case_name = case_name
    snapshot.commanded_q = list(vector(case_name))
    snapshot.validation_decision = decision
    snapshot.validation_reasons = list(report.get("blocking_reason_codes") or [])
    snapshot.validation_summary = str(report.get("summary") or "")
    snapshot.checks = _checks(report)
    snapshot.collision_pair = pair
    snapshot.collision_sample = sample
    snapshot.execute_enabled = decision == "SAFE_TO_ATTEMPT"
    snapshot.controller_submitted = False
    snapshot.tracking_started = False
    snapshot.completed = False
    snapshot.settled = None
    snapshot.fault = False
    snapshot.execution_state = "IDLE"
    snapshot.controller_state = None
    snapshot.ui_state = "READY" if decision == "SAFE_TO_ATTEMPT" else "REJECTED"
    snapshot.mode = "LIVE PROJECT RUN"
    snapshot.backend_blocked = decision != "SAFE_TO_ATTEMPT"
    return snapshot


def snapshot_from_run(result: dict) -> HmiSnapshot:
    faulted = bool(result.get("fault_occurred"))
    completed = bool(result.get("completed")) and not faulted
    settled = bool(result.get("settled")) and not faulted
    samples = list(result.get("samples") or [])
    metrics = result.get("metrics") or {}
    observation = result.get("fault_observation") or {}
    latest = samples[-1] if samples else None
    decision = str(result.get("validation_decision") or "NOT RUN")
    snapshot = initial_snapshot()
    snapshot.command_id = result.get("case")
    snapshot.case_name = result.get("case")
    snapshot.validation_decision = decision
    snapshot.validation_reasons = list(result.get("blocking_reason_codes") or [])
    snapshot.execute_enabled = decision == "SAFE_TO_ATTEMPT" and not faulted
    snapshot.controller_submitted = bool(result.get("controller_submitted"))
    snapshot.tracking_started = bool(result.get("tracking_started"))
    snapshot.completed = completed
    snapshot.settled = settled
    snapshot.settling_time_s = result.get("settling_time_s") if settled else None
    snapshot.settled_at_s = metrics.get("settled_time_s") if settled else None
    snapshot.completion_time_s = metrics.get("completion_time_s") if completed else None
    snapshot.fault = faulted
    snapshot.fault_code = observation.get("fault") if faulted else None
    snapshot.backend_blocked = decision != "SAFE_TO_ATTEMPT"
    snapshot.mode = "LIVE PROJECT RUN"
    snapshot.series = _series(samples)
    snapshot.peak_joint_error_rad = metrics.get("global_max_joint_error_rad")
    _copy_peak_sample(snapshot, samples)
    if latest is not None:
        snapshot.target_q = list(latest["target_q"])
        snapshot.actual_q = list(latest["actual_q"])
        snapshot.joint_error = list(latest["joint_error_rad"])
        snapshot.target_tcp = latest["target_tcp"]
        snapshot.actual_tcp = latest["actual_tcp"]
        snapshot.tcp_position_error_m = latest["tcp_position_error_m"]
        snapshot.tcp_orientation_error_rad = latest["tcp_orientation_error_rad"]
    if faulted:
        snapshot.controller_state = str(observation.get("controller_state") or "ERROR")
        snapshot.execution_state = "FAULTED"
        snapshot.ui_state = "FAULT"
        snapshot.execute_enabled = False
        snapshot.completed = False
        snapshot.settled = False
        snapshot.settling_time_s = None
        snapshot.settled_at_s = None
        snapshot.completion_time_s = None
    elif completed:
        snapshot.controller_state = str((latest or {}).get("controller_state") or "READY")
        snapshot.execution_state = "COMPLETED"
        snapshot.ui_state = "READY"
    elif decision == "REJECT":
        snapshot.controller_state = None
        snapshot.execution_state = "IDLE"
        snapshot.ui_state = "REJECTED"
        snapshot.execute_enabled = False
    else:
        snapshot.controller_state = None if latest is None else latest.get("controller_state")
        snapshot.execution_state = "IDLE" if latest is None else str(latest.get("execution_state") or "IDLE")
        snapshot.ui_state = "READY" if decision == "SAFE_TO_ATTEMPT" else "NOT RUN"
    return snapshot


def format_quantity(value: object, unit: str, scale: float = 1.0, digits: int = 2) -> str:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        return "INVALID"
    return f"{float(value) * scale:.{digits}f}{unit}"


def format_deg_from_rad(value: object) -> str:
    return format_quantity(value, "°", scale=180.0 / math.pi)


def format_mm(value: object) -> str:
    return format_quantity(value, " mm", scale=1000.0)


def format_rad(value: object) -> str:
    return format_quantity(value, " rad", digits=4)


def format_seconds(value: object) -> str:
    return format_quantity(value, " s", digits=2)


def _checks(report: dict) -> list[dict]:
    rows = []
    for check in report.get("checks") or []:
        rows.append(
            {
                "name": check.get("name"),
                "result": check.get("result"),
                "reason_code": check.get("reason_code"),
            }
        )
    return rows


def _collision_view(report: dict) -> tuple[dict | None, str | None]:
    for check in report.get("checks") or []:
        if check.get("result") != "FAIL" or check.get("reason_code") != "COLLISION":
            continue
        metrics = check.get("metrics") or {}
        objects = metrics.get("involved_objects") or []
        pair = objects[0] if objects else None
        index = metrics.get("sample_index")
        count = metrics.get("sample_count")
        if index is None or count is None:
            return pair, None
        return pair, f"{index} / {count}"
    return None, None


def _copy_peak_sample(snapshot: HmiSnapshot, samples: list[dict]) -> None:
    values = [row["max_abs_joint_error_rad"] for row in snapshot.series]
    if not values or any(value is None for value in values):
        return
    index = max(range(len(values)), key=values.__getitem__)
    chosen = samples[index]
    snapshot.peak_target_q = list(chosen["target_q"])
    snapshot.peak_actual_q = list(chosen["actual_q"])
    snapshot.peak_joint_error = list(chosen["joint_error_rad"])


def _series(samples: list[dict]) -> list[dict]:
    rows = []
    for sample in samples:
        values = sample.get("abs_joint_error_rad") or []
        if not values or any(not isinstance(value, (int, float)) or not math.isfinite(float(value)) for value in values):
            rows.append({"time_s": sample.get("timestamp_s"), "max_abs_joint_error_rad": None})
            continue
        rows.append(
            {
                "time_s": sample["timestamp_s"],
                "max_abs_joint_error_rad": max(float(value) for value in values),
            }
        )
    return rows
