"""Validator를 통과한 명령만 controller와 monitor로 넘긴다."""

from __future__ import annotations

from controller.evidence import DT_S
from controller.virtual_pmac import JointCommand, VirtualPmacController
from core.kinematics.configurations import baseline_configurations
from core.model.ur10e import load_ur10e_model
from digital_twin.monitor import DigitalTwinMonitor, load_monitor_config
from validator.gate import ExecutionGate
from validator.validate import load_validator_config
from workcell.layout import resolve_workcell

DESIGNATED_SAFE = ("A-zero", "B-positive-1", "C-mixed-1", "E-fixed-3")
KNOWN_COLLISION = "E-fixed-1"


def vector(name: str) -> tuple[float, ...]:
    for row in baseline_configurations():
        if row["test_id"] == name:
            return tuple(row["joint_vector_rad"])
    raise ValueError(name)


def run_case(
    name: str,
    model=None,
    resolved: dict | None = None,
    validator_config: dict | None = None,
    monitor_config: dict | None = None,
    fault_after_steps: int | None = None,
) -> dict:
    robot = load_ur10e_model() if model is None else model
    workcell = resolve_workcell() if resolved is None else resolved
    validation = load_validator_config() if validator_config is None else validator_config
    monitor_settings = load_monitor_config() if monitor_config is None else monitor_config
    if float(monitor_settings["sample_period_sec"]) != DT_S:
        raise ValueError("monitor sample period must stay on the P-006 project clock")
    controller = VirtualPmacController(robot)
    controller.connect()
    controller.servo_on()
    gate = ExecutionGate(controller)
    command = JointCommand(name, "MOVE_JOINT", vector(name))
    before = gate.submit_count
    report = gate.consider(command, controller.get_state(), workcell, validation)
    monitor = DigitalTwinMonitor(robot, monitor_settings)
    if report["decision"] != "SAFE_TO_ATTEMPT":
        monitor.record_rejection(name)
        finished = monitor.finish(controller.trace(name))
        return _envelope(name, report, gate.submit_count, before, finished, False, None)
    if report["controller_result"]["result"] != "ACCEPTED":
        monitor.record_rejection(name)
        finished = monitor.finish(controller.trace(name))
        return _envelope(name, report, gate.submit_count, before, finished, False, None)
    monitor.begin(name)
    _observe(monitor, controller.get_state())
    steps = 0
    limit = int(monitor_settings["max_execution_steps"])
    while steps < limit and monitor.status == "TRACKING":
        if fault_after_steps is not None and steps == fault_after_steps:
            controller.inject_fault("TEST_FAULT")
            _observe(monitor, controller.get_state())
            break
        _observe(monitor, controller.step(DT_S))
        steps += 1
    finished = monitor.finish(controller.trace(name))
    timed_out = monitor.status == "TRACKING"
    return _envelope(name, report, gate.submit_count, before, finished, timed_out, fault_after_steps)


def _observe(monitor: DigitalTwinMonitor, state) -> None:
    monitor.observe(
        state.target_joint_position,
        state.joint_position,
        state.controller_state,
        state.execution_state,
        state.timestamp_s,
        state.fault,
    )


def _envelope(
    name: str,
    report: dict,
    submit_after: int,
    submit_before: int,
    finished: dict,
    timed_out: bool,
    fault_step: int | None,
) -> dict:
    metrics = finished["metrics"] or {}
    return {
        "case": name,
        "validation_decision": report["decision"],
        "blocking_reason_codes": list(report["blocking_reason_codes"]),
        "controller_submitted": report["controller_submitted"],
        "submit_count_before": submit_before,
        "submit_count_after": submit_after,
        "tracking_started": finished["tracking_started"],
        "execution_trace_created": bool(finished["controller_trace"]),
        "completed": finished["completion_state"] == "COMPLETED",
        "settled": finished["settled"],
        "status": finished["status"],
        "sample_count": 0 if metrics is None else metrics.get("sample_count", 0),
        "metrics": finished["metrics"],
        "samples": finished["samples"],
        "settling_time_s": finished["settling_time_s"],
        "completion_state": finished["completion_state"],
        "controller_trace": finished["controller_trace"],
        "fault_occurred": finished["fault_occurred"],
        "fault_observation": finished["fault_observation"],
        "timed_out": timed_out,
        "fault_step": fault_step,
    }
