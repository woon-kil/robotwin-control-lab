"""Operator HMI records. Controller vocabulary는 바꾸지 않는다."""

from __future__ import annotations

from dataclasses import dataclass, field


UI_STATES = ("NOT RUN", "VALIDATING", "EXECUTING", "READY", "REJECTED", "FAULT")
MODES = ("NOT RUN", "LIVE PROJECT RUN")


@dataclass
class HmiSnapshot:
    command_id: str | None = None
    case_name: str | None = None
    commanded_q: list[float] | None = None
    validation_decision: str = "NOT RUN"
    validation_reasons: list[str] = field(default_factory=list)
    validation_summary: str = ""
    controller_state: str | None = None
    execution_state: str = "IDLE"
    target_q: list[float] | None = None
    actual_q: list[float] | None = None
    joint_error: list[float] | None = None
    target_tcp: dict | None = None
    actual_tcp: dict | None = None
    tcp_position_error_m: float | None = None
    tcp_orientation_error_rad: float | None = None
    settled: bool | None = None
    settling_time_s: float | None = None
    settled_at_s: float | None = None
    completion_time_s: float | None = None
    fault: bool = False
    fault_code: str | None = None
    execute_enabled: bool = False
    controller_submitted: bool = False
    tracking_started: bool = False
    completed: bool = False
    backend_blocked: bool = False
    ui_state: str = "NOT RUN"
    mode: str = "NOT RUN"
    collision_pair: dict | None = None
    collision_sample: str | None = None
    checks: list[dict] = field(default_factory=list)
    series: list[dict] = field(default_factory=list)
    peak_joint_error_rad: float | None = None
    peak_target_q: list[float] | None = None
    peak_actual_q: list[float] | None = None
    peak_joint_error: list[float] | None = None
    joint_names: list[str] = field(default_factory=list)

    def as_dict(self) -> dict:
        return {
            "command_id": self.command_id,
            "case_name": self.case_name,
            "commanded_q": self.commanded_q,
            "validation_decision": self.validation_decision,
            "validation_reasons": list(self.validation_reasons),
            "validation_summary": self.validation_summary,
            "controller_state": self.controller_state,
            "execution_state": self.execution_state,
            "target_q": self.target_q,
            "actual_q": self.actual_q,
            "joint_error": self.joint_error,
            "target_tcp": self.target_tcp,
            "actual_tcp": self.actual_tcp,
            "tcp_position_error_m": self.tcp_position_error_m,
            "tcp_orientation_error_rad": self.tcp_orientation_error_rad,
            "settled": self.settled,
            "settling_time_s": self.settling_time_s,
            "settled_at_s": self.settled_at_s,
            "completion_time_s": self.completion_time_s,
            "fault": self.fault,
            "fault_code": self.fault_code,
            "execute_enabled": self.execute_enabled,
            "controller_submitted": self.controller_submitted,
            "tracking_started": self.tracking_started,
            "completed": self.completed,
            "backend_blocked": self.backend_blocked,
            "ui_state": self.ui_state,
            "mode": self.mode,
            "collision_pair": self.collision_pair,
            "collision_sample": self.collision_sample,
            "checks": self.checks,
            "series": self.series,
            "peak_joint_error_rad": self.peak_joint_error_rad,
            "peak_target_q": self.peak_target_q,
            "peak_actual_q": self.peak_actual_q,
            "peak_joint_error": self.peak_joint_error,
            "joint_names": list(self.joint_names),
        }
