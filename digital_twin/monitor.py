"""승인된 명령의 target과 actual을 project clock으로 기록한다."""

from __future__ import annotations

import json
import math
from pathlib import Path

from core.model.types import RobotModel
from core.model.ur10e import load_ur10e_model
from digital_twin.metrics import aggregate, joint_error_rad, tcp_pair
from digital_twin.model import MONITOR_STATES, TrackingSample

CONFIG_PATH = Path(__file__).resolve().parent / "config" / "default.json"


def load_monitor_config(path: Path | None = None) -> dict:
    source = CONFIG_PATH if path is None else path
    payload = json.loads(source.read_text(encoding="utf-8"))
    period = payload["sample_period_sec"]
    tolerance = payload["joint_settling_tolerance_rad"]
    count = payload["settling_consecutive_samples"]
    limit = payload["max_execution_steps"]
    if isinstance(period, bool) or not isinstance(period, (int, float)) or not math.isfinite(period) or period <= 0.0:
        raise ValueError("sample_period_sec must be a positive finite number")
    if isinstance(tolerance, bool) or not isinstance(tolerance, (int, float)) or not math.isfinite(tolerance) or tolerance <= 0.0:
        raise ValueError("joint_settling_tolerance_rad must be a positive finite number")
    if isinstance(count, bool) or not isinstance(count, int) or count < 2:
        raise ValueError("settling_consecutive_samples must be an integer >= 2")
    if isinstance(limit, bool) or not isinstance(limit, int) or limit < 1:
        raise ValueError("max_execution_steps must be an integer >= 1")
    return payload


class DigitalTwinMonitor:
    def __init__(self, model: RobotModel | None = None, config: dict | None = None) -> None:
        self._model = load_ur10e_model() if model is None else model
        self._config = load_monitor_config() if config is None else config
        self._status = "IDLE"
        self._command_id: str | None = None
        self._samples: list[TrackingSample] = []
        self._tracking_started = False
        self._settled = False
        self._settling_time_s: float | None = None
        self._execution_start_s: float | None = None
        self._accepted_time_s: float | None = None
        self._completion_time_s: float | None = None
        self._consecutive = 0
        self._non_finite = False
        self._fault = False
        self._fault_observation: dict[str, object] | None = None
        self._joint_names = tuple(self._model.actuated_joint_order)

    @property
    def status(self) -> str:
        return self._status

    @property
    def settled(self) -> bool:
        return self._settled

    @property
    def tracking_started(self) -> bool:
        return self._tracking_started

    @property
    def samples(self) -> tuple[TrackingSample, ...]:
        return tuple(self._samples)

    def begin(self, command_id: str) -> None:
        if self._status != "IDLE":
            self._fail("begin while not IDLE")
            return
        self._command_id = command_id
        self._status = "TRACKING"
        self._tracking_started = True

    def record_rejection(self, command_id: str) -> None:
        if self._status != "IDLE" or self._samples:
            self._fail("rejection after tracking started")
            return
        self._command_id = command_id
        self._status = "REJECTED"
        self._tracking_started = False
        self._settled = False

    def observe(
        self,
        target_q: tuple[float, ...] | list[float],
        actual_q: tuple[float, ...] | list[float],
        controller_state: str,
        execution_state: str | None,
        timestamp_s: float,
        fault: str | None = None,
    ) -> None:
        if self._status != "TRACKING":
            self._fail("observe while not TRACKING")
            return
        target = _vector(target_q)
        actual = _vector(actual_q)
        timestamp = _finite_number(timestamp_s)
        if target is None or actual is None or timestamp is None:
            self._non_finite = True
            self._fail("non-finite or invalid sample")
            return
        if fault is not None or controller_state == "ERROR" or execution_state == "FAULTED":
            self._fault = True
            self._settled = False
            self._fault_observation = {
                "timestamp_s": timestamp,
                "controller_state": controller_state,
                "execution_state": execution_state,
                "fault": fault,
                "target_q": list(target),
                "actual_q": list(actual),
            }
            self._status = "ERROR"
            return
        if self._samples and timestamp <= self._samples[-1].timestamp_s:
            self._fail("timestamp did not advance")
            return
        previous = self._samples[-1].timestamp_s if self._samples else timestamp
        error = joint_error_rad(target, actual)
        tcp = tcp_pair(self._model, target, actual)
        sample = TrackingSample(
            sequence=len(self._samples),
            timestamp_s=timestamp,
            dt_s=0.0 if not self._samples else timestamp - previous,
            controller_state=controller_state,
            execution_state=execution_state,
            target_q=target,
            actual_q=actual,
            joint_error_rad=error,
            abs_joint_error_rad=tuple(abs(value) for value in error),
            target_tcp=tcp["target_tcp"],
            actual_tcp=tcp["actual_tcp"],
            tcp_position_error_m=float(tcp["tcp_position_error_m"]),
            tcp_orientation_error_rad=float(tcp["tcp_orientation_error_rad"]),
        )
        self._samples.append(sample)
        self._mark_times(sample)
        self._update_settling(sample)

    def finish(self, controller_trace: tuple[str, ...] = ()) -> dict[str, object]:
        if self._status not in MONITOR_STATES:
            self._fail("unknown monitor state")
        completed = list(controller_trace) == ["ACCEPTED", "EXECUTING", "COMPLETED"]
        if self._status == "ERROR":
            self._settled = False
            self._settling_time_s = None
        metrics = aggregate(tuple(self._samples), self._joint_names) if self._samples else None
        if metrics is not None:
            metrics["settling_time_sec"] = self._settling_time_s
            metrics["command_accepted_time_s"] = self._accepted_time_s
            metrics["execution_start_time_s"] = self._execution_start_s
            metrics["completion_time_s"] = self._completion_time_s
            metrics["settled_time_s"] = self._settled_time_s()
        return {
            "command_id": self._command_id,
            "status": self._status,
            "samples": [sample.as_dict() for sample in self._samples],
            "metrics": metrics,
            "settled": self._settled,
            "settling_time_s": self._settling_time_s,
            "completion_state": "COMPLETED" if completed else "NOT_COMPLETED",
            "tracking_started": self._tracking_started,
            "non_finite_rejected": self._non_finite,
            "fault_occurred": self._fault,
            "fault_observation": self._fault_observation,
            "controller_trace": list(controller_trace),
        }

    def _mark_times(self, sample: TrackingSample) -> None:
        if self._accepted_time_s is None and sample.execution_state == "ACCEPTED":
            self._accepted_time_s = sample.timestamp_s
        if self._execution_start_s is None and sample.execution_state in ("EXECUTING", "COMPLETED"):
            self._execution_start_s = sample.timestamp_s
        if self._completion_time_s is None and sample.execution_state == "COMPLETED":
            self._completion_time_s = sample.timestamp_s

    def _update_settling(self, sample: TrackingSample) -> None:
        if self._status != "TRACKING":
            return
        tolerance = float(self._config["joint_settling_tolerance_rad"])
        required = int(self._config["settling_consecutive_samples"])
        if max(sample.abs_joint_error_rad) <= tolerance:
            self._consecutive += 1
        else:
            self._consecutive = 0
        if self._consecutive >= required and self._execution_start_s is not None:
            self._settled = True
            self._settling_time_s = sample.timestamp_s - self._execution_start_s
            self._status = "SETTLED"

    def _settled_time_s(self) -> float | None:
        if self._settling_time_s is None or self._execution_start_s is None:
            return None
        return self._execution_start_s + self._settling_time_s

    def _fail(self, _reason: str) -> None:
        self._status = "ERROR"
        self._settled = False
        self._settling_time_s = None


def _vector(values: tuple[float, ...] | list[float]) -> tuple[float, ...] | None:
    if isinstance(values, (str, bytes)):
        return None
    try:
        row = tuple(values)
    except TypeError:
        return None
    if len(row) != 6:
        return None
    parsed = []
    for value in row:
        number = _finite_number(value)
        if number is None:
            return None
        parsed.append(number)
    return tuple(parsed)


def _finite_number(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    if not math.isfinite(number):
        return None
    return number
