"""Digital twin tracking records. Controller state 이름은 바꾸지 않는다."""

from __future__ import annotations

from dataclasses import dataclass


MONITOR_STATES = ("IDLE", "TRACKING", "SETTLED", "REJECTED", "ERROR")


@dataclass(frozen=True)
class TrackingSample:
    sequence: int
    timestamp_s: float
    dt_s: float
    controller_state: str
    execution_state: str | None
    target_q: tuple[float, ...]
    actual_q: tuple[float, ...]
    joint_error_rad: tuple[float, ...]
    abs_joint_error_rad: tuple[float, ...]
    target_tcp: dict[str, object]
    actual_tcp: dict[str, object]
    tcp_position_error_m: float
    tcp_orientation_error_rad: float

    def as_dict(self) -> dict[str, object]:
        payload: dict[str, object] = {
            "sequence": self.sequence,
            "timestamp_s": self.timestamp_s,
            "time_s": self.timestamp_s,
            "dt_s": self.dt_s,
            "controller_state": self.controller_state,
            "execution_state": self.execution_state,
            "target_q": list(self.target_q),
            "actual_q": list(self.actual_q),
            "joint_error_rad": list(self.joint_error_rad),
            "abs_joint_error_rad": list(self.abs_joint_error_rad),
            "target_tcp": self.target_tcp,
            "actual_tcp": self.actual_tcp,
            "tcp_position_error_m": self.tcp_position_error_m,
            "tcp_orientation_error_rad": self.tcp_orientation_error_rad,
        }
        for index, value in enumerate(self.joint_error_rad, start=1):
            payload["joint_" + str(index) + "_error_rad"] = value
        return payload
