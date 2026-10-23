"""Target과 actual을 각각 FK한 뒤 오차를 계산한다."""

from __future__ import annotations

import math

from core.kinematics.fk import Pose, forward_kinematics
from core.model.types import RobotModel
from core.validation.tcp_compare import orientation_error_rad, position_error_m
from digital_twin.model import TrackingSample


def joint_error_rad(target_q: tuple[float, ...], actual_q: tuple[float, ...]) -> tuple[float, ...]:
    """error_i = target_i - actual_i. 단위는 radian."""
    return tuple(target - actual for target, actual in zip(target_q, actual_q, strict=True))


def tcp_pair(model: RobotModel, target_q: tuple[float, ...], actual_q: tuple[float, ...]) -> dict[str, object]:
    target_pose = forward_kinematics(model, target_q)
    actual_pose = forward_kinematics(model, actual_q)
    position_m = position_error_m(_xyz(target_pose), _xyz(actual_pose))
    orientation_rad = orientation_error_rad(_xyzw(target_pose), _xyzw(actual_pose))
    return {
        "target_tcp": _pose(target_pose),
        "actual_tcp": _pose(actual_pose),
        "tcp_position_error_m": position_m,
        "tcp_orientation_error_rad": orientation_rad,
    }


def aggregate(samples: tuple[TrackingSample, ...], joint_names: tuple[str, ...]) -> dict[str, object]:
    if not samples:
        raise ValueError("metrics require at least one sample")
    per_joint = []
    for index, name in enumerate(joint_names):
        series = [sample.abs_joint_error_rad[index] for sample in samples]
        per_joint.append(
            {
                "joint": name,
                "max_abs_error_rad": max(series),
                "mean_abs_error_rad": sum(series) / len(series),
                "final_abs_error_rad": series[-1],
            }
        )
    position = [sample.tcp_position_error_m for sample in samples]
    orientation = [sample.tcp_orientation_error_rad for sample in samples]
    duration = samples[-1].timestamp_s - samples[0].timestamp_s
    return {
        "sample_count": len(samples),
        "duration_sec": duration,
        "per_joint": per_joint,
        "global_max_joint_error_rad": max(row["max_abs_error_rad"] for row in per_joint),
        "final_max_joint_error_rad": max(row["final_abs_error_rad"] for row in per_joint),
        "max_tcp_position_error_m": max(position),
        "final_tcp_position_error_m": position[-1],
        "max_tcp_orientation_error_rad": max(orientation),
        "final_tcp_orientation_error_rad": orientation[-1],
        "max_tcp_position_error_mm": max(position) * 1000.0,
        "max_tcp_orientation_error_deg": max(orientation) * 180.0 / math.pi,
        "accuracy_claim": "PROJECT VIRTUAL CONTROLLER tracking error. Not UR10e positioning accuracy.",
    }


def _pose(pose: Pose) -> dict[str, object]:
    return {
        "frame": pose.frame,
        "reference_frame": pose.reference_frame,
        "position_m": [pose.x, pose.y, pose.z],
        "quaternion_xyzw": [pose.qx, pose.qy, pose.qz, pose.qw],
    }


def _xyz(pose: Pose) -> dict[str, float]:
    return {"x": pose.x, "y": pose.y, "z": pose.z}


def _xyzw(pose: Pose) -> dict[str, float]:
    return {"x": pose.qx, "y": pose.qy, "z": pose.qz, "w": pose.qw}
