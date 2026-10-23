"""UR10e 독립 FK. 시뮬레이터 pose와 ROS TF를 읽지 않는다."""

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

from core.errors import KinematicsError
from core.kinematics.transform import (
    axis_rotation,
    identity,
    matrix_to_quaternion,
    multiply,
    origin_matrix,
)
from core.model.types import Joint, RobotModel


@dataclass(frozen=True)
class Pose:
    """base_frame에서 본 tcp_frame. 위치는 meter, 자세는 (x, y, z, w)."""

    frame: str
    reference_frame: str
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float

    def as_dict(self) -> dict[str, float | str]:
        return {
            "frame": self.frame,
            "reference_frame": self.reference_frame,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "qx": self.qx,
            "qy": self.qy,
            "qz": self.qz,
            "qw": self.qw,
        }


def forward_kinematics(
    robot_model: RobotModel,
    joint_positions: Sequence[float],
) -> Pose:
    """canonical actuated order의 radian 관절값으로 T_base_tcp를 계산한다.

    한계 밖 값은 clamp하지 않는다. limit 검사는 limit_violations가 맡는다.
    """
    positions = _read_actuated_positions(robot_model, joint_positions)
    path = kinematic_path(robot_model)
    position_by_name = dict(zip(robot_model.actuated_joint_order, positions, strict=True))
    transform = identity()
    for joint in path:
        transform = multiply(transform, _joint_motion(joint, position_by_name))
    qx, qy, qz, qw = matrix_to_quaternion(transform)
    return Pose(
        frame=robot_model.tcp_frame,
        reference_frame=robot_model.base_frame,
        x=transform[0][3],
        y=transform[1][3],
        z=transform[2][3],
        qx=qx,
        qy=qy,
        qz=qz,
        qw=qw,
    )


def kinematic_path(robot_model: RobotModel) -> tuple[Joint, ...]:
    """base_frame에서 tcp_frame까지 유일한 joint 경로."""
    children: dict[str, list[Joint]] = {}
    for joint in robot_model.joints:
        children.setdefault(joint.parent, []).append(joint)
    paths: list[tuple[Joint, ...]] = []

    def walk(link: str, trail: tuple[Joint, ...], seen: set[str]) -> None:
        if link == robot_model.tcp_frame:
            paths.append(trail)
            return
        if link in seen:
            return
        next_seen = set(seen)
        next_seen.add(link)
        for joint in children.get(link, []):
            walk(joint.child, trail + (joint,), next_seen)

    walk(robot_model.base_frame, (), set())
    if len(paths) != 1:
        raise KinematicsError(
            "INVALID_MODEL",
            f"expected one path from {robot_model.base_frame} to {robot_model.tcp_frame}, found {len(paths)}",
        )
    path = paths[0]
    for joint in path:
        if joint.joint_type not in ("fixed", "revolute"):
            raise KinematicsError(
                "UNSUPPORTED_JOINT",
                f"{joint.name} has unsupported type {joint.joint_type}",
            )
    actuated = tuple(joint.name for joint in path if joint.joint_type == "revolute")
    if actuated != robot_model.actuated_joint_order:
        raise KinematicsError(
            "INVALID_MODEL",
            "revolute joints on the tcp path do not match actuated_joint_order",
        )
    return path


def chain_transform(robot_model: RobotModel, joint_positions: Sequence[float]):
    """경로의 누적 4x4. 테스트가 마지막 행과 직교성을 본다."""
    frames = chain_frames(robot_model, joint_positions)
    if not frames:
        return identity()
    return frames[-1]["cumulative_matrix"]


def chain_frames(robot_model: RobotModel, joint_positions: Sequence[float]) -> tuple[dict[str, object], ...]:
    """base에서 tcp까지 기존 joint motion을 누적하고 자식 프레임마다 기록한다.

    forward_kinematics와 같은 `_joint_motion`이다. 기하 파라미터를 바꾸지 않는다.
    """
    positions = _read_actuated_positions(robot_model, joint_positions)
    position_by_name = dict(zip(robot_model.actuated_joint_order, positions, strict=True))
    transform = identity()
    frames: list[dict[str, object]] = []
    for joint in kinematic_path(robot_model):
        local = _joint_motion(joint, position_by_name)
        transform = multiply(transform, local)
        frames.append(
            {
                "joint": joint.name,
                "joint_type": joint.joint_type,
                "parent": joint.parent,
                "child": joint.child,
                "origin_xyz_m": [float(value) for value in joint.origin_xyz],
                "origin_rpy_rad": [float(value) for value in joint.origin_rpy],
                "axis_xyz": [float(value) for value in joint.axis],
                "local_matrix": local,
                "cumulative_matrix": transform,
                "local": _matrix_pose(local, joint.child, joint.parent),
                "cumulative": _matrix_pose(transform, joint.child, robot_model.base_frame),
            }
        )
    return tuple(frames)


def _matrix_pose(matrix, frame: str, reference_frame: str) -> dict[str, float | str]:
    qx, qy, qz, qw = matrix_to_quaternion(matrix)
    return {
        "frame": frame,
        "reference_frame": reference_frame,
        "x": matrix[0][3],
        "y": matrix[1][3],
        "z": matrix[2][3],
        "qx": qx,
        "qy": qy,
        "qz": qz,
        "qw": qw,
    }


def _joint_motion(joint: Joint, position_by_name: dict[str, float]):
    origin = origin_matrix(joint.origin_xyz, joint.origin_rpy)
    if joint.joint_type == "fixed":
        return origin
    if joint.joint_type != "revolute":
        raise KinematicsError(
            "UNSUPPORTED_JOINT",
            f"{joint.name} has unsupported type {joint.joint_type}",
        )
    if joint.name not in position_by_name:
        raise KinematicsError("UNKNOWN_JOINT", joint.name)
    return multiply(origin, axis_rotation(joint.axis, position_by_name[joint.name]))


def _read_actuated_positions(
    robot_model: RobotModel,
    joint_positions: Sequence[float],
) -> tuple[float, ...]:
    if isinstance(joint_positions, Mapping):
        raise KinematicsError(
            "JOINT_ORDER",
            "joint_positions must be a sequence in actuated_joint_order, not a mapping",
        )
    if isinstance(joint_positions, (str, bytes)):
        raise KinematicsError("JOINT_COUNT", "joint_positions must be six numbers")
    try:
        values = tuple(joint_positions)
    except TypeError as exc:
        raise KinematicsError("JOINT_COUNT", "joint_positions is not a sequence") from exc
    expected = len(robot_model.actuated_joint_order)
    if len(values) != expected:
        raise KinematicsError(
            "JOINT_COUNT",
            f"expected {expected} joint positions, got {len(values)}",
        )
    numbers: list[float] = []
    for index, value in enumerate(values):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise KinematicsError(
                "NON_FINITE",
                f"joint index {index} is not a real number",
            )
        number = float(value)
        if not math.isfinite(number):
            raise KinematicsError("NON_FINITE", f"joint index {index} is not finite")
        numbers.append(number)
    return tuple(numbers)
