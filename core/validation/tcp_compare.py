"""Canonical TCP 비교.

T_A_B는 프레임 B의 점을 프레임 A로 보낸다. p_A = T_A_B * p_B.
T_base_tcp = inverse(T_world_base) * T_world_tcp.

Unity 성분은 URDF Importer 0.5.2 hash 90f353e4352aae4df52fa2c05e49b804631d2a63의
BuiltInExtensions.Unity2Ros다. 위치 (z, -x, y), quaternion xyzw (-z, x, -y, w).
chosenAxis zAxis는 visual/collision mesh만 돌리고 link transform에는 곱하지 않는다.
출처: 같은 패키지의 UrdfRobotExtensions.CorrectAxis.
"""

from __future__ import annotations

import math
from typing import Mapping

from core.kinematics.transform import matrix_to_quaternion, multiply

Matrix = list[list[float]]

FK_POSITION_LIMIT_MM = 0.5
FK_ORIENTATION_LIMIT_DEG = 0.1
CROSS_POSITION_LIMIT_MM = 1.0

UNITY_CONVENTION_ID = (
    "Unity.Robotics.UrdfImporter.BuiltInExtensions.Unity2Ros"
    "@90f353e4352aae4df52fa2c05e49b804631d2a63"
)
ISAAC_CONVENTION_ID = "UsdGeom.XformCache.GetLocalToWorldTransform right-handed Z-up wxyz to Hamilton xyzw"


def position_error_m(reference: Mapping[str, float], test: Mapping[str, float]) -> float:
    return math.dist(
        (reference["x"], reference["y"], reference["z"]),
        (test["x"], test["y"], test["z"]),
    )


def orientation_error_rad(reference_xyzw: Mapping[str, float], test_xyzw: Mapping[str, float]) -> float:
    """R_error = R_ref^-1 * R_test의 회전각. q와 -q는 같은 행렬이다."""
    relative = multiply(transpose(quaternion_to_matrix(reference_xyzw)), quaternion_to_matrix(test_xyzw))
    trace = relative[0][0] + relative[1][1] + relative[2][2]
    cosine = max(-1.0, min(1.0, (trace - 1.0) * 0.5))
    return math.acos(cosine)


def quaternion_to_matrix(xyzw: Mapping[str, float]) -> Matrix:
    x = float(xyzw["x"])
    y = float(xyzw["y"])
    z = float(xyzw["z"])
    w = float(xyzw["w"])
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0 or not math.isfinite(norm):
        raise ValueError("quaternion norm is not finite and nonzero")
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w), 0.0],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w), 0.0],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y), 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def transpose(matrix: Matrix) -> Matrix:
    return [[matrix[col][row] for col in range(4)] for row in range(4)]


def invert_rigid(matrix: Matrix) -> Matrix:
    rotation = transpose(matrix)
    for row in range(3):
        rotation[row][3] = 0.0
        rotation[3][row] = 0.0
    rotation[3][3] = 1.0
    tx = matrix[0][3]
    ty = matrix[1][3]
    tz = matrix[2][3]
    rotation[0][3] = -(rotation[0][0] * tx + rotation[0][1] * ty + rotation[0][2] * tz)
    rotation[1][3] = -(rotation[1][0] * tx + rotation[1][1] * ty + rotation[1][2] * tz)
    rotation[2][3] = -(rotation[2][0] * tx + rotation[2][1] * ty + rotation[2][2] * tz)
    return rotation


def pose_matrix(position: Mapping[str, float], quaternion_xyzw: Mapping[str, float]) -> Matrix:
    matrix = quaternion_to_matrix(quaternion_xyzw)
    matrix[0][3] = float(position["x"])
    matrix[1][3] = float(position["y"])
    matrix[2][3] = float(position["z"])
    return matrix


def matrix_to_pose(matrix: Matrix, frame: str, reference_frame: str) -> dict[str, float | str]:
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


def relative_pose(
    world_base: Matrix,
    world_tcp: Matrix,
    frame: str = "tool0",
    reference_frame: str = "base_link",
) -> dict[str, float | str]:
    return matrix_to_pose(multiply(invert_rigid(world_base), world_tcp), frame, reference_frame)


def unity_position_to_canonical(position: Mapping[str, float]) -> dict[str, float]:
    return {"x": float(position["z"]), "y": -float(position["x"]), "z": float(position["y"])}


def unity_quaternion_to_canonical(quaternion: Mapping[str, float]) -> dict[str, float]:
    return {
        "x": -float(quaternion["z"]),
        "y": float(quaternion["x"]),
        "z": -float(quaternion["y"]),
        "w": float(quaternion["w"]),
    }


def unity_pose_to_canonical(position: Mapping[str, float], quaternion_xyzw: Mapping[str, float]) -> Matrix:
    return pose_matrix(unity_position_to_canonical(position), unity_quaternion_to_canonical(quaternion_xyzw))


def isaac_quaternion_to_canonical(quaternion_wxyz: Mapping[str, float]) -> dict[str, float]:
    return {
        "x": float(quaternion_wxyz["x"]),
        "y": float(quaternion_wxyz["y"]),
        "z": float(quaternion_wxyz["z"]),
        "w": float(quaternion_wxyz["w"]),
    }


def isaac_pose_to_canonical(position: Mapping[str, float], quaternion_wxyz: Mapping[str, float]) -> Matrix:
    return pose_matrix(
        {"x": float(position["x"]), "y": float(position["y"]), "z": float(position["z"])},
        isaac_quaternion_to_canonical(quaternion_wxyz),
    )


def classify_discrepancy(position_mm: float, orientation_deg: float) -> str:
    if position_mm > 50.0 and orientation_deg < 20.0:
        return "FRAME_MISMATCH"
    if abs(orientation_deg - 90.0) < 5.0 or abs(orientation_deg - 180.0) < 5.0:
        return "QUATERNION_ORDER_MISMATCH"
    if position_mm > 50.0:
        return "UNIT_MISMATCH"
    if orientation_deg > 20.0:
        return "AXIS_MISMATCH"
    if position_mm < 5.0 and orientation_deg < 1.0:
        return "UNKNOWN"
    return "KINEMATIC_MODEL_MISMATCH"
