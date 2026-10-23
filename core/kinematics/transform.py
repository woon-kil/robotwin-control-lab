"""4x4 동차변환. 외부 FK 라이브러리를 쓰지 않는다.

URDF origin rpy는 고정축 회전이다. 부모 좌표의 점은
p_parent = xyz + Rz(yaw) Ry(pitch) Rx(roll) p_child 이다.
출처: http://wiki.ros.org/urdf/XML/joint (조회 2026-10-23).
회전 관절은 그 origin 뒤에 축 회전을 곱한다.
T_parent_child(q) = T_origin * T_joint(q).
"""

import math

Matrix = list[list[float]]


def identity() -> Matrix:
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def multiply(left: Matrix, right: Matrix) -> Matrix:
    result = [[0.0, 0.0, 0.0, 0.0] for _ in range(4)]
    for row in range(4):
        for col in range(4):
            result[row][col] = (
                left[row][0] * right[0][col]
                + left[row][1] * right[1][col]
                + left[row][2] * right[2][col]
                + left[row][3] * right[3][col]
            )
    return result


def rpy_matrix(roll: float, pitch: float, yaw: float) -> Matrix:
    """Rz(yaw) * Ry(pitch) * Rx(roll). 각은 radian."""
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    rotation = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    transform = identity()
    for row in range(3):
        for col in range(3):
            transform[row][col] = rotation[row][col]
    return transform


def translation_matrix(x: float, y: float, z: float) -> Matrix:
    transform = identity()
    transform[0][3] = x
    transform[1][3] = y
    transform[2][3] = z
    return transform


def axis_rotation(axis: tuple[float, float, float], angle: float) -> Matrix:
    """관절 프레임의 축에 대한 Rodrigues 회전. angle은 radian."""
    ax, ay, az = axis
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if norm == 0.0 or not math.isfinite(norm):
        raise ValueError("joint axis norm is not finite and nonzero")
    x = ax / norm
    y = ay / norm
    z = az / norm
    c = math.cos(angle)
    s = math.sin(angle)
    v = 1.0 - c
    rotation = [
        [c + x * x * v, x * y * v - z * s, x * z * v + y * s],
        [y * x * v + z * s, c + y * y * v, y * z * v - x * s],
        [z * x * v - y * s, z * y * v + x * s, c + z * z * v],
    ]
    transform = identity()
    for row in range(3):
        for col in range(3):
            transform[row][col] = rotation[row][col]
    return transform


def origin_matrix(
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float],
) -> Matrix:
    roll, pitch, yaw = rpy
    return multiply(translation_matrix(*xyz), rpy_matrix(roll, pitch, yaw))


def matrix_to_quaternion(rotation_rows: Matrix) -> tuple[float, float, float, float]:
    """Hamilton quaternion (x, y, z, w). 부호는 이 변환이 내는 값을 유지한다."""
    r00 = rotation_rows[0][0]
    r11 = rotation_rows[1][1]
    r22 = rotation_rows[2][2]
    trace = r00 + r11 + r22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation_rows[2][1] - rotation_rows[1][2]) / scale
        y = (rotation_rows[0][2] - rotation_rows[2][0]) / scale
        z = (rotation_rows[1][0] - rotation_rows[0][1]) / scale
    elif r00 > r11 and r00 > r22:
        scale = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        w = (rotation_rows[2][1] - rotation_rows[1][2]) / scale
        x = 0.25 * scale
        y = (rotation_rows[0][1] + rotation_rows[1][0]) / scale
        z = (rotation_rows[0][2] + rotation_rows[2][0]) / scale
    elif r11 > r22:
        scale = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        w = (rotation_rows[0][2] - rotation_rows[2][0]) / scale
        x = (rotation_rows[0][1] + rotation_rows[1][0]) / scale
        y = 0.25 * scale
        z = (rotation_rows[1][2] + rotation_rows[2][1]) / scale
    else:
        scale = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        w = (rotation_rows[1][0] - rotation_rows[0][1]) / scale
        x = (rotation_rows[0][2] + rotation_rows[2][0]) / scale
        y = (rotation_rows[1][2] + rotation_rows[2][1]) / scale
        z = 0.25 * scale
    return (x, y, z, w)
