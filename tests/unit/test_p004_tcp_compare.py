"""P-004 canonical TCP 비교. 시뮬레이터를 실행하지 않는다."""

import math

from core.validation.tcp_compare import (
    orientation_error_rad,
    unity_pose_to_canonical,
    unity_position_to_canonical,
    unity_quaternion_to_canonical,
)
from core.validation.compare_tcp import pose_errors


def test_unity2ros_matches_importer_fixture():
    # BuiltInExtensionsTests.ToFromRosTests, vector (1, 2, 3), quaternion (0.1, 0.2, 0.3, 0.4).
    position = unity_position_to_canonical({"x": 1.0, "y": 2.0, "z": 3.0})
    quaternion = unity_quaternion_to_canonical({"x": 0.1, "y": 0.2, "z": 0.3, "w": 0.4})
    assert position == {"x": 3.0, "y": -1.0, "z": 2.0}
    assert quaternion == {"x": -0.3, "y": 0.1, "z": -0.2, "w": 0.4}


def test_orientation_treats_quaternion_sign_as_the_same_rotation():
    raw = (0.1, 0.2, 0.3, 0.4)
    norm = math.sqrt(sum(value * value for value in raw))
    reference = {"x": raw[0] / norm, "y": raw[1] / norm, "z": raw[2] / norm, "w": raw[3] / norm}
    flipped = {key: -value for key, value in reference.items()}
    assert orientation_error_rad(reference, flipped) == 0.0


def test_same_pose_has_zero_error_after_unity_map():
    position = {"x": -0.29, "y": 0.06, "z": 1.18}
    quaternion = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    matrix = unity_pose_to_canonical(position, quaternion)
    pose = {
        "x": matrix[0][3],
        "y": matrix[1][3],
        "z": matrix[2][3],
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.0,
        "qw": 1.0,
    }
    errors = pose_errors(pose, pose)
    assert errors["position_error_m"] == 0.0
    assert errors["orientation_error_rad"] == 0.0
    assert math.isclose(pose["x"], 1.18)
    assert math.isclose(pose["y"], 0.29)
    assert math.isclose(pose["z"], 0.06)
