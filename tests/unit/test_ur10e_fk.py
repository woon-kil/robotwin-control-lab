"""독립 FK와 UR10e 모델의 구조 검사. 외부 pose와 비교하지 않는다."""

import math
from dataclasses import replace

import pytest

from core.errors import KinematicsError
from core.kinematics.configurations import baseline_configurations
from core.kinematics.fk import chain_frames, chain_transform, forward_kinematics, kinematic_path
from core.kinematics.limits import limit_violations
from core.kinematics.transform import (
    axis_rotation,
    identity,
    matrix_to_quaternion,
    multiply,
    origin_matrix,
    rpy_matrix,
)
from core.model.ur10e import (
    ACTUATED_JOINT_ORDER,
    SOURCE_COMMIT,
    SOURCE_MANIFEST,
    degrees_to_radians,
    load_ur10e_model,
    repo_root,
    source_root,
)


@pytest.fixture(scope="module")
def model():
    return load_ur10e_model()


def test_source_pin_matches_manifest():
    text = (repo_root() / SOURCE_MANIFEST).read_text(encoding="utf-8")
    assert SOURCE_COMMIT in text
    assert "UniversalRobots/Universal_Robots_ROS2_Description" in text


def test_actuated_count_and_order(model):
    assert model.actuated_joint_order == ACTUATED_JOINT_ORDER
    assert len(model.actuated_joint_order) == 6
    path = kinematic_path(model)
    revolute = tuple(joint.name for joint in path if joint.joint_type == "revolute")
    assert revolute == ACTUATED_JOINT_ORDER
    parents = {
        "shoulder_pan_joint": ("base_link_inertia", "shoulder_link"),
        "shoulder_lift_joint": ("shoulder_link", "upper_arm_link"),
        "elbow_joint": ("upper_arm_link", "forearm_link"),
        "wrist_1_joint": ("forearm_link", "wrist_1_link"),
        "wrist_2_joint": ("wrist_1_link", "wrist_2_link"),
        "wrist_3_joint": ("wrist_2_link", "wrist_3_link"),
    }
    for name, (parent, child) in parents.items():
        joint = model.joint_by_name(name)
        assert (joint.parent, joint.child) == (parent, child)
        assert joint.axis == (0.0, 0.0, 1.0)
        assert joint.joint_type == "revolute"


def test_base_and_tool_frames_stay_distinct(model):
    base = model.joint_by_name("base_link-base_fixed_joint")
    inertia = model.joint_by_name("base_link-base_link_inertia")
    assert base.child == "base"
    assert inertia.child == "base_link_inertia"
    assert base.child != inertia.child
    assert model.base_frame == "base_link"
    assert model.tcp_frame == "tool0"
    assert model.joint_by_name("flange-tool0").parent == "flange"
    assert model.joint_by_name("wrist_3-flange").child == "flange"


def test_degree_tags_become_radians(model):
    elbow = model.joint_by_name("elbow_joint").limits
    shoulder = model.joint_by_name("shoulder_pan_joint").limits
    assert elbow.source_min_position == -180.0
    assert elbow.source_max_position == 180.0
    assert elbow.lower == pytest.approx(degrees_to_radians(-180.0))
    assert elbow.upper == pytest.approx(math.pi)
    assert shoulder.source_max_position == 360.0
    assert shoulder.upper == pytest.approx(2.0 * math.pi)
    assert shoulder.source_position_unit == "degrees"
    wrist = model.joint_by_name("wrist_3_joint").limits
    assert wrist.effort == 54.0


def test_fixed_origins_match_xacro_text():
    macro = (source_root() / "urdf" / "ur_macro.xacro").read_text(encoding="utf-8")
    entry = (source_root() / "urdf" / "ur.urdf.xacro").read_text(encoding="utf-8")
    assert 'rpy="0 0 ${pi}"' in macro
    assert 'rpy="0 ${-pi/2.0} ${-pi/2.0}"' in macro
    assert 'rpy="${pi/2.0} 0 ${pi/2.0}"' in macro
    assert 'rpy="${pi} 0 0"' in macro
    assert '<axis xyz="0 0 1" />' in macro
    assert 'xyz="0 0 0" rpy="0 0 0"' in entry


def test_chain_frames_use_the_same_tool0_pose(model):
    joints = (0.2, -0.4, 0.3, -0.2, 0.6, -0.7)
    pose = forward_kinematics(model, joints)
    frames = chain_frames(model, joints)
    cumulative = frames[-1]["cumulative"]
    assert frames[-1]["child"] == model.tcp_frame
    assert cumulative["reference_frame"] == model.base_frame
    assert cumulative["x"] == pose.x
    assert cumulative["y"] == pose.y
    assert cumulative["z"] == pose.z
    assert cumulative["qx"] == pose.qx
    assert cumulative["qy"] == pose.qy
    assert cumulative["qz"] == pose.qz
    assert cumulative["qw"] == pose.qw
    shoulder = next(frame for frame in chain_frames(model, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)) if frame["child"] == "shoulder_link")
    assert shoulder["cumulative"]["z"] == pytest.approx(0.1807)


def test_zero_configuration_partial_chain(model):
    zeros = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    shoulder = _prefix_until(model, zeros, "shoulder_link")
    forearm = _prefix_until(model, zeros, "forearm_link")
    assert shoulder[0][3] == pytest.approx(0.0)
    assert shoulder[1][3] == pytest.approx(0.0)
    assert shoulder[2][3] == pytest.approx(0.1807)
    assert forearm[0][3] == pytest.approx(0.6127)
    assert forearm[1][3] == pytest.approx(0.0)
    assert forearm[2][3] == pytest.approx(0.1807)
    transform = chain_transform(model, zeros)
    assert transform[3] == pytest.approx([0.0, 0.0, 0.0, 1.0])


def test_configurations_are_finite_deterministic_and_inside_limits(model):
    rows = baseline_configurations()
    assert len(rows) >= 10
    ids = [row["test_id"] for row in rows]
    assert ids[0] == "A-zero"
    assert any(str(item).startswith("B-") for item in ids)
    assert any(str(item).startswith("C-") for item in ids)
    assert any(str(item).startswith("D-") for item in ids)
    assert any(str(item).startswith("E-") for item in ids)
    for row in rows:
        joints = tuple(row["joint_vector_rad"])
        assert limit_violations(model, joints) == ()
        first = forward_kinematics(model, joints)
        second = forward_kinematics(model, joints)
        assert first == second
        for value in (first.x, first.y, first.z, first.qx, first.qy, first.qz, first.qw):
            assert math.isfinite(value)
        transform = chain_transform(model, joints)
        assert transform[3] == pytest.approx([0.0, 0.0, 0.0, 1.0])
        _assert_rotation(transform)
        quat_norm = math.sqrt(first.qx**2 + first.qy**2 + first.qz**2 + first.qw**2)
        assert quat_norm == pytest.approx(1.0)


def test_zero_configuration_is_reproducible(model):
    zeros = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert forward_kinematics(model, zeros) == forward_kinematics(model, list(zeros))


def test_invalid_inputs_are_rejected(model):
    with pytest.raises(KinematicsError) as short:
        forward_kinematics(model, (0.0, 0.0, 0.0, 0.0, 0.0))
    assert short.value.code == "JOINT_COUNT"
    with pytest.raises(KinematicsError) as long:
        forward_kinematics(model, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    assert long.value.code == "JOINT_COUNT"
    with pytest.raises(KinematicsError) as nan:
        forward_kinematics(model, (0.0, 0.0, math.nan, 0.0, 0.0, 0.0))
    assert nan.value.code == "NON_FINITE"
    with pytest.raises(KinematicsError) as infinite:
        forward_kinematics(model, (0.0, math.inf, 0.0, 0.0, 0.0, 0.0))
    assert infinite.value.code == "NON_FINITE"
    with pytest.raises(KinematicsError) as mapping:
        forward_kinematics(model, {"elbow_joint": 0.0})
    assert mapping.value.code == "JOINT_ORDER"


def test_unsupported_joint_type_is_rejected(model):
    broken_joints = []
    for joint in model.joints:
        if joint.name == "elbow_joint":
            broken_joints.append(replace(joint, joint_type="prismatic"))
        else:
            broken_joints.append(joint)
    broken = replace(model, joints=tuple(broken_joints))
    with pytest.raises(KinematicsError) as error:
        forward_kinematics(broken, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    assert error.value.code == "UNSUPPORTED_JOINT"


def test_fk_does_not_clamp_outside_limits(model):
    elbow = model.joint_by_name("elbow_joint").limits
    outside = (0.0, 0.0, elbow.upper + 0.5, 0.0, 0.0, 0.0)
    original = outside
    clamped = (0.0, 0.0, elbow.upper, 0.0, 0.0, 0.0)
    pose_outside = forward_kinematics(model, outside)
    pose_clamped = forward_kinematics(model, clamped)
    assert outside == original
    assert pose_outside != pose_clamped
    assert limit_violations(model, outside) == ("elbow_joint",)
    assert limit_violations(model, clamped) == ()


def test_rpy_and_quaternion_match_known_rotations():
    rz = rpy_matrix(0.0, 0.0, math.pi)
    assert rz[0][0] == pytest.approx(-1.0)
    assert rz[1][1] == pytest.approx(-1.0)
    assert rz[2][2] == pytest.approx(1.0)
    quarter = axis_rotation((0.0, 0.0, 1.0), math.pi / 2.0)
    qx, qy, qz, qw = matrix_to_quaternion(quarter)
    assert qx == pytest.approx(0.0)
    assert qy == pytest.approx(0.0)
    assert qz == pytest.approx(math.sqrt(0.5))
    assert qw == pytest.approx(math.sqrt(0.5))
    identity = matrix_to_quaternion(rpy_matrix(0.0, 0.0, 0.0))
    assert identity == pytest.approx((0.0, 0.0, 0.0, 1.0))


def _prefix_until(model, joints, child_name):
    position_by_name = dict(zip(model.actuated_joint_order, joints, strict=True))
    transform = identity()
    for joint in kinematic_path(model):
        origin = origin_matrix(joint.origin_xyz, joint.origin_rpy)
        if joint.joint_type == "revolute":
            motion = multiply(origin, axis_rotation(joint.axis, position_by_name[joint.name]))
        else:
            motion = origin
        transform = multiply(transform, motion)
        if joint.child == child_name:
            return transform
    raise AssertionError(child_name)


def _assert_rotation(transform):
    rotation = [row[:3] for row in transform[:3]]
    for i in range(3):
        for j in range(3):
            dot = sum(rotation[k][i] * rotation[k][j] for k in range(3))
            assert dot == pytest.approx(1.0 if i == j else 0.0)
    det = (
        rotation[0][0] * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1] * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2] * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
    )
    assert det == pytest.approx(1.0)
