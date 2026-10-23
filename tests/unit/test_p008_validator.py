"""P-008 pre-execution validator. Isaac과 ROS evidence는 다시 쓰지 않는다."""

import json
import math
from pathlib import Path

import pytest

from controller.evidence import DT_S
from controller.virtual_pmac import JointCommand, RobotState, VirtualPmacController
from core.kinematics.configurations import baseline_configurations
from core.model.ur10e import load_ur10e_model
from validator.gate import ExecutionGate
from validator.geometry import obb_intersects_aabb, triangle_intersects_aabb, triangles_intersect
from validator.scene import load_collision_links
from validator.validate import interpolate, joint_limit_rows, load_validator_config, trajectory_limit_metrics, validate
from workcell.layout import resolve_workcell

ROOT = Path(__file__).resolve().parents[2]
ZERO = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def test_triangle_and_obb_overlap_rules():
    box = ((0.0, 0.0, 0.0), (1.0, 1.0, 1.0))
    inside = ((0.1, 0.1, 0.1), (0.2, 0.1, 0.1), (0.1, 0.2, 0.1))
    outside = ((2.0, 2.0, 2.0), (3.0, 2.0, 2.0), (2.0, 3.0, 2.0))
    touching = ((1.0, 0.1, 0.1), (1.0, 0.2, 0.1), (1.0, 0.1, 0.2))
    penetrating = ((0.9, 0.2, 0.2), (1.1, 0.2, 0.2), (0.9, 0.4, 0.2))
    assert triangle_intersects_aabb(inside, *box) is True
    assert triangle_intersects_aabb(outside, *box) is False
    assert triangle_intersects_aabb(touching, *box) is False
    assert triangle_intersects_aabb(penetrating, *box) is True
    coplanar = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0))
    assert triangles_intersect(coplanar, ((0.1, 0.1, 0.0), (0.8, 0.1, 0.0), (0.1, 0.8, 0.0))) is True
    assert triangles_intersect(coplanar, ((0.2, 0.2, -1.0), (0.2, 0.2, 1.0), (0.8, 0.8, 0.0))) is True
    assert triangles_intersect(inside, outside) is False
    axes = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    assert obb_intersects_aabb((0.5, 0.5, 0.5), axes, (0.1, 0.1, 0.1), *box) is True
    assert obb_intersects_aabb((3.0, 0.5, 0.5), axes, (0.1, 0.1, 0.1), *box) is False


def test_mesh_origins_come_from_the_pinned_yaml():
    links = {link.name: link for link in load_collision_links()}
    assert links["upper_arm"].origin_xyz[2] == 0.1762
    assert links["forearm"].origin_xyz[2] == 0.0393
    assert links["wrist_1"].origin_xyz == (0.0, 0.0, -0.135)
    assert links["wrist_2"].origin_xyz == (0.0, 0.0, -0.12)
    assert links["wrist_3"].origin_xyz == (0.0, -0.0005, -0.1168)
    assert links["base"].local_low[2] > -1e-6
    assert links["base"].local_high[2] == pytest.approx(0.0993, abs=1e-4)


def test_interpolation_includes_endpoints_and_catches_a_middle_sample():
    config = load_validator_config()
    assert config["trajectory_sample_count"] == 100
    samples = interpolate((0.0, 0.0), (1.0, -1.0), 100)
    assert samples[0] == (0.0, 0.0)
    assert samples[-1] == (1.0, -1.0)
    assert samples[50] == pytest.approx((50 / 99, -50 / 99))
    model = load_ur10e_model()
    limits = model.joint_by_name("elbow_joint").limits
    outside = limits.lower - 0.2
    start = ZERO
    target = (0.0, 0.0, outside, 0.0, 0.0, 0.0)
    path = interpolate(start, target, 100)
    metrics = trajectory_limit_metrics(model, path)
    assert metrics["sample_count"] == 100
    assert metrics["first_violation_index"] > 0
    previous = path[metrics["first_violation_index"] - 1][2]
    current = path[metrics["first_violation_index"]][2]
    assert previous >= limits.lower
    assert current < limits.lower


def test_joint_limit_boundaries_follow_source_inclusion():
    model = load_ur10e_model()
    for index, name in enumerate(model.actuated_joint_order):
        limits = model.joint_by_name(name).limits
        probes = {
            limits.lower + 1e-4: True,
            limits.lower: True,
            limits.lower - 1e-4: False,
            limits.upper - 1e-4: True,
            limits.upper: True,
            limits.upper + 1e-4: False,
        }
        for value, inside in probes.items():
            joints = tuple(value if joint_index == index else 0.0 for joint_index in range(6))
            row = joint_limit_rows(model, joints)[index]
            assert row["joint"] == name
            assert row["inside"] is inside
            assert row["margin_to_lower"] == pytest.approx(value - limits.lower)
            assert row["margin_to_upper"] == pytest.approx(limits.upper - value)


def test_non_finite_and_unsupported_commands_do_not_reach_the_controller():
    resolved = resolve_workcell()
    controller = _ready()
    gate = ExecutionGate(controller)
    nan = gate.consider(JointCommand("nan", "MOVE_JOINT", (0, 0, math.nan, 0, 0, 0)), _state(ZERO), resolved)
    inf = gate.consider(JointCommand("inf", "MOVE_JOINT", (0, 0, 0, 0, 0, math.inf)), _state(ZERO), resolved)
    tcp = gate.consider(JointCommand("tcp", "MOVE_TCP", ZERO), _state(ZERO), resolved)
    assert nan["decision"] == "REJECT"
    assert inf["decision"] == "REJECT"
    assert tcp["decision"] == "REJECT"
    assert "NON_FINITE" in nan["blocking_reason_codes"]
    assert "NON_FINITE" in inf["blocking_reason_codes"]
    assert "UNSUPPORTED_COMMAND" in tcp["blocking_reason_codes"]
    assert gate.submit_count == 0
    assert controller.trace("nan") == ()
    assert controller.trace("inf") == ()
    assert controller.trace("tcp") == ()


def test_outside_limit_rejects_and_does_not_treat_not_evaluated_as_pass():
    resolved = resolve_workcell()
    model = load_ur10e_model()
    limits = model.joint_by_name("elbow_joint").limits
    target = (0.0, 0.0, limits.lower - 0.2, 0.0, 0.0, 0.0)
    controller = _ready()
    gate = ExecutionGate(controller)
    report = gate.consider(JointCommand("low", "MOVE_JOINT", target), _state(ZERO), resolved)
    by_name = {check["name"]: check for check in report["checks"]}
    assert report["decision"] == "REJECT"
    assert report["blocking_reason_codes"][0] == "JOINT_LIMIT"
    assert "TRAJECTORY_INVALID" in report["blocking_reason_codes"]
    assert "UNREACHABLE" in report["blocking_reason_codes"]
    assert by_name["robot_environment_collision"]["result"] == "NOT_EVALUATED"
    assert "NOT_EVALUATED" in report["blocking_reason_codes"]
    assert gate.submit_count == 0


def test_zero_pose_is_safe_and_reaches_the_controller():
    resolved = resolve_workcell()
    controller = _ready()
    gate = ExecutionGate(controller)
    before = gate.submit_count
    report = gate.consider(JointCommand("A-zero", "MOVE_JOINT", ZERO), controller.get_state(), resolved)
    assert report["decision"] == "SAFE_TO_ATTEMPT"
    assert gate.submit_count == before + 1
    state = controller.step(DT_S)
    assert state.execution_state == "COMPLETED"
    assert controller.trace("A-zero") == ("ACCEPTED", "EXECUTING", "COMPLETED")
    by_name = {check["name"]: check for check in report["checks"]}
    assert by_name["tcp_alignment"]["result"] == "NOT_APPLICABLE"
    assert by_name["ik_feasibility"]["result"] == "NOT_APPLICABLE"
    assert by_name["self_collision"]["result"] == "PASS"
    assert by_name["self_collision"]["metrics"]["label"] == "EVALUATED"


def test_canonical_path_collision_is_not_forced_safe():
    resolved = resolve_workcell()
    rows = {row["test_id"]: row["joint_vector_rad"] for row in baseline_configurations()}
    controller = _ready()
    gate = ExecutionGate(controller)
    report = gate.consider(
        JointCommand("E-fixed-1", "MOVE_JOINT", rows["E-fixed-1"]),
        controller.get_state(),
        resolved,
    )
    collision = next(check for check in report["checks"] if check["name"] == "robot_environment_collision")
    assert report["decision"] == "REJECT"
    assert collision["result"] == "FAIL"
    assert collision["metrics"]["involved_objects"][0] == {"a": "wrist_3_link", "b": "bolt_panel"}
    assert collision["metrics"]["triangle_tests"] > 0
    assert gate.submit_count == 0
    assert controller.get_state().joint_position == ZERO


def test_explicit_safety_zone_rejects_a_pose_that_is_otherwise_clear():
    resolved = resolve_workcell()
    config = load_validator_config()
    inside = json_config(config)
    inside["safety_zone"]["mode"] = "explicit_aabb"
    inside["safety_zone"]["explicit_aabb_m"] = {"min_m": [1.0, 0.0, 0.5], "max_m": [1.4, 0.5, 1.2]}
    report = validate(JointCommand("zone", "MOVE_JOINT", ZERO), _state(ZERO), resolved, inside)
    by_name = {check["name"]: check for check in report["checks"]}
    assert by_name["robot_environment_collision"]["result"] == "PASS"
    assert by_name["safety_zone"]["result"] == "FAIL"
    assert by_name["safety_zone"]["reason_code"] == "SAFETY_ZONE"
    assert report["decision"] == "REJECT"
    assert report["blocking_reason_codes"] == ["SAFETY_ZONE"]


def test_fail_closed_on_internal_error_and_replay_matches():
    resolved = resolve_workcell()
    broken = {"bolt_panel_world": {}}
    report = validate(JointCommand("broken", "MOVE_JOINT", ZERO), _state(ZERO), broken)
    assert report["decision"] == "REJECT"
    assert report["blocking_reason_codes"] == ["VALIDATION_INTERNAL_ERROR"]
    config = load_validator_config()
    first = validate(JointCommand("same", "MOVE_JOINT", ZERO), _state(ZERO), resolved, config)
    second = validate(JointCommand("same", "MOVE_JOINT", ZERO), _state(ZERO), resolved, config)
    assert first["decision"] == second["decision"]
    assert first["blocking_reason_codes"] == second["blocking_reason_codes"]
    assert first["summary"] == second["summary"]


def test_validator_is_outside_the_controller_and_ros_adapter():
    for relative in ("controller/virtual_pmac.py", "ros2_adapter/node.py", "ros2_adapter/convert.py"):
        text = (ROOT / relative).read_text(encoding="utf-8")
        assert "validator" not in text
    gate = (ROOT / "validator" / "gate.py").read_text(encoding="utf-8")
    assert "SAFE_TO_ATTEMPT" in gate
    assert "submit" in gate


def json_config(config: dict) -> dict:
    return json.loads(json.dumps(config))


def _ready() -> VirtualPmacController:
    controller = VirtualPmacController(load_ur10e_model())
    controller.connect()
    controller.servo_on()
    return controller


def _state(joints: tuple[float, ...]) -> RobotState:
    zeros = tuple(0.0 for _ in joints)
    return RobotState(0.0, "ur10e", "READY", joints, zeros, zeros, zeros, {}, None, None, None, None, None)
