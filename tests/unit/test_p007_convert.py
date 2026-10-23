"""P-007 transport conversion. rclpy와 simulator를 import하지 않는다."""

import json
import math
from pathlib import Path

from controller.virtual_pmac import RobotState
from ros2_adapter.contract import CANONICAL_JOINT_ORDER, QOS_DEPTH, QOS_DURABILITY, QOS_HISTORY, QOS_RELIABILITY
from ros2_adapter.convert import actual_positions, decode_command, feedback_document, target_positions

ROOT = Path(__file__).resolve().parents[2]
ORDER = CANONICAL_JOINT_ORDER


def test_joint_names_are_reordered_and_degrees_are_not_applied():
    payload = {
        "command_id": "reorder",
        "joint_names": ["wrist_3_joint", "shoulder_pan_joint", "elbow_joint", "wrist_1_joint", "shoulder_lift_joint", "wrist_2_joint"],
        "positions": [0.06, 0.01, -0.03, 0.04, 0.02, -0.05],
    }
    decoded = decode_command(json.dumps(payload))

    assert decoded.error_code is None
    assert decoded.command is not None
    assert decoded.command.command_type == "MOVE_JOINT"
    assert decoded.command.joint_targets == (0.01, 0.02, -0.03, 0.04, -0.05, 0.06)
    raw = decode_command(json.dumps({**payload, "positions": [180, 0, 0, 0, 0, 0], "joint_names": list(ORDER)}))
    assert raw.command is not None
    assert raw.command.joint_targets[0] == 180.0


def test_invalid_joint_names_and_non_finite_values_are_rejected():
    assert _code({"command_id": "u", "joint_names": ["unknown_joint", *ORDER[1:]], "positions": [0, 0, 0, 0, 0, 0]}) == "UNKNOWN_JOINT"
    assert _code({"command_id": "d", "joint_names": [ORDER[0], ORDER[0], *ORDER[2:]], "positions": [0, 0, 0, 0, 0, 0]}) == "DUPLICATE_JOINT"
    assert _code({"command_id": "m", "joint_names": list(ORDER[:5]), "positions": [0, 0, 0, 0, 0]}) == "MISSING_JOINT"
    assert _code({"command_id": "e", "joint_names": [*ORDER, "extra_joint"], "positions": [0, 0, 0, 0, 0, 0, 0]}) == "EXTRA_JOINT"
    assert _code({"command_id": "n", "joint_names": list(ORDER), "positions": [0, 0, 0, 0, 0, math.nan]}) == "NON_FINITE"
    assert _code({"command_id": "i", "joint_names": list(ORDER), "positions": [0, 0, 0, 0, 0, math.inf]}) == "NON_FINITE"
    assert _code({"command_id": "deg", "joint_names": list(ORDER), "positions": [0, 0, 0, 0, 0, 0], "unit": "degree"}) == "UNIT"
    assert decode_command("{").error_code == "SCHEMA"


def test_feedback_keeps_controller_state_and_splits_actual_from_target():
    state = RobotState(
        timestamp_s=0.02,
        robot_id="ur10e",
        controller_state="RUNNING",
        joint_position=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        joint_velocity=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        target_joint_position=(0.2, 0.3, 0.4, 0.5, 0.6, 0.7),
        joint_error=(0.2, 0.3, 0.4, 0.5, 0.6, 0.7),
        tcp_pose={},
        command_id="move",
        active_command_id="move",
        execution_state="EXECUTING",
        error_code=None,
        fault=None,
    )
    document = feedback_document(state, 4)

    assert document["controller_state"] == "RUNNING"
    assert document["actual_joint_positions"] != document["target_joint_positions"]
    assert actual_positions(state) == state.joint_position
    assert target_positions(state) == state.target_joint_position
    assert document["angle_unit"] == "radian"
    assert document["fault"] is None


def test_adapter_source_does_not_call_simulators_or_copy_the_slew():
    node = (ROOT / "ros2_adapter" / "node.py").read_text(encoding="utf-8")
    convert = (ROOT / "ros2_adapter" / "convert.py").read_text(encoding="utf-8")
    evidence = (ROOT / "ros2_adapter" / "evidence.py").read_text(encoding="utf-8")
    for banned in ("isaacsim", "omni.", "UnityEngine", "rclpy", "slew_step"):
        assert banned not in convert
    for banned in ("isaacsim", "omni.", "UnityEngine", "slew_step"):
        assert banned not in node
    assert "VirtualPmacController(" not in evidence
    assert "actual_positions" in node
    assert "target_positions" in node
    assert QOS_RELIABILITY == "RELIABLE"
    assert QOS_HISTORY == "KEEP_LAST"
    assert QOS_DEPTH == 10
    assert QOS_DURABILITY == "VOLATILE"


def _code(payload: dict) -> str | None:
    return decode_command(json.dumps(payload, allow_nan=True)).error_code
