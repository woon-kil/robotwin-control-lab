"""ROS payload와 Canonical joint command 사이의 변환.

degree로 바꾸는 계수는 두지 않는다. 관절 배열 순서는 이름으로만 맞춘다.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass

from controller.virtual_pmac import COMMAND_TYPE_MOVE_JOINT, JointCommand, RobotState
from ros2_adapter.contract import CANONICAL_JOINT_ORDER

_REQUIRED = ("command_id", "joint_names", "positions")


@dataclass(frozen=True)
class DecodedCommand:
    command: JointCommand | None
    error_code: str | None
    input_names: tuple[str, ...]
    input_positions: tuple[float, ...]
    canonical_positions: tuple[float, ...] | None


def decode_command(payload_text: str) -> DecodedCommand:
    """JSON 문자열을 MOVE_JOINT 후보로 바꾼다. 실패하면 controller에 넣지 않는다."""
    try:
        payload = json.loads(payload_text)
    except json.JSONDecodeError:
        return _failed("SCHEMA")
    if not isinstance(payload, dict) or any(key not in payload for key in _REQUIRED):
        return _failed("SCHEMA")
    command_id = payload["command_id"]
    names = payload["joint_names"]
    positions = payload["positions"]
    if not isinstance(command_id, str) or command_id == "":
        return _failed("SCHEMA")
    if not isinstance(names, list) or not isinstance(positions, list):
        return _failed("SCHEMA")
    if any(not isinstance(name, str) for name in names):
        return _failed("SCHEMA")
    if "unit" in payload and payload["unit"] != "radian":
        return _failed("UNIT", names)
    command_type = payload.get("command_type", COMMAND_TYPE_MOVE_JOINT)
    if not isinstance(command_type, str):
        return _failed("SCHEMA", names)
    parsed, finite_code = _finite_positions(positions)
    if finite_code is not None:
        return _failed(finite_code, names)
    assert parsed is not None
    if len(names) != len(parsed):
        return _failed("SCHEMA", names, parsed)
    name_error = _name_error(names)
    if name_error is not None:
        return _failed(name_error, names, parsed)
    ordered = tuple(parsed[names.index(joint)] for joint in CANONICAL_JOINT_ORDER)
    command = JointCommand(
        command_id=command_id,
        command_type=command_type,
        joint_targets=ordered,
        timestamp_s=0.0,
        robot_id="ur10e",
    )
    return DecodedCommand(command, None, tuple(names), parsed, ordered)


def feedback_document(state: RobotState, feedback_seq: int) -> dict[str, object]:
    """Controller state를 ROS로 나가기 전의 plain object로 둔다."""
    return {
        "simulation_time_s": state.timestamp_s,
        "controller_state": state.controller_state,
        "active_command_id": state.active_command_id,
        "execution_state": state.execution_state,
        "command_id": state.command_id,
        "actual_joint_positions": list(state.joint_position),
        "target_joint_positions": list(state.target_joint_position),
        "joint_errors": list(state.joint_error),
        "joint_names": list(CANONICAL_JOINT_ORDER),
        "fault": state.fault,
        "error_code": state.error_code,
        "feedback_seq": feedback_seq,
        "angle_unit": "radian",
        "time_unit": "second",
    }


def actual_positions(state: RobotState) -> tuple[float, ...]:
    """ /joint_states.position 에 들어가는 값. target을 반환하지 않는다."""
    return tuple(state.joint_position)


def target_positions(state: RobotState) -> tuple[float, ...]:
    """target topic에만 들어간다."""
    return tuple(state.target_joint_position)


def _name_error(names: list[str]) -> str | None:
    if len(names) != len(set(names)):
        return "DUPLICATE_JOINT"
    if len(names) > len(CANONICAL_JOINT_ORDER):
        return "EXTRA_JOINT"
    unknown = [name for name in names if name not in CANONICAL_JOINT_ORDER]
    if unknown:
        return "UNKNOWN_JOINT"
    if set(names) != set(CANONICAL_JOINT_ORDER):
        return "MISSING_JOINT"
    return None


def _finite_positions(positions: list[object]) -> tuple[tuple[float, ...] | None, str | None]:
    parsed: list[float] = []
    for value in positions:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return None, "NON_FINITE"
        number = float(value)
        if not math.isfinite(number):
            return None, "NON_FINITE"
        parsed.append(number)
    return tuple(parsed), None


def _failed(
    code: str,
    names: list[str] | tuple[str, ...] = (),
    positions: tuple[float, ...] = (),
) -> DecodedCommand:
    return DecodedCommand(None, code, tuple(names), positions, None)
