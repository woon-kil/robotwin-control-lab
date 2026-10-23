"""P-006 virtual controller. Isaac과 wall-clock sleep을 쓰지 않는다."""

import math
from pathlib import Path

import pytest

from controller.evidence import DT_S, build_payload
from controller.virtual_pmac import (
    COMPLETION_TOLERANCE_RAD,
    CLAIM_BOUNDARY,
    ControllerError,
    JointCommand,
    VirtualPmacController,
)
from core.model.ur10e import load_ur10e_model
from isaac.ur10e_commands import CONFIGURATIONS

ROOT = Path(__file__).resolve().parents[2]
SELECTED = ("A-zero", "B-positive-1", "C-mixed-1", "E-fixed-1", "E-fixed-3")


def test_lifecycle_and_joint_command_do_not_teleport():
    controller = _ready_controller()
    command = JointCommand("move", "MOVE_JOINT", CONFIGURATIONS[1][1])
    accepted = controller.submit(command)
    before = controller.get_state()

    assert accepted.result == "ACCEPTED"
    assert before.controller_state == "RUNNING"
    assert before.execution_state == "ACCEPTED"
    assert before.joint_position != before.target_joint_position
    assert before.joint_position == tuple(0.0 for _ in range(6))

    stepped = controller.step(DT_S)
    assert stepped.execution_state == "EXECUTING"
    assert stepped.joint_position != command.joint_targets
    assert max(abs(value) for value in stepped.joint_position) <= 3.141592653589793 * DT_S + 1e-12


def test_five_configurations_complete_in_order_and_replay_matches():
    payload = build_payload()
    rows = payload["command_results"]["configurations"]

    assert [row["command_id"] for row in rows] == list(SELECTED)
    assert all(row["execution_trace"] == ["ACCEPTED", "EXECUTING", "COMPLETED"] for row in rows)
    assert all(row["max_joint_error_rad"] <= COMPLETION_TOLERANCE_RAD for row in rows)
    assert payload["command_results"]["sequence"]["result"] == "PASS"
    assert payload["command_results"]["sequence"]["busy"]["second_code"] == "BUSY"
    assert payload["determinism_results"]["max_difference_rad"] == 0.0
    assert payload["determinism_results"]["run1_completion_steps"] == payload["determinism_results"]["run2_completion_steps"]
    assert payload["fault_results"]["joint_state_preserved"] is True
    assert payload["fault_results"]["reset_succeeds"] is True
    assert payload["result"] == "PASS"


def test_invalid_commands_and_illegal_transitions_are_rejected():
    disconnected = VirtualPmacController(load_ur10e_model())
    refused = disconnected.submit(JointCommand("x", "MOVE_JOINT", (0, 0, 0, 0, 0, 0)))
    assert refused.result == "REJECTED"
    assert refused.error_code == "ILLEGAL_STATE"

    controller = _ready_controller()
    assert controller.submit(JointCommand("short", "MOVE_JOINT", (0, 0, 0, 0, 0))).error_code == "JOINT_COUNT"
    assert controller.submit(JointCommand("long", "MOVE_JOINT", (0, 0, 0, 0, 0, 0, 0))).error_code == "JOINT_COUNT"
    assert controller.submit(JointCommand("nan", "MOVE_JOINT", (0, 0, 0, 0, 0, math.nan))).error_code == "NON_FINITE"
    assert controller.submit(JointCommand("inf", "MOVE_JOINT", (0, 0, 0, 0, 0, math.inf))).error_code == "NON_FINITE"
    assert controller.submit(JointCommand("tcp", "MOVE_TCP", (0, 0, 0, 0, 0, 0))).error_code == "UNSUPPORTED_COMMAND"
    assert controller.submit(JointCommand("limit", "MOVE_JOINT", (7, 0, 0, 0, 0, 0))).error_code == "JOINT_LIMIT"
    assert controller.get_state().controller_state == "READY"

    with pytest.raises(ControllerError):
        controller.reset()


def test_fault_blocks_commands_and_reset_keeps_joints():
    controller = _ready_controller()
    controller.submit(JointCommand("move", "MOVE_JOINT", CONFIGURATIONS[2][1]))
    controller.step(DT_S)
    held = controller.get_state().joint_position
    controller.inject_fault("TEST_FAULT")
    rejected = controller.submit(JointCommand("next", "MOVE_JOINT", CONFIGURATIONS[1][1]))

    assert controller.get_state().controller_state == "ERROR"
    assert controller.trace("move")[-1] == "FAULTED"
    assert rejected.result == "REJECTED"
    controller.reset_fault()
    assert controller.get_state().controller_state == "CONNECTED"
    assert controller.get_state().joint_position == held


def test_disconnect_aborts_without_zeroing_joints():
    controller = _ready_controller()
    controller.submit(JointCommand("move", "MOVE_JOINT", CONFIGURATIONS[3][1]))
    controller.step(DT_S)
    held = controller.get_state().joint_position
    controller.disconnect()
    state = controller.get_state()

    assert state.controller_state == "DISCONNECTED"
    assert state.joint_position == held
    assert state.joint_position != tuple(0.0 for _ in range(6))
    assert controller.trace("move")[-1] == "REJECTED"


def test_controller_core_does_not_call_simulator_apis():
    text = "\n".join(
        path.read_text(encoding="utf-8")
        for path in (ROOT / "controller").glob("*.py")
        if path.name != "evidence.py"
    )
    for banned in ("isaacsim", "omni.", "rclpy", "UnityEngine"):
        assert banned not in text
    assert "NOT VERIFIED AGAINST REAL POWER PMAC" in CLAIM_BOUNDARY


def test_committed_evidence_matches_the_deterministic_report():
    payload = build_payload()
    evidence = ROOT / "evidence" / "P-006"
    summary = (evidence / "summary.md").read_text(encoding="utf-8")
    contract = (evidence / "controller_contract.json").read_text(encoding="utf-8")

    assert payload["result"] == "PASS"
    assert "NOT VERIFIED AGAINST REAL POWER PMAC" in summary
    assert "NOT VERIFIED AGAINST REAL POWER PMAC" in contract
    assert "real_pmac_compatibility_claimed\": false" in contract or '"real_pmac_compatibility_claimed": false' in contract
    assert "MOVE_JOINT" in contract
    assert "Power PMAC binary" not in summary
    assert (evidence / "latency_results.json").is_file()


def _ready_controller() -> VirtualPmacController:
    controller = VirtualPmacController(load_ur10e_model())
    controller.connect()
    controller.enable()
    return controller
