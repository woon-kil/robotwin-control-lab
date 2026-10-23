"""P-008 FIX2 acceptance. Validator collision 판정은 완화하지 않는다."""

import json
from pathlib import Path

import pytest

from validator.acceptance_fix2 import CLASSIFICATION, DESIGNATED_SAFE, assert_output_directory, build_acceptance
from validator.validate import OPTIONAL_CHECKS, REQUIRED_CHECKS, load_validator_config

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def acceptance() -> dict:
    return build_acceptance()


def test_classification_keeps_canonical_names_and_separates_workcell_safety():
    assert DESIGNATED_SAFE == ("A-zero", "B-positive-1", "C-mixed-1", "E-fixed-3")
    assert CLASSIFICATION["E-fixed-1"] == "KNOWN_COLLISION"
    assert set(CLASSIFICATION) == {"A-zero", "B-positive-1", "C-mixed-1", "E-fixed-1", "E-fixed-3"}


def test_validator_config_is_not_weakened():
    config = load_validator_config()
    assert config["trajectory_sample_count"] == 100
    assert config["collision"]["touching_faces_are_collision"] is False
    assert config["safety_zone"]["prohibited_depth_m"] == 0.4
    assert config["safety_zone"]["class"] == "PROJECT TEST SAFETY ZONE"
    assert REQUIRED_CHECKS == (
        "finite_input",
        "command_support",
        "target_validity",
        "joint_limit",
        "reachability",
        "trajectory",
        "fk_trajectory",
        "robot_environment_collision",
        "tool_environment_collision",
        "tool_robot_collision",
        "self_collision",
        "safety_zone",
    )
    assert OPTIONAL_CHECKS == ("tcp_alignment", "ik_feasibility")


def test_writer_cannot_overwrite_frozen_evidence():
    with pytest.raises(RuntimeError):
        assert_output_directory(ROOT / "evidence" / "P-008")
    with pytest.raises(RuntimeError):
        assert_output_directory(ROOT / "evidence" / "P-008-FIX1")


def test_frozen_failure_and_true_collision_remain():
    summary = (ROOT / "evidence" / "P-008" / "summary.md").read_text(encoding="utf-8")
    assert "\nFAIL\n" in summary
    root_cause = json.loads((ROOT / "evidence" / "P-008-FIX1" / "root_cause.json").read_text(encoding="utf-8"))
    assert root_cause["classification"] == "TRUE_COLLISION"
    assert root_cause["validator_modified"] is False


def test_designated_safe_cases_match_expected_and_complete(acceptance: dict):
    assert acceptance["designated_safe_pass"] == 4
    assert [row["case"] for row in acceptance["safe_cases"]] == list(DESIGNATED_SAFE)
    for row in acceptance["safe_cases"]:
        assert row["case_classification"] == "DESIGNATED_SAFE"
        assert row["expected_decision"] == "SAFE_TO_ATTEMPT"
        assert row["actual_decision"] == "SAFE_TO_ATTEMPT"
        assert row["actual_reason"] == []
        assert row["controller_submitted"] is True
        assert row["execution_completed"] is True
        assert row["execution_trace"] == ["ACCEPTED", "EXECUTING", "COMPLETED"]
        assert row["optional_not_applicable"] is True
        assert row["self_collision_evaluated"] is True
        assert row["result"] == "PASS"


def test_e_fixed_1_stays_a_known_collision_reject(acceptance: dict):
    row = acceptance["e_fixed_1"]
    assert row["case_classification"] == "KNOWN_COLLISION"
    assert row["expected_decision"] == "REJECT"
    assert row["actual_decision"] == "REJECT"
    assert row["expected_reason"] == "COLLISION"
    assert "COLLISION" in row["actual_reason"]
    assert row["collision_pair"] == {"a": "wrist_3_link", "b": "bolt_panel"}
    assert row["first_collision_sample"] == 44
    assert row["controller_submitted"] is False
    assert row["result"] == "PASS"


def test_negative_suite_rejects_without_reaching_the_controller(acceptance: dict):
    assert acceptance["negative_count"] >= 9
    assert acceptance["negative_pass"] == acceptance["negative_count"]
    for row in acceptance["reject_cases"]:
        assert row["expected_decision"] == "REJECT"
        assert row["actual_decision"] == "REJECT"
        assert row["expected_reason"] in row["actual_reason"]
        assert row["controller_submitted"] is False
        assert row["result"] == "PASS"


def test_gate_fail_closed_and_replay(acceptance: dict):
    gate = acceptance["execution_gate_results"]
    assert gate["safe_case"]["trace"] == ["ACCEPTED", "EXECUTING", "COMPLETED"]
    assert gate["safe_case"]["submit_count_after"] == gate["safe_case"]["submit_count_before"] + 1
    assert gate["rejected_collision_case"]["decision"] == "REJECT"
    assert gate["rejected_collision_case"]["submit_count_after"] == gate["rejected_collision_case"]["submit_count_before"]
    assert gate["invalid_input_case"]["decision"] == "REJECT"
    assert "NON_FINITE" in gate["invalid_input_case"]["reason_codes"]
    assert gate["invalid_input_case"]["submit_count_after"] == gate["invalid_input_case"]["submit_count_before"]
    assert gate["joint_limit_case"]["decision"] == "REJECT"
    assert "JOINT_LIMIT" in gate["joint_limit_case"]["reason_codes"]
    assert gate["joint_limit_case"]["submit_count_after"] == gate["joint_limit_case"]["submit_count_before"]
    assert acceptance["fail_closed"] == "PASS"
    assert acceptance["determinism_results"]["equal"] is True
    assert acceptance["timeout_count"] == 0
    assert acceptance["non_finite_count"] == 0
    assert acceptance["case_result"] == "PASS"
