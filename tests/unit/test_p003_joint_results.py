"""P-003 evidence가 canonical joint command와 Isaac feedback 판정을 기록하는지 확인한다."""

import json
import math
from pathlib import Path

import pytest

from core.kinematics.configurations import baseline_configurations
from isaac.ur10e_commands import CONFIGURATIONS, Ur10eCommandError, map_canonical_dofs, validate_joint_targets

ROOT = Path(__file__).resolve().parents[2]
EVIDENCE = ROOT / "evidence" / "P-003"
RESULTS = EVIDENCE / "joint_results.json"
SUMMARY = EVIDENCE / "summary.md"
IMPORT_REPORT = EVIDENCE / "import_report.json"
SELECTED = ("A-zero", "B-positive-1", "C-mixed-1", "E-fixed-1", "E-fixed-3")
ORDER = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
LIMITATIONS = (
    "official nominal UR10e model",
    "not robot-specific calibration",
    "not compared with actual UR10e",
    "not yet compared with Unity",
    "not yet compared with Python FK",
    "no physical robot accuracy claim",
    "Isaac controller/drive is simulation test configuration",
)


def test_commands_reject_invalid_input_and_do_not_assume_dof_order():
    with pytest.raises(Ur10eCommandError) as count_error:
        validate_joint_targets([0.1, 0.1, 0.1, 0.1, 0.1])
    assert count_error.value.code == "JOINT_COUNT"
    with pytest.raises(Ur10eCommandError) as finite_error:
        validate_joint_targets([0.0, 0.0, math.nan, 0.0, 0.0, 0.0])
    assert finite_error.value.code == "NON_FINITE"
    with pytest.raises(Ur10eCommandError) as missing_error:
        map_canonical_dofs(["shoulder_pan_joint"])
    assert missing_error.value.code == "MAPPING"
    with pytest.raises(Ur10eCommandError) as duplicate_error:
        map_canonical_dofs(list(ORDER) + ["shoulder_pan_joint"])
    assert duplicate_error.value.code == "MAPPING"
    reversed_names = list(reversed(ORDER))
    mapping = map_canonical_dofs(reversed_names)
    assert [row["canonical_name"] for row in mapping] == list(ORDER)
    assert [row["isaac_dof_index"] for row in mapping] == [5, 4, 3, 2, 1, 0]


def test_selected_vectors_match_p001b_baseline():
    baseline = {row["test_id"]: row["joint_vector_rad"] for row in baseline_configurations()}
    assert [item[0] for item in CONFIGURATIONS] == list(SELECTED)
    for test_id, joints in CONFIGURATIONS:
        assert joints == baseline[test_id]


def test_p003_records_the_measured_joint_baseline():
    data = json.loads(RESULTS.read_text())
    summary = SUMMARY.read_text()
    report = json.loads(IMPORT_REPORT.read_text())
    attempt1 = json.loads((EVIDENCE / "attempt1" / "joint_results.json").read_text())
    attempt2 = json.loads((EVIDENCE / "attempt2" / "joint_results.json").read_text())
    baseline = {row["test_id"]: row["joint_vector_rad"] for row in baseline_configurations()}
    rows = data["configurations"]

    assert data["source_commit"] == "7ce98d8c3512b9d0239da93b75de68ea64f61bd6"
    assert data["canonical_joint_order"] == list(ORDER)
    assert [row["test_id"] for row in rows] == list(SELECTED)
    assert data["invalid_joint_count_rejected"] is True
    assert data["non_finite_rejected"] is True
    assert data["unknown_mapping_rejected"] is True
    assert data["timeouts"] == 0
    assert math.isfinite(data["deterministic_rerun_max_abs_rad"])
    assert "Isaac TCP accuracy" not in summary
    assert "more accurate than Unity" not in summary
    for line in LIMITATIONS:
        assert line in summary

    names = [row["canonical_name"] for row in report["joint_mapping"]]
    assert names == list(ORDER)
    assert len(names) == len(set(names))
    assert report["fixed_base"] is True
    assert report["source_commit"] == data["source_commit"]

    target_rad = data["engineering_target_deg"] * math.pi / 180.0
    global_max = 0.0
    for row in rows:
        assert _close(row["target_joint_rad"], baseline[row["test_id"]])
        assert len(row["actual_joint_rad"]) == 6
        assert row["target_joint_rad"] != row["actual_joint_rad"]
        assert row["timed_out"] is False
        for target, actual, error in zip(row["target_joint_rad"], row["actual_joint_rad"], row["error_joint_rad"], strict=True):
            assert math.isclose(error, actual - target, rel_tol=0.0, abs_tol=1e-12)
            assert math.isfinite(actual)
        assert math.isclose(row["max_abs_error_rad"], max(abs(value) for value in row["error_joint_rad"]), rel_tol=0.0, abs_tol=1e-12)
        global_max = max(global_max, row["max_abs_error_rad"])
        pose = row["tool0_pose"]
        assert pose["frame"] == "isaac_world"
        assert pose["quaternion_order"] == "wxyz"
        for value in pose["position_m"].values():
            assert math.isfinite(value)
        for value in pose["quaternion_wxyz"].values():
            assert math.isfinite(value)

    assert math.isclose(data["global_max_joint_error_rad"], global_max, rel_tol=0.0, abs_tol=1e-12)
    expected = "PASS" if global_max < target_rad and data["timeouts"] == 0 else "FAIL"
    assert data["result"] == expected
    assert f"Result\n{expected}" in summary
    assert attempt1["result"] == "FAIL"
    assert attempt2["result"] == "FAIL"
    assert attempt2["global_max_joint_error_deg"] > data["engineering_target_deg"]
    e3 = next(row for row in rows if row["test_id"] == "E-fixed-3")
    assert e3["tool0_pose"]["position_m"]["z"] == -40.485015869140625


def test_fix_attempt1_rejects_falling_tool0_and_keeps_joint_target():
    folder = EVIDENCE / "fix-attempt1"
    data = json.loads((folder / "joint_results.json").read_text())
    tool = json.loads((folder / "tool0_results.json").read_text())
    summary = (folder / "summary.md").read_text()
    diagnostics = (folder / "diagnostics.md").read_text()
    baseline = {row["test_id"]: row["joint_vector_rad"] for row in baseline_configurations()}
    rows = data["configurations"]
    target_rad = data["engineering_target_deg"] * math.pi / 180.0

    assert data["result"] == "PASS"
    assert data["timeouts"] == 0
    assert data["cumulative_world_drift"] is False
    assert data["global_max_joint_error_rad"] < target_rad
    assert data["tool0_base_distance_max_m"] <= data["tool0_base_distance_limit_m"]
    assert data["tool0_base_distance_limit_name"] == "PROJECT SANITY CHECK"
    assert data["link_distance_span_max_m"] <= data["link_distance_span_limit_m"]
    assert data["invalid_joint_count_rejected"] is True
    assert data["non_finite_rejected"] is True
    assert data["unknown_mapping_rejected"] is True
    assert "RigidPrim" not in data["tool0_observation_api"]
    assert "Isaac TCP accuracy" not in summary
    assert "Initial P-003" in summary and "Overall:\nFAIL" in summary
    assert "Fix Attempt 1" in diagnostics
    assert [row["test_id"] for row in rows] == list(SELECTED)
    assert [row["test_id"] for row in tool["configurations"]] == list(SELECTED)
    previous_xy = None
    for row in rows:
        assert _close(row["target_joint_rad"], baseline[row["test_id"]])
        assert row["target_joint_rad"] != row["actual_joint_rad"]
        assert row["timed_out"] is False
        pose = row["tool0_pose"]
        assert pose["frame"] == "isaac_world"
        assert pose["quaternion_order"] == "wxyz"
        assert pose["rigid_body_api_before_read"] is False
        assert all(math.isfinite(value) for value in pose["position_m"].values())
        assert all(math.isfinite(value) for value in pose["quaternion_wxyz"].values())
        assert row["tool0_base_distance_m"] <= 2.0
        xy = (pose["position_m"]["x"], pose["position_m"]["y"])
        if previous_xy is not None:
            assert math.dist(previous_xy, xy) > 0.01
        previous_xy = xy
        span_values = list(row["link_pair_distance_m"].values())
        assert all(math.isfinite(value) for value in span_values)


def _close(left, right) -> bool:
    return all(math.isclose(a, b, rel_tol=0.0, abs_tol=1e-12) for a, b in zip(left, right, strict=True))
