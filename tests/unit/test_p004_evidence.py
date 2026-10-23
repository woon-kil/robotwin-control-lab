"""P-004 evidence가 raw pose를 유지한 채 canonical 비교를 기록하는지 확인한다."""

import json
import math
from pathlib import Path

from core.kinematics.fk import forward_kinematics
from core.model.ur10e import load_ur10e_model
from core.validation.compare_tcp import build_report
from core.validation.tcp_compare import CROSS_POSITION_LIMIT_MM, FK_ORIENTATION_LIMIT_DEG, FK_POSITION_LIMIT_MM

ROOT = Path(__file__).resolve().parents[2]
EVIDENCE = ROOT / "evidence" / "P-004"
SELECTED = ("A-zero", "B-positive-1", "C-mixed-1", "E-fixed-1", "E-fixed-3")


def test_p004_records_independent_tcp_and_keeps_targets():
    unity = json.loads((EVIDENCE / "raw" / "unity_results.json").read_text())
    isaac = json.loads((EVIDENCE / "raw" / "isaac_results.json").read_text())
    report = json.loads((EVIDENCE / "comparison_results.json").read_text())
    summary = (EVIDENCE / "summary.md").read_text()
    rebuilt = build_report(unity, isaac)
    robot = load_ur10e_model()

    assert unity["normalization_applied"] is False
    assert isaac["normalization_applied"] is False
    assert "RigidPrim" not in isaac["tool0_observation_api"]
    assert [row["test_id"] for row in unity["configurations"]] == list(SELECTED)
    assert [row["test_id"] for row in isaac["configurations"]] == list(SELECTED)
    assert report["result"] == "FAIL"
    assert report["targets"]["unity_vs_isaac_orientation_deg"] is None
    assert report["max_python_unity_position_error_mm"] < FK_POSITION_LIMIT_MM
    assert report["max_python_unity_orientation_error_deg"] < FK_ORIENTATION_LIMIT_DEG
    assert report["max_python_isaac_position_error_mm"] >= FK_POSITION_LIMIT_MM
    assert report["max_python_isaac_orientation_error_deg"] < FK_ORIENTATION_LIMIT_DEG
    assert report["max_unity_isaac_position_error_mm"] >= CROSS_POSITION_LIMIT_MM
    assert math.isclose(report["max_python_isaac_position_error_mm"], rebuilt["max_python_isaac_position_error_mm"])
    assert "actual UR10e" in summary or "실제 로봇" in summary
    assert "Isaac TCP accuracy" not in summary

    for row, raw in zip(report["configurations"], unity["configurations"], strict=True):
        fk = forward_kinematics(robot, raw["actual_joint_rad"])
        observed = raw["tool0_pose"]["position_m"]
        assert not math.isclose(observed["x"], fk.x, abs_tol=1e-6)
        assert row["unity_tcp_base"]["reference_frame"] == "base_link"
        assert row["unity_tcp_base"]["frame"] == "tool0"
        assert "qx" in row["python_fk_target"]
