"""P-002 evidence가 P-001B joint vector와 측정 판정을 그대로 기록하는지 확인한다."""

import json
import math
from pathlib import Path

from core.kinematics.configurations import baseline_configurations

ROOT = Path(__file__).resolve().parents[2]
RESULTS = ROOT / "evidence" / "P-002" / "joint_results.json"
SUMMARY = ROOT / "evidence" / "P-002" / "summary.md"
ATTEMPT1 = ROOT / "evidence" / "P-002" / "attempt1" / "joint_results.json"
SELECTED = ("A-zero", "B-positive-1", "C-mixed-1", "E-fixed-1", "E-fixed-3")
ORDER = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


def test_p002_records_the_measured_joint_baseline():
    data = json.loads(RESULTS.read_text())
    summary = SUMMARY.read_text()
    attempt1 = json.loads(ATTEMPT1.read_text())
    baseline = {row["test_id"]: row["joint_vector_rad"] for row in baseline_configurations()}
    rows = data["configurations"]

    assert data["source_commit"] == "7ce98d8c3512b9d0239da93b75de68ea64f61bd6"
    assert data["canonical_joint_order"] == list(ORDER)
    assert [row["test_id"] for row in rows] == list(SELECTED)
    assert data["invalid_joint_count_rejected"] is True
    assert "FK accuracy validated" not in summary
    assert "0.5 mm" not in summary

    target_rad = data["engineering_target_deg"] * math.pi / 180.0
    global_max = 0.0
    for row in rows:
        assert row["target_joint_rad"] == list(baseline[row["test_id"]]) or _close(row["target_joint_rad"], baseline[row["test_id"]])
        assert len(row["actual_joint_rad"]) == 6
        assert len(row["error_joint_rad"]) == 6
        assert row["target_joint_rad"] != row["actual_joint_rad"]
        for target, actual, error in zip(row["target_joint_rad"], row["actual_joint_rad"], row["error_joint_rad"], strict=True):
            assert math.isclose(error, actual - target, rel_tol=0.0, abs_tol=1e-12)
            assert math.isfinite(actual)
        assert math.isclose(row["max_abs_error_rad"], max(abs(value) for value in row["error_joint_rad"]), rel_tol=0.0, abs_tol=1e-12)
        global_max = max(global_max, row["max_abs_error_rad"])
        pose = row["tool0_pose"]
        assert pose["frame"] == "unity_world"
        for value in pose["position_m"].values():
            assert math.isfinite(value)
        for value in pose["quaternion_xyzw"].values():
            assert math.isfinite(value)

    assert math.isclose(data["global_max_joint_error_rad"], global_max, rel_tol=0.0, abs_tol=1e-12)
    timed_out = any(row["timed_out"] for row in rows)
    expected = "PASS" if global_max < target_rad and not timed_out else "FAIL"
    assert data["result"] == expected
    assert f"Result\n{expected}" in summary
    assert attempt1["result"] == "FAIL"
    assert attempt1["global_max_joint_error_rad"] > data["global_max_joint_error_rad"]


def _close(left, right) -> bool:
    return all(math.isclose(a, b, rel_tol=0.0, abs_tol=1e-12) for a, b in zip(left, right, strict=True))
