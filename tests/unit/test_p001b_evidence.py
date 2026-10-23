"""저장된 FK evidence가 같은 함수의 재계산과 같은지 확인한다."""

import json
import math

from core.kinematics.evidence import TEST_COMMAND
from core.kinematics.fk import forward_kinematics
from core.model.ur10e import load_ur10e_model, repo_root


def test_saved_evidence_matches_fk_and_makes_no_accuracy_claim():
    root = repo_root()
    payload = json.loads((root / "evidence" / "P-001B" / "fk_results.json").read_text(encoding="utf-8"))
    summary = (root / "evidence" / "P-001B" / "summary.md").read_text(encoding="utf-8")
    model = load_ur10e_model()
    assert payload["external_fk_accuracy_claim"] is False
    assert "FK accuracy validated" not in summary
    assert "physical robot accuracy를 주장하지 않는다" in summary
    assert "Unity와 비교하지 않았다" in summary
    assert "Isaac Sim과 비교하지 않았다" in summary
    assert TEST_COMMAND in summary
    assert payload["source_commit"] == model.source_commit
    assert payload["base_frame"] == "base_link"
    assert payload["tcp_frame"] == "tool0"
    assert payload["actuated_joint_order"] == list(model.actuated_joint_order)
    assert len(payload["configurations"]) >= 10
    for row in payload["configurations"]:
        pose = forward_kinematics(model, row["joint_vector_rad"])
        assert math.isclose(pose.x, row["position_m"]["x"], rel_tol=0.0, abs_tol=1e-12)
        assert math.isclose(pose.y, row["position_m"]["y"], rel_tol=0.0, abs_tol=1e-12)
        assert math.isclose(pose.z, row["position_m"]["z"], rel_tol=0.0, abs_tol=1e-12)
        quat = row["quaternion_xyzw"]
        assert math.isclose(pose.qx, quat["x"], rel_tol=0.0, abs_tol=1e-12)
        assert math.isclose(pose.qy, quat["y"], rel_tol=0.0, abs_tol=1e-12)
        assert math.isclose(pose.qz, quat["z"], rel_tol=0.0, abs_tol=1e-12)
        assert math.isclose(pose.qw, quat["w"], rel_tol=0.0, abs_tol=1e-12)
        assert row["finite"] is True
        assert row["deterministic_rerun"] is True
