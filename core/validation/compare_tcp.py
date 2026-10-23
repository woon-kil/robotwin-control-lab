"""raw simulator evidence를 바꾸지 않고 canonical TCP를 비교한다."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from core.kinematics.fk import forward_kinematics
from core.model.ur10e import SOURCE_COMMIT, load_ur10e_model
from core.validation.tcp_compare import (
    CROSS_POSITION_LIMIT_MM,
    FK_ORIENTATION_LIMIT_DEG,
    FK_POSITION_LIMIT_MM,
    ISAAC_CONVENTION_ID,
    UNITY_CONVENTION_ID,
    classify_discrepancy,
    isaac_pose_to_canonical,
    orientation_error_rad,
    position_error_m,
    relative_pose,
    unity_pose_to_canonical,
)

SELECTED = ("A-zero", "B-positive-1", "C-mixed-1", "E-fixed-1", "E-fixed-3")


def _xyzw(pose: dict[str, float | str]) -> dict[str, float]:
    if "qx" in pose:
        return {"x": float(pose["qx"]), "y": float(pose["qy"]), "z": float(pose["qz"]), "w": float(pose["qw"])}
    return {"x": float(pose["x"]), "y": float(pose["y"]), "z": float(pose["z"]), "w": float(pose["w"])}


def pose_errors(reference: dict[str, float | str], test: dict[str, float | str]) -> dict[str, float]:
    position_m = position_error_m(reference, test)
    orientation_rad = orientation_error_rad(_xyzw(reference), _xyzw(test))
    return {
        "position_error_m": position_m,
        "position_error_mm": position_m * 1000.0,
        "orientation_error_rad": orientation_rad,
        "orientation_error_deg": orientation_rad * 180.0 / math.pi,
    }


def canonical_from_raw(source: str, base_pose: dict, tool_pose: dict) -> dict[str, float | str]:
    if source == "unity":
        world_base = unity_pose_to_canonical(base_pose["position_m"], base_pose["quaternion_xyzw"])
        world_tcp = unity_pose_to_canonical(tool_pose["position_m"], tool_pose["quaternion_xyzw"])
    elif source == "isaac":
        world_base = isaac_pose_to_canonical(base_pose["position_m"], base_pose["quaternion_wxyz"])
        world_tcp = isaac_pose_to_canonical(tool_pose["position_m"], tool_pose["quaternion_wxyz"])
    else:
        raise ValueError("unknown simulator source " + source)
    return relative_pose(world_base, world_tcp)


def fk_pose(robot, joints: list[float]) -> dict[str, float | str]:
    pose = forward_kinematics(robot, joints)
    return pose.as_dict()


def _row(robot, unity_row: dict, isaac_row: dict) -> dict[str, object]:
    unity_tcp = canonical_from_raw("unity", unity_row["base_pose"], unity_row["tool0_pose"])
    isaac_tcp = canonical_from_raw("isaac", isaac_row["base_pose"], isaac_row["tool0_pose"])
    python_target = fk_pose(robot, unity_row["target_joint_rad"])
    python_unity = fk_pose(robot, unity_row["actual_joint_rad"])
    python_isaac = fk_pose(robot, isaac_row["actual_joint_rad"])
    if unity_row["target_joint_rad"] != isaac_row["target_joint_rad"]:
        raise ValueError(unity_row["test_id"] + " target joints differ")
    pairs = {
        "python_vs_unity_actual_joint": pose_errors(python_unity, unity_tcp),
        "python_vs_isaac_actual_joint": pose_errors(python_isaac, isaac_tcp),
        "unity_vs_isaac": pose_errors(unity_tcp, isaac_tcp),
        "python_target_vs_unity": pose_errors(python_target, unity_tcp),
        "python_target_vs_isaac": pose_errors(python_target, isaac_tcp),
    }
    return {
        "test_id": unity_row["test_id"],
        "target_joint_rad": unity_row["target_joint_rad"],
        "unity_actual_joint_rad": unity_row["actual_joint_rad"],
        "isaac_actual_joint_rad": isaac_row["actual_joint_rad"],
        "python_fk_target": python_target,
        "python_fk_unity_actual": python_unity,
        "python_fk_isaac_actual": python_isaac,
        "unity_tcp_base": unity_tcp,
        "isaac_tcp_base": isaac_tcp,
        "python_vs_unity_actual_joint_position_error_mm": pairs["python_vs_unity_actual_joint"]["position_error_mm"],
        "python_vs_unity_actual_joint_orientation_error_deg": pairs["python_vs_unity_actual_joint"]["orientation_error_deg"],
        "python_vs_isaac_actual_joint_position_error_mm": pairs["python_vs_isaac_actual_joint"]["position_error_mm"],
        "python_vs_isaac_actual_joint_orientation_error_deg": pairs["python_vs_isaac_actual_joint"]["orientation_error_deg"],
        "unity_vs_isaac_position_error_mm": pairs["unity_vs_isaac"]["position_error_mm"],
        "unity_vs_isaac_orientation_error_deg": pairs["unity_vs_isaac"]["orientation_error_deg"],
        "python_target_vs_unity_position_error_mm": pairs["python_target_vs_unity"]["position_error_mm"],
        "python_target_vs_unity_orientation_error_deg": pairs["python_target_vs_unity"]["orientation_error_deg"],
        "python_target_vs_isaac_position_error_mm": pairs["python_target_vs_isaac"]["position_error_mm"],
        "python_target_vs_isaac_orientation_error_deg": pairs["python_target_vs_isaac"]["orientation_error_deg"],
    }


def _max_key(rows: list[dict], key: str) -> float:
    return max(float(row[key]) for row in rows)


def _rerun_delta(source: str, first: dict, rerun: dict) -> dict[str, float]:
    left = canonical_from_raw(source, first["base_pose"], first["tool0_pose"])
    right = canonical_from_raw(source, rerun["base_pose"], rerun["tool0_pose"])
    pose = pose_errors(left, right)
    joints = max(abs(a - b) for a, b in zip(first["actual_joint_rad"], rerun["actual_joint_rad"], strict=True))
    return {
        "joint_rerun_max_abs_rad": joints,
        "tool0_position_rerun_m": pose["position_error_m"],
        "tool0_orientation_rerun_deg": pose["orientation_error_deg"],
    }


def build_report(unity: dict, isaac: dict) -> dict[str, object]:
    robot = load_ur10e_model()
    unity_rows = {row["test_id"]: row for row in unity["configurations"]}
    isaac_rows = {row["test_id"]: row for row in isaac["configurations"]}
    if tuple(unity_rows) != SELECTED or tuple(isaac_rows) != SELECTED:
        raise ValueError("configurations are not the P-002/P-003 set")
    rows = [_row(robot, unity_rows[test_id], isaac_rows[test_id]) for test_id in SELECTED]
    failures: list[str] = []
    for row in rows:
        checks = (
            ("python_vs_unity_actual_joint_position_error_mm", FK_POSITION_LIMIT_MM, "Python ↔ Unity position"),
            ("python_vs_unity_actual_joint_orientation_error_deg", FK_ORIENTATION_LIMIT_DEG, "Python ↔ Unity orientation"),
            ("python_vs_isaac_actual_joint_position_error_mm", FK_POSITION_LIMIT_MM, "Python ↔ Isaac position"),
            ("python_vs_isaac_actual_joint_orientation_error_deg", FK_ORIENTATION_LIMIT_DEG, "Python ↔ Isaac orientation"),
            ("unity_vs_isaac_position_error_mm", CROSS_POSITION_LIMIT_MM, "Unity ↔ Isaac position"),
        )
        for key, limit, label in checks:
            if float(row[key]) >= limit:
                orientation = float(row["unity_vs_isaac_orientation_error_deg"]) if "Isaac" in label and "Unity ↔" in label else float(
                    row[key.replace("position_error_mm", "orientation_error_deg")]
                )
                category = classify_discrepancy(float(row[key]) if "position" in key else 0.0, orientation if "orientation" in key else float(row[key.replace("position_error_mm", "orientation_error_deg")]))
                failures.append(row["test_id"] + " " + label + " " + category)
    unity_rerun = _rerun_delta("unity", unity_rows["A-zero"], unity["rerun"])
    isaac_rerun = _rerun_delta("isaac", isaac_rows["A-zero"], isaac["rerun"])
    rerun_fail = (
        unity_rerun["tool0_position_rerun_m"] * 1000.0 >= FK_POSITION_LIMIT_MM
        or unity_rerun["tool0_orientation_rerun_deg"] >= FK_ORIENTATION_LIMIT_DEG
        or isaac_rerun["tool0_position_rerun_m"] * 1000.0 >= FK_POSITION_LIMIT_MM
        or isaac_rerun["tool0_orientation_rerun_deg"] >= FK_ORIENTATION_LIMIT_DEG
    )
    if rerun_fail:
        failures.append("deterministic rerun exceeds the existing FK position or orientation target")
    result = "FAIL" if failures else "PASS"
    return {
        "gate": "P-004",
        "result": result,
        "source_commit": SOURCE_COMMIT,
        "tcp_frame": "tool0",
        "reference_frame": "base_link",
        "unity_convention_id": UNITY_CONVENTION_ID,
        "isaac_convention_id": ISAAC_CONVENTION_ID,
        "targets": {
            "python_vs_unity_position_mm": FK_POSITION_LIMIT_MM,
            "python_vs_unity_orientation_deg": FK_ORIENTATION_LIMIT_DEG,
            "python_vs_isaac_position_mm": FK_POSITION_LIMIT_MM,
            "python_vs_isaac_orientation_deg": FK_ORIENTATION_LIMIT_DEG,
            "unity_vs_isaac_position_mm": CROSS_POSITION_LIMIT_MM,
            "unity_vs_isaac_orientation_deg": None,
            "unity_vs_isaac_orientation_note": "REQUIREMENTS and VALIDATION_PLAN specify cross-simulator position only",
        },
        "max_python_unity_position_error_mm": _max_key(rows, "python_vs_unity_actual_joint_position_error_mm"),
        "max_python_unity_orientation_error_deg": _max_key(rows, "python_vs_unity_actual_joint_orientation_error_deg"),
        "max_python_isaac_position_error_mm": _max_key(rows, "python_vs_isaac_actual_joint_position_error_mm"),
        "max_python_isaac_orientation_error_deg": _max_key(rows, "python_vs_isaac_actual_joint_orientation_error_deg"),
        "max_unity_isaac_position_error_mm": _max_key(rows, "unity_vs_isaac_position_error_mm"),
        "max_unity_isaac_orientation_error_deg": _max_key(rows, "unity_vs_isaac_orientation_error_deg"),
        "max_python_target_vs_unity_position_error_mm": _max_key(rows, "python_target_vs_unity_position_error_mm"),
        "max_python_target_vs_unity_orientation_error_deg": _max_key(rows, "python_target_vs_unity_orientation_error_deg"),
        "max_python_target_vs_isaac_position_error_mm": _max_key(rows, "python_target_vs_isaac_position_error_mm"),
        "max_python_target_vs_isaac_orientation_error_deg": _max_key(rows, "python_target_vs_isaac_orientation_error_deg"),
        "unity_rerun": unity_rerun,
        "isaac_rerun": isaac_rerun,
        "failures": failures,
        "configurations": rows,
    }


def render_summary(report: dict) -> str:
    return "\n".join(
        [
            "# P-004 Cross-simulator TCP",
            "",
            "```text",
            "Gate",
            "P-004",
            "",
            "Result",
            str(report["result"]),
            "",
            "Robot",
            "UR10e Golden Reference",
            "",
            "Source commit",
            str(report["source_commit"]),
            "",
            "Configurations",
            "5",
            "",
            "Python ↔ Unity",
            "Max Position:",
            format(float(report["max_python_unity_position_error_mm"]), ".17g") + " mm",
            "Max Orientation:",
            format(float(report["max_python_unity_orientation_error_deg"]), ".17g") + " deg",
            "",
            "Python ↔ Isaac",
            "Max Position:",
            format(float(report["max_python_isaac_position_error_mm"]), ".17g") + " mm",
            "Max Orientation:",
            format(float(report["max_python_isaac_orientation_error_deg"]), ".17g") + " deg",
            "",
            "Unity ↔ Isaac",
            "Max Position:",
            format(float(report["max_unity_isaac_position_error_mm"]), ".17g") + " mm",
            "Max Orientation:",
            format(float(report["max_unity_isaac_orientation_error_deg"]), ".17g") + " deg",
            "",
            "Deterministic rerun:",
            "FAIL" if any("rerun" in item for item in report["failures"]) else "PASS",
            "",
            "Known limitations:",
            "official nominal UR10e model",
            "not robot-specific calibration",
            "not compared with actual UR10e",
            "no physical robot accuracy claim",
            "TCP is tool0, not bolt_tool_tcp",
            "Unity ↔ Isaac orientation has no acceptance target in REQUIREMENTS or VALIDATION_PLAN",
            "```",
            "",
            "이 숫자는 동일 nominal UR10e 모델에 대한 독립 Python FK와 Unity/Isaac TCP transform consistency다. 실제 로봇 정확도가 아니다.",
            "",
        ]
    )


def write_report(evidence: Path, unity: dict, isaac: dict) -> dict[str, object]:
    report = build_report(unity, isaac)
    normalized = evidence / "normalized"
    normalized.mkdir(parents=True, exist_ok=True)
    (normalized / "unity_tcp.json").write_text(
        json.dumps(
            {
                "convention_id": UNITY_CONVENTION_ID,
                "configurations": [
                    {"test_id": row["test_id"], "tcp_base": row["unity_tcp_base"]} for row in report["configurations"]
                ],
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    (normalized / "isaac_tcp.json").write_text(
        json.dumps(
            {
                "convention_id": ISAAC_CONVENTION_ID,
                "configurations": [
                    {"test_id": row["test_id"], "tcp_base": row["isaac_tcp_base"]} for row in report["configurations"]
                ],
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    (normalized / "python_fk.json").write_text(
        json.dumps(
            {
                "implementation": "core.kinematics.fk.forward_kinematics",
                "configurations": [
                    {
                        "test_id": row["test_id"],
                        "python_fk_target": row["python_fk_target"],
                        "python_fk_unity_actual": row["python_fk_unity_actual"],
                        "python_fk_isaac_actual": row["python_fk_isaac_actual"],
                    }
                    for row in report["configurations"]
                ],
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    (evidence / "comparison_results.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    (evidence / "summary.md").write_text(render_summary(report), encoding="utf-8")
    (evidence / "environment.md").write_text(_environment(unity, isaac), encoding="utf-8")
    (evidence / "test_results.txt").write_text(
        "result " + str(report["result"]) + "\n" + "\n".join(report["failures"]) + "\n",
        encoding="utf-8",
    )
    return report


def _environment(unity: dict, isaac: dict) -> str:
    return "\n".join(
        [
            "# P-004 environment",
            "",
            "```text",
            "Unity version",
            str(unity.get("unity_version", "")),
            "",
            "Unity convention",
            UNITY_CONVENTION_ID,
            "",
            "Isaac version",
            str(isaac.get("isaac_version_file", "")),
            "",
            "Isaac convention",
            ISAAC_CONVENTION_ID,
            "",
            "Isaac pose API",
            str(isaac.get("tool0_observation_api", "")),
            "```",
            "",
        ]
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence", type=Path, default=None)
    parsed = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    evidence = parsed.evidence if parsed.evidence is not None else root / "evidence" / "P-004"
    unity = json.loads((evidence / "raw" / "unity_results.json").read_text(encoding="utf-8"))
    isaac = json.loads((evidence / "raw" / "isaac_results.json").read_text(encoding="utf-8"))
    report = write_report(evidence, unity, isaac)
    print(report["result"])
    print(report["max_python_unity_position_error_mm"], report["max_python_isaac_position_error_mm"], report["max_unity_isaac_position_error_mm"])
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
