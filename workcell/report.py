"""P-005 evidence 판정. 측정값을 바꾸지 않는다."""

from __future__ import annotations

import json
from pathlib import Path


def judge_workcell(measurements: dict) -> dict:
    limits = measurements["acceptance"]
    collision_measured = (
        measurements["unexpected_collision_count"] is not None
        and measurements["initial_penetration_count"] is not None
    )
    checks = [
        _check("scene_reproducible", measurements["build_position_difference_m"] <= limits["build_position_m_max"]),
        _check("robot_base_authored", measurements["base_authoring_error_m"] <= limits["build_position_m_max"]),
        _check(
            "tool_collision_authored",
            measurements["tool_collision_authoring_error_m"] <= limits["build_position_m_max"],
        ),
        _check("bolt_targets_authored", measurements["bolt_authoring_error_m"] <= limits["build_position_m_max"]),
        _check(
            "robot_base_at_configured_pose",
            measurements["initial_base_error_m"] <= limits["base_displacement_m_max"],
        ),
        _check("required_components_present", measurements["required_prims_present"] is True),
        _check("required_frames_present", measurements["required_frames_present"] is True),
        _check("collision_geometry_present", measurements["collision_geometry_present"] is True),
        _check("panel_collision_query", measurements["panel_collision_query_hit"] is True),
        _check("finite_transforms", measurements["finite_transforms"] is True),
        _check("non_finite_state", measurements["non_finite_count"] <= limits["non_finite_count_max"]),
        _check("robot_base_fixed", measurements["base_displacement_m"] <= limits["base_displacement_m_max"]),
        _check("robot_does_not_fall", measurements["max_joint_drift_rad"] <= limits["joint_drift_rad_max"]),
        _check("tool0_stable", measurements["tool0_drift_m"] <= limits["tool0_drift_m_max"]),
        _check("floor_static", measurements["floor_displacement_m"] <= limits["static_displacement_m_max"]),
        _check(
            "work_structure_static",
            measurements["work_structure_displacement_m"] <= limits["static_displacement_m_max"],
        ),
        _check("bolt_panel_static", measurements["bolt_panel_displacement_m"] <= limits["static_displacement_m_max"]),
        _check("fixture_static", measurements["pedestal_displacement_m"] <= limits["static_displacement_m_max"]),
        _check(
            "unexpected_initial_collision",
            collision_measured
            and measurements["unexpected_collision_count"] <= limits["unexpected_collision_count_max"],
        ),
        _check(
            "initial_penetration",
            collision_measured
            and measurements["initial_penetration_count"] <= limits["initial_penetration_count_max"],
        ),
        _check("reset_robot_base", measurements["reset_robot_base_difference_m"] <= limits["reset_position_m_max"]),
        _check("reset_joints", measurements["reset_joint_difference_rad"] <= limits["reset_joint_rad_max"]),
        _check("reset_tool0", measurements["reset_tool0_difference_m"] <= limits["reset_position_m_max"]),
        _check("reset_bolt_panel", measurements["reset_bolt_panel_difference_m"] <= limits["reset_position_m_max"]),
        _check(
            "bolt_targets_reproducible",
            measurements["bolt_targets_build_difference_m"] <= limits["build_position_m_max"],
        ),
    ]
    result = "PASS" if all(item["result"] == "PASS" for item in checks) else "FAIL"
    return {"result": result, "checks": checks}


def write_evidence(directory: Path, payload: dict) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    _write_json(directory / "workcell_config.json", payload["workcell_config"])
    _write_json(directory / "scene_validation.json", payload["scene_validation"])
    _write_json(directory / "stability_results.json", payload["stability_results"])
    _write_json(directory / "reset_results.json", payload["reset_results"])
    _write_json(directory / "collision_results.json", payload["collision_results"])
    (directory / "environment.md").write_text(payload["environment_md"], encoding="utf-8")
    (directory / "summary.md").write_text(payload["summary_md"], encoding="utf-8")
    (directory / "test_results.txt").write_text(payload["test_results_txt"], encoding="utf-8")


def render_test_results(judgement: dict) -> str:
    lines = [item["name"] + " " + item["result"] for item in judgement["checks"]]
    lines.append("RESULT " + judgement["result"])
    return "\n".join(lines) + "\n"


def render_summary(claim: str, judgement: dict, highlights: dict) -> str:
    lines = [
        "# P-005",
        "",
        claim,
        "",
        "이 결과는 UR10e-based reference workcell의 PROJECT TEST CONDITION이다.",
        "현장 안전 검증, FilterBot, bolt fastening 검증이 아니다.",
        "Reach 비교는 geometric sanity이며 IK validation이 아니다.",
        "",
        "Result",
        judgement["result"],
        "",
        "Robot base displacement m",
        _text(highlights.get("base_displacement_m")),
        "",
        "Max joint drift rad",
        _text(highlights.get("max_joint_drift_rad")),
        "",
        "Tool0 drift m",
        _text(highlights.get("tool0_drift_m")),
        "",
        "Unexpected initial collisions",
        _text(highlights.get("unexpected_collision_count")),
        "",
        "Initial penetrations",
        _text(highlights.get("initial_penetration_count")),
        "",
        "Reset robot base difference m",
        _text(highlights.get("reset_robot_base_difference_m")),
        "",
        "Deterministic build difference m",
        _text(highlights.get("build_position_difference_m")),
        "",
    ]
    return "\n".join(lines)


def _check(name: str, passed: bool) -> dict[str, str]:
    return {"name": name, "result": "PASS" if passed else "FAIL"}


def _write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _text(value: object) -> str:
    if value is None:
        return "NOT_MEASURED"
    return str(value)
