"""P-010 operator HMI. Validator와 monitor 결과를 다시 계산하지 않는다."""

import math
import re
from pathlib import Path

from hmi.actions import OperatorSession
from hmi.model import HmiSnapshot
from hmi.render import render
from hmi.view_model import DEMO_SAFE, format_deg_from_rad, format_mm, format_quantity, snapshot_from_run

ROOT = Path(__file__).resolve().parents[2]
ZERO = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
POSE = {
    "frame": "tool0",
    "reference_frame": "base_link",
    "position_m": [1.0, 0.0, 0.0],
    "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
}


def test_initial_state_is_not_a_pass():
    session = OperatorSession(runner=_fail_runner, validate_fn=_fail_validate)
    page = render(session.snapshot)
    assert session.snapshot.validation_decision == "NOT RUN"
    assert session.snapshot.execution_state == "IDLE"
    assert session.snapshot.execute_enabled is False
    assert session.snapshot.controller_submitted is False
    assert "NOT RUN" in page
    assert "Execute disabled" in page
    assert "SAFE_TO_ATTEMPT" not in page
    assert "PROJECT TEST" in page


def test_non_finite_display_is_invalid():
    assert format_quantity(float("nan"), "°") == "INVALID"
    assert format_quantity(float("inf"), " mm") == "INVALID"
    assert format_quantity(float("-inf"), " rad") == "INVALID"
    assert "0.00" not in format_quantity(float("nan"), "°")
    snapshot = _tracked_snapshot()
    snapshot.joint_error = [float("nan"), 0.1, 0.0, 0.0, 0.0, 0.0]
    snapshot.tcp_position_error_m = float("nan")
    snapshot.series = [{"time_s": 0.0, "max_abs_joint_error_rad": None}]
    page = render(snapshot)
    assert "INVALID" in page
    assert "DATA ERROR" in page
    assert "Position Error: INVALID" in page


def test_safe_validation_enables_execute_and_reject_blocks_the_runner():
    safe = OperatorSession(runner=_fail_runner, validate_fn=lambda name: _report("SAFE_TO_ATTEMPT", summary="SAFE_TO_ATTEMPT — pre-execution checks passed"))
    safe.select(DEMO_SAFE)
    safe.validate()
    assert safe.snapshot.validation_decision == "SAFE_TO_ATTEMPT"
    assert safe.snapshot.execute_enabled is True
    assert "Pre-execution project checks passed." in render(safe.snapshot)
    safe.select("C-mixed-1")
    assert safe.snapshot.validation_decision == "NOT RUN"
    assert safe.snapshot.execute_enabled is False
    safe.execute()
    assert safe.snapshot.controller_submitted is False
    assert safe.snapshot.backend_blocked is True

    calls = []

    def runner(*args, **kwargs):
        calls.append((args, kwargs))
        raise AssertionError("rejected command reached the runner")

    rejected = OperatorSession(runner=runner, validate_fn=lambda name: _collision_report())
    rejected.select("E-fixed-1")
    rejected.validate()
    page = render(rejected.snapshot)
    assert rejected.snapshot.validation_decision == "REJECT"
    assert rejected.snapshot.execute_enabled is False
    assert "COLLISION" in page
    assert "wrist_3_link" in page
    assert "bolt_panel" in page
    assert "44 / 100" in page
    assert "Execute disabled" in page
    rejected.execute()
    assert calls == []
    assert rejected.snapshot.controller_submitted is False
    assert rejected.snapshot.tracking_started is False


def test_target_actual_tcp_and_fault_mapping_keep_monitor_values():
    stored_error = [9.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    result = _run_result(
        samples=[
            _sample(ZERO, ZERO, stored_error, [9.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.0, tcp_error=0.123, orientation=0.02),
            _sample([0.2] * 6, [0.2] * 6, ZERO, ZERO, 0.23, tcp_error=0.0, orientation=0.0, state="READY", execution="COMPLETED"),
        ],
        metrics={
            "global_max_joint_error_rad": 9.0,
            "completion_time_s": 0.23,
            "settled_time_s": 0.25,
        },
        completed=True,
        settled=True,
        settling_time_s=0.24,
    )
    snapshot = snapshot_from_run(result)
    assert snapshot.joint_error == ZERO
    assert snapshot.peak_joint_error == stored_error
    assert snapshot.target_q == snapshot.actual_q
    assert snapshot.tcp_position_error_m == 0.0
    assert snapshot.peak_joint_error_rad == 9.0
    assert snapshot.peak_target_q == snapshot.peak_actual_q
    assert max(row["max_abs_joint_error_rad"] for row in snapshot.series) == 9.0
    page = render(snapshot)
    assert format_mm(0.0) in page
    assert "COMPLETED" in page
    assert "SETTLED" in page
    assert _marker_x(page, "COMPLETED") != _marker_x(page, "SETTLED")
    assert "CERTIFIED" not in page

    fault = snapshot_from_run(
        _run_result(
            samples=[_sample([0.2] * 6, ZERO, [0.2] * 6, [0.2] * 6, 0.01, tcp_error=0.4, orientation=0.5)],
            metrics={"global_max_joint_error_rad": 0.2, "completion_time_s": 0.2, "settled_time_s": 0.2},
            completed=False,
            settled=False,
            fault=True,
        )
    )
    fault_page = render(fault)
    assert fault.fault is True
    assert fault.completed is False
    assert fault.settled is False
    assert fault.controller_state == "ERROR"
    assert fault.execution_state == "FAULTED"
    assert fault.ui_state == "FAULT"
    assert fault.tcp_position_error_m == 0.4
    assert format_mm(0.4) in fault_page
    assert format_deg_from_rad(0.5) in fault_page
    assert ">FAULT<" in fault_page or "FAULT" in fault_page
    assert "COMPLETED" not in fault_page
    assert "SETTLED" not in fault_page
    assert "SUCCESS" not in fault_page
    assert "Settled: NO" in fault_page


def test_live_safe_reject_and_fault_use_the_project_validator():
    safe = OperatorSession()
    safe.select(DEMO_SAFE)
    safe.validate()
    assert safe.snapshot.validation_decision == "SAFE_TO_ATTEMPT"
    assert safe.snapshot.execute_enabled is True
    assert safe.snapshot.controller_submitted is False
    safe.execute()
    assert safe.snapshot.controller_submitted is True
    assert safe.snapshot.tracking_started is True
    assert safe.snapshot.completed is True
    assert safe.snapshot.settled is True
    assert safe.snapshot.completion_time_s != safe.snapshot.settled_at_s
    assert safe.snapshot.peak_target_q != safe.snapshot.peak_actual_q
    assert safe.snapshot.peak_joint_error_rad == max(row["max_abs_joint_error_rad"] for row in safe.snapshot.series)
    safe_page = render(safe.snapshot)
    assert "SAFE_TO_ATTEMPT" in safe_page
    assert "COMPLETED" in safe_page
    assert "SETTLED" in safe_page
    assert "Controller: READY" in safe_page
    assert "°" in safe_page
    assert " mm" in safe_page

    calls = []
    rejected = OperatorSession()
    rejected.select("E-fixed-1")
    rejected.validate()
    real_runner = rejected._runner

    def guard(*args, **kwargs):
        calls.append(args)
        return real_runner(*args, **kwargs)

    rejected._runner = guard
    rejected.execute()
    assert calls == []
    assert rejected.snapshot.validation_decision == "REJECT"
    assert "COLLISION" in rejected.snapshot.validation_reasons
    assert rejected.snapshot.collision_pair == {"a": "wrist_3_link", "b": "bolt_panel"}
    assert rejected.snapshot.collision_sample == "44 / 100"
    assert rejected.snapshot.execute_enabled is False
    assert rejected.snapshot.controller_submitted is False
    assert rejected.snapshot.tracking_started is False

    faulted = OperatorSession()
    faulted.select(DEMO_SAFE)
    faulted.validate()
    faulted.fault()
    assert faulted.snapshot.fault is True
    assert faulted.snapshot.completed is False
    assert faulted.snapshot.settled is False
    assert faulted.snapshot.ui_state == "FAULT"
    assert faulted.snapshot.controller_state == "ERROR"
    assert faulted.snapshot.execution_state == "FAULTED"
    assert faulted.snapshot.mode == "LIVE PROJECT RUN"
    assert "SUCCESS" not in render(faulted.snapshot)


def test_core_packages_do_not_import_hmi():
    for folder in ("controller", "validator", "digital_twin"):
        for path in (ROOT / folder).rglob("*.py"):
            text = path.read_text(encoding="utf-8")
            assert "import hmi" not in text
            assert "from hmi" not in text


def _report(decision: str, reasons: list[str] | None = None, checks: list[dict] | None = None, summary: str = "") -> dict:
    return {
        "decision": decision,
        "blocking_reason_codes": reasons or [],
        "summary": summary,
        "checks": checks or [],
    }


def _collision_report() -> dict:
    return _report(
        "REJECT",
        ["COLLISION"],
        [
            {
                "name": "robot_environment_collision",
                "result": "FAIL",
                "reason_code": "COLLISION",
                "metrics": {
                    "sample_index": 44,
                    "sample_count": 100,
                    "involved_objects": [{"a": "wrist_3_link", "b": "bolt_panel"}],
                },
            }
        ],
        "REJECT — wrist_3_link intersects bolt_panel at sample 44/100",
    )


def _sample(target, actual, error, absolute, time_s, tcp_error, orientation, state="RUNNING", execution="EXECUTING"):
    return {
        "timestamp_s": time_s,
        "controller_state": state,
        "execution_state": execution,
        "target_q": list(target),
        "actual_q": list(actual),
        "joint_error_rad": list(error),
        "abs_joint_error_rad": list(absolute),
        "target_tcp": dict(POSE),
        "actual_tcp": dict(POSE),
        "tcp_position_error_m": tcp_error,
        "tcp_orientation_error_rad": orientation,
    }


def _run_result(samples, metrics, completed, settled, settling_time_s=None, fault=False):
    return {
        "case": DEMO_SAFE,
        "validation_decision": "SAFE_TO_ATTEMPT",
        "blocking_reason_codes": [],
        "controller_submitted": True,
        "tracking_started": True,
        "completed": completed,
        "settled": settled,
        "settling_time_s": settling_time_s,
        "samples": samples,
        "metrics": metrics,
        "fault_occurred": fault,
        "fault_observation": {"controller_state": "ERROR", "execution_state": "FAULTED", "fault": "TEST_FAULT"} if fault else None,
    }


def _tracked_snapshot() -> HmiSnapshot:
    snapshot = snapshot_from_run(
        _run_result(
            samples=[_sample(ZERO, ZERO, ZERO, ZERO, 0.0, tcp_error=0.001, orientation=0.001, state="READY", execution="COMPLETED")],
            metrics={"global_max_joint_error_rad": 0.0, "completion_time_s": 0.0, "settled_time_s": 0.02},
            completed=True,
            settled=True,
            settling_time_s=0.02,
        )
    )
    return snapshot


def _marker_x(page: str, label: str) -> float:
    match = re.search(rf'<line x1="([0-9.]+)"[^>]*></line><text[^>]*>{label}</text>', page)
    assert match is not None
    return float(match.group(1))


def _fail_runner(*args, **kwargs):
    raise AssertionError("runner")


def _fail_validate(name: str):
    raise AssertionError(name)


def test_degrees_have_units():
    assert format_deg_from_rad(math.radians(20.0)) == "20.00°"
    assert format_mm(0.00035) == "0.35 mm"
