"""P-009 digital twin monitor. HMI와 P-010은 만들지 않는다."""

import json
import math
from pathlib import Path

import pytest

from controller.virtual_pmac import COMPLETION_TOLERANCE_RAD
from core.model.ur10e import load_ur10e_model
from core.validation.tcp_compare import orientation_error_rad, position_error_m
from digital_twin.metrics import joint_error_rad, tcp_pair
from digital_twin.monitor import DigitalTwinMonitor, load_monitor_config
from digital_twin.session import DESIGNATED_SAFE, run_case
from validator.validate import load_validator_config
from workcell.layout import resolve_workcell

ROOT = Path(__file__).resolve().parents[2]
ZERO = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


@pytest.fixture(scope="module")
def model():
    return load_ur10e_model()


@pytest.fixture(scope="module")
def monitor_config() -> dict:
    return load_monitor_config()


def test_joint_error_is_target_minus_actual():
    error = joint_error_rad((1.0, -0.5, 0.0, 0.0, 0.0, 0.0), ZERO)
    assert error == (1.0, -0.5, 0.0, 0.0, 0.0, 0.0)
    assert tuple(abs(value) for value in error)[0] == 1.0


def test_tcp_errors_use_the_same_fk_and_shortest_rotation(model):
    same = tcp_pair(model, ZERO, ZERO)
    assert same["tcp_position_error_m"] == 0.0
    assert same["tcp_orientation_error_rad"] == 0.0
    moved = tcp_pair(model, (0.2, 0.0, 0.0, 0.0, 0.0, 0.0), ZERO)
    assert moved["tcp_position_error_m"] > 0.0
    assert moved["target_tcp"]["position_m"] != moved["actual_tcp"]["position_m"]
    assert position_error_m({"x": 3.0, "y": 0.0, "z": 0.0}, {"x": 0.0, "y": 0.0, "z": 0.0}) == 3.0
    error = orientation_error_rad(_yaw(170.0), _yaw(-170.0))
    assert error == pytest.approx(math.radians(20.0))
    assert abs(170.0 - (-170.0)) != pytest.approx(20.0)


def test_settling_requires_consecutive_samples_after_execution_starts(model, monitor_config):
    early = DigitalTwinMonitor(model, monitor_config)
    early.begin("once")
    early.observe(ZERO, ZERO, "RUNNING", "ACCEPTED", 0.0)
    early.observe(ZERO, ZERO, "READY", "COMPLETED", 0.01)
    finished = early.finish(("ACCEPTED", "EXECUTING", "COMPLETED"))
    assert finished["completion_state"] == "COMPLETED"
    assert finished["settled"] is False
    assert early.status == "TRACKING"

    settled = DigitalTwinMonitor(model, monitor_config)
    settled.begin("three")
    settled.observe(ZERO, ZERO, "RUNNING", "ACCEPTED", 0.0)
    settled.observe(ZERO, ZERO, "READY", "COMPLETED", 0.01)
    settled.observe(ZERO, ZERO, "READY", "COMPLETED", 0.02)
    assert settled.settled is True
    assert settled.status == "SETTLED"
    assert settled.finish(("ACCEPTED", "EXECUTING", "COMPLETED"))["settling_time_s"] == pytest.approx(0.01)


def test_fault_and_non_finite_do_not_become_settled(model, monitor_config):
    monitor = DigitalTwinMonitor(model, monitor_config)
    monitor.begin("fault")
    target = (0.2, 0.0, 0.0, 0.0, 0.0, 0.0)
    monitor.observe(target, ZERO, "RUNNING", "EXECUTING", 0.01)
    assert monitor.samples[0].target_q != monitor.samples[0].actual_q
    monitor.observe(ZERO, ZERO, "ERROR", "FAULTED", 0.02, "TEST_FAULT")
    monitor.observe(ZERO, ZERO, "READY", "COMPLETED", 0.03)
    finished = monitor.finish(("ACCEPTED", "EXECUTING", "FAULTED"))
    assert finished["settled"] is False
    assert finished["status"] == "ERROR"
    assert finished["fault_observation"]["actual_q"] == list(ZERO)
    assert finished["completion_state"] == "NOT_COMPLETED"
    assert len(finished["samples"]) == 1

    rejected = DigitalTwinMonitor(model, monitor_config)
    rejected.record_rejection("E-fixed-1")
    rejection = rejected.finish(())
    assert rejection["status"] == "REJECTED"
    assert rejection["tracking_started"] is False
    assert rejection["samples"] == []

    broken = DigitalTwinMonitor(model, monitor_config)
    broken.begin("nan")
    broken.observe((math.nan, 0.0, 0.0, 0.0, 0.0, 0.0), ZERO, "RUNNING", "EXECUTING", 0.0)
    assert broken.status == "ERROR"
    assert broken.samples == ()
    assert broken.finish(())["non_finite_rejected"] is True
    assert broken.settled is False


def test_repeated_samples_match_exactly(model, monitor_config):
    left = _synthetic(model, monitor_config)
    right = _synthetic(model, monitor_config)
    assert json.dumps(left, sort_keys=True) == json.dumps(right, sort_keys=True)


def test_monitor_period_stays_on_the_project_clock(monitor_config, tmp_path: Path):
    assert monitor_config["sample_period_sec"] == 0.01
    assert monitor_config["settling_consecutive_samples"] >= 2
    assert monitor_config["joint_settling_tolerance_rad"] == COMPLETION_TOLERANCE_RAD
    payload = dict(monitor_config)
    payload["settling_consecutive_samples"] = 1
    path = tmp_path / "monitor.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError):
        load_monitor_config(path)


def test_monitor_is_outside_the_validator_and_controller():
    for relative in (
        "controller/virtual_pmac.py",
        "validator/validate.py",
        "validator/gate.py",
        "ros2_adapter/node.py",
    ):
        assert "digital_twin" not in (ROOT / relative).read_text(encoding="utf-8")
    session = (ROOT / "digital_twin" / "session.py").read_text(encoding="utf-8")
    assert "ExecutionGate" in session
    assert ".submit(" not in session
    metrics = (ROOT / "digital_twin" / "metrics.py").read_text(encoding="utf-8")
    assert "orientation_error_rad" in metrics
    assert "forward_kinematics" in metrics


def test_designated_cases_track_and_known_collision_does_not(model):
    resolved = resolve_workcell()
    validator_config = load_validator_config()
    safe = [
        run_case(name, model, resolved, validator_config)
        for name in DESIGNATED_SAFE
    ]
    by_name = {row["case"]: row for row in safe}
    assert [row["validation_decision"] for row in safe] == ["SAFE_TO_ATTEMPT"] * 4
    assert all(row["completed"] and row["settled"] and row["tracking_started"] and not row["timed_out"] for row in safe)
    zero = by_name["A-zero"]
    assert all(sample["target_q"] == sample["actual_q"] for sample in zero["samples"])
    assert zero["metrics"]["completion_time_s"] < zero["metrics"]["settled_time_s"]
    for name in ("B-positive-1", "C-mixed-1", "E-fixed-3"):
        row = by_name[name]
        assert row["samples"][0]["target_q"] != row["samples"][0]["actual_q"]
        assert any(sample["target_q"] != sample["actual_q"] for sample in row["samples"][1:-1])
        assert row["metrics"]["final_max_joint_error_rad"] <= 1.0e-9
        assert row["metrics"]["completion_time_s"] < row["metrics"]["settled_time_s"]
        assert row["controller_trace"] == ["ACCEPTED", "EXECUTING", "COMPLETED"]
    rejected = run_case("E-fixed-1", model, resolved, validator_config)
    assert rejected["validation_decision"] == "REJECT"
    assert "COLLISION" in rejected["blocking_reason_codes"]
    assert rejected["controller_submitted"] is False
    assert rejected["submit_count_after"] == rejected["submit_count_before"]
    assert rejected["tracking_started"] is False
    assert rejected["execution_trace_created"] is False
    assert rejected["samples"] == []
    fault = run_case("B-positive-1", model, resolved, validator_config, fault_after_steps=1)
    assert fault["tracking_started"] is True
    assert fault["fault_occurred"] is True
    assert fault["completed"] is False
    assert fault["settled"] is False
    assert fault["status"] == "ERROR"
    assert fault["fault_observation"]["actual_q"] == fault["samples"][-1]["actual_q"]
    assert fault["samples"][-1]["target_q"] != fault["samples"][-1]["actual_q"]


def _synthetic(model, config) -> dict:
    monitor = DigitalTwinMonitor(model, config)
    monitor.begin("same")
    target = (0.2, 0.0, 0.0, 0.0, 0.0, 0.0)
    monitor.observe(target, ZERO, "RUNNING", "ACCEPTED", 0.0)
    monitor.observe(target, (0.1, 0.0, 0.0, 0.0, 0.0, 0.0), "RUNNING", "EXECUTING", 0.01)
    return monitor.finish(("ACCEPTED", "EXECUTING"))


def _yaw(degrees: float) -> dict[str, float]:
    half = math.radians(degrees) * 0.5
    return {"x": 0.0, "y": 0.0, "z": math.sin(half), "w": math.cos(half)}
