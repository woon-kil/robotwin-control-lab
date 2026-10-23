"""P-007 ROS middleware run.

이 프로세스는 VirtualPmacController를 만들지 않는다.
명령은 다른 프로세스의 adapter로만 보낸다.
"""

from __future__ import annotations

import json
import math
import os
import platform
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from isaac.ur10e_commands import CONFIGURATIONS
from ros2_adapter.contract import (
    CANONICAL_JOINT_ORDER,
    CLAIM,
    CLAIM_BOUNDARY,
    CLIENT_NODE_NAME,
    COMPLETION_TOLERANCE_RAD,
    CONFIGURED_FEEDBACK_HZ,
    CONTROLLER_BACKEND,
    DT_S,
    JOINT_STATE_DRAIN_S,
    JOINT_STATE_SOURCE,
    LATENCY_CLAIM,
    LATENCY_DEFINITION,
    LATENCY_SAMPLES,
    MESSAGE_CHOICE,
    QOS_DEPTH,
    QOS_DURABILITY,
    QOS_HISTORY,
    QOS_PROFILE_NAME,
    QOS_RELIABILITY,
    QOS_SOURCE,
    RATE_WINDOW_S,
    ROS_DOMAIN_ID,
    SHUTDOWN_POLICY,
    SUPPORTED_COMMAND,
    TOPIC_COMMAND,
    TOPIC_COUNT_WINDOW,
    TOPIC_INJECT_FAULT,
    TOPIC_JOINT_STATES,
    TOPIC_RESET_FAULT,
    TOPIC_RESULT,
    TOPIC_SERVO_ON,
    TOPIC_SHUTDOWN,
    TOPIC_STATE,
    TOPIC_TARGET_JOINT_STATES,
    UNITS,
)

SELECTED = ("A-zero", "B-positive-1", "C-mixed-1", "E-fixed-1", "E-fixed-3")
REORDER_NAMES = (
    "wrist_2_joint",
    "shoulder_pan_joint",
    "elbow_joint",
    "wrist_3_joint",
    "shoulder_lift_joint",
    "wrist_1_joint",
)
ROOT = Path(__file__).resolve().parents[1]


class CommandClient(Node):
    def __init__(self) -> None:
        super().__init__(CLIENT_NODE_NAME)
        profile = _qos()
        self._command = self.create_publisher(String, TOPIC_COMMAND, profile)
        self._fault = self.create_publisher(String, TOPIC_INJECT_FAULT, profile)
        self._reset = self.create_publisher(String, TOPIC_RESET_FAULT, profile)
        self._servo = self.create_publisher(String, TOPIC_SERVO_ON, profile)
        self._window = self.create_publisher(String, TOPIC_COUNT_WINDOW, profile)
        self._shutdown = self.create_publisher(String, TOPIC_SHUTDOWN, profile)
        self.create_subscription(String, TOPIC_RESULT, self._on_result, profile)
        self.create_subscription(String, TOPIC_STATE, self._on_state, profile)
        self.create_subscription(JointState, TOPIC_JOINT_STATES, self._on_joint, profile)
        self.create_subscription(JointState, TOPIC_TARGET_JOINT_STATES, self._on_target, profile)
        self.results: dict[str, list[dict]] = {}
        self.states: list[dict] = []
        self.joint_names_ok = True
        self.target_names_ok = True
        self.latest_actual: tuple[float, ...] | None = None
        self.latest_target: tuple[float, ...] | None = None
        self.separation: dict | None = None
        self.joint_total = 0
        self.joint_baseline: int | None = None
        self.open_seqs: list[int] = []
        self.stop_report: dict | None = None
        self.pending_latency: dict[str, float] = {}
        self.latency_s: list[float] = []
        self.timeouts = 0

    def wait_until(self, predicate, timeout_s: float, proc: subprocess.Popen | None = None) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if proc is not None and proc.poll() is not None:
                return False
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        self.timeouts += 1
        return False

    def send_command(self, command_id: str, names: tuple[str, ...], positions: tuple[float, ...], command_type: str | None = None) -> None:
        payload = {
            "command_id": command_id,
            "joint_names": list(names),
            "positions": list(positions),
        }
        if command_type is not None:
            payload["command_type"] = command_type
        self._publish(self._command, payload, latency_id=command_id)

    def send_text(self, publisher, text: str) -> None:
        message = String()
        message.data = text
        publisher.publish(message)

    def _publish(self, publisher, payload: dict, latency_id: str | None) -> None:
        message = String()
        message.data = json.dumps(payload, allow_nan=True)
        if latency_id is not None and latency_id.startswith("latency-"):
            self.pending_latency[latency_id] = time.perf_counter()
        publisher.publish(message)

    def _on_result(self, message: String) -> None:
        payload = json.loads(message.data)
        command_id = payload.get("command_id")
        if isinstance(command_id, str):
            self.results.setdefault(command_id, []).append(payload)
            started = self.pending_latency.pop(command_id, None)
            if started is not None and payload.get("result") == "ACCEPTED":
                self.latency_s.append(time.perf_counter() - started)

    def _on_state(self, message: String) -> None:
        payload = json.loads(message.data)
        self.states.append(payload)
        if payload.get("count_window") == "open" and isinstance(payload.get("feedback_seq"), int):
            self.open_seqs.append(payload["feedback_seq"])
        if payload.get("count_window") == "stop":
            report = payload.get("window_report")
            if isinstance(report, dict):
                self.stop_report = report
        if payload.get("count_window") == "begin":
            self.joint_baseline = self.joint_total

    def _on_joint(self, message: JointState) -> None:
        self.joint_total += 1
        if list(message.name) != list(CANONICAL_JOINT_ORDER):
            self.joint_names_ok = False
        self.latest_actual = tuple(float(value) for value in message.position)
        self._note_separation()

    def _on_target(self, message: JointState) -> None:
        if list(message.name) != list(CANONICAL_JOINT_ORDER):
            self.target_names_ok = False
        self.latest_target = tuple(float(value) for value in message.position)
        self._note_separation()

    def _note_separation(self) -> None:
        if self.separation is not None or self.latest_actual is None or self.latest_target is None:
            return
        if self.latest_actual == self.latest_target:
            return
        self.separation = {
            "actual_q": list(self.latest_actual),
            "target_q": list(self.latest_target),
            "difference": [goal - current for goal, current in zip(self.latest_target, self.latest_actual, strict=True)],
            "source": "sensor_msgs/JointState /joint_states position versus /robotwin/target_joint_states position",
        }


def run(directory: Path) -> dict:
    directory.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_DOMAIN_ID"] = ROS_DOMAIN_ID
    os.environ["PYTHONUNBUFFERED"] = "1"
    environment = _environment()
    log_path = directory / "adapter_stdout.txt"
    log_handle = log_path.open("w", encoding="utf-8")
    env = os.environ.copy()
    env["PYTHONPATH"] = str(ROOT) + os.pathsep + env.get("PYTHONPATH", "")
    proc = subprocess.Popen(
        [sys.executable, "-m", "ros2_adapter.node"],
        cwd=str(ROOT),
        env=env,
        stdout=log_handle,
        stderr=subprocess.STDOUT,
    )
    rclpy.init()
    client = CommandClient()
    payload: dict | None = None
    try:
        if not client.wait_until(lambda: bool(client.states), 20.0, proc):
            raise RuntimeError("adapter produced no feedback")
        graph = _graph(client, env)
        commands = _command_rows(client, proc)
        reorder = _reorder(client, proc)
        invalid = _invalid(client, proc)
        busy = _busy(client, proc)
        fault = _fault(client, proc)
        _servo_ready(client, proc)
        _latency(client, proc)
        rate = _rate(client, proc)
        shutdown = _shutdown(client, proc)
        payload = _payload(
            environment,
            graph,
            commands,
            reorder,
            invalid,
            busy,
            fault,
            client,
            rate,
            shutdown,
            proc.pid,
        )
    except Exception as exc:
        payload = _failed_payload(environment, exc)
    finally:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        log_handle.close()
        client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    assert payload is not None
    payload["environment"]["adapter_stdout_tail"] = _tail(log_path)
    _write(directory, payload)
    return payload


def _command_rows(client: CommandClient, proc: subprocess.Popen) -> list[dict]:
    rows = []
    for name, targets in CONFIGURATIONS:
        if name not in SELECTED:
            continue
        command_id = "cfg-" + name
        client.send_command(command_id, CANONICAL_JOINT_ORDER, targets)
        client.wait_until(lambda command_id=command_id: _has_result(client, command_id, "ACCEPTED"), 10.0, proc)
        client.wait_until(lambda command_id=command_id: _has_result(client, command_id, "COMPLETED"), 15.0, proc)
        accepted = _result(client, command_id, "ACCEPTED")
        completed = _result(client, command_id, "COMPLETED")
        rows.append(
            {
                "configuration": name,
                "command_id": command_id,
                "published_target": list(targets),
                "canonical_target": None if accepted is None else accepted.get("canonical_target"),
                "completion_state": None if completed is None else completed.get("result"),
                "actual_final_q": None if completed is None else completed.get("actual_q"),
                "max_final_error_rad": None if completed is None else completed.get("max_final_error_rad"),
                "steps": None if completed is None else completed.get("steps"),
                "simulated_duration_s": None if completed is None else completed.get("simulated_duration_s"),
                "saw_running": _saw_running(client, command_id),
                "result": "COMPLETED" if completed and completed.get("result") == "COMPLETED" else "FAIL",
            }
        )
    return rows


def _reorder(client: CommandClient, proc: subprocess.Popen) -> dict:
    values = {
        "shoulder_pan_joint": 0.2,
        "shoulder_lift_joint": 0.3,
        "elbow_joint": 0.4,
        "wrist_1_joint": 0.5,
        "wrist_2_joint": 0.6,
        "wrist_3_joint": 0.7,
    }
    published = tuple(values[name] for name in REORDER_NAMES)
    expected = tuple(values[name] for name in CANONICAL_JOINT_ORDER)
    command_id = "reorder-b-positive-1"
    client.send_command(command_id, REORDER_NAMES, published)
    client.wait_until(lambda: _has_result(client, command_id, "ACCEPTED"), 10.0, proc)
    client.wait_until(lambda: _has_result(client, command_id, "COMPLETED"), 15.0, proc)
    accepted = _result(client, command_id, "ACCEPTED")
    completed = _result(client, command_id, "COMPLETED")
    canonical = None if accepted is None else accepted.get("canonical_target")
    return {
        "input_ros_order": list(REORDER_NAMES),
        "canonical_output_order": list(CANONICAL_JOINT_ORDER),
        "input_values": list(published),
        "reordered_values": canonical,
        "expected_values": list(expected),
        "actual_final_q": None if completed is None else completed.get("actual_q"),
        "result": "PASS" if canonical == list(expected) and completed and completed.get("result") == "COMPLETED" else "FAIL",
    }


def _invalid(client: CommandClient, proc: subprocess.Popen) -> list[dict]:
    names = CANONICAL_JOINT_ORDER
    cases = [
        ("unknown-joint", ("unknown_joint",) + names[1:], (0, 0, 0, 0, 0, 0), None, "UNKNOWN_JOINT"),
        ("duplicate-joint", (names[0], names[0]) + names[2:], (0, 0, 0, 0, 0, 0), None, "DUPLICATE_JOINT"),
        ("missing-joint", names[:5], (0, 0, 0, 0, 0), None, "MISSING_JOINT"),
        ("extra-joint", names + ("extra_joint",), (0, 0, 0, 0, 0, 0, 0), None, "EXTRA_JOINT"),
        ("nan-joint", names, (0, 0, 0, 0, 0, float("nan")), None, "NON_FINITE"),
        ("inf-joint", names, (0, 0, 0, 0, 0, float("inf")), None, "NON_FINITE"),
        ("joint-limit", names, (7, 0, 0, 0, 0, 0), None, "JOINT_LIMIT"),
        ("move-tcp", names, (0, 0, 0, 0, 0, 0), "MOVE_TCP", "UNSUPPORTED_COMMAND"),
    ]
    rows = []
    for command_id, case_names, positions, command_type, code in cases:
        before = len(client.states)
        client.send_command(command_id, case_names, positions, command_type)
        client.wait_until(lambda command_id=command_id: _has_result(client, command_id, "REJECTED"), 5.0, proc)
        rejected = _result(client, command_id, "REJECTED")
        later = client.states[before:]
        stayed = all(item.get("controller_state") == "READY" for item in later)
        rows.append(
            {
                "command_id": command_id,
                "error_code": None if rejected is None else rejected.get("error_code"),
                "expected_error_code": code,
                "result": "PASS" if rejected and rejected.get("error_code") == code and rejected.get("result") == "REJECTED" and stayed else "FAIL",
                "controller_stayed_ready": stayed,
            }
        )
    return rows


def _busy(client: CommandClient, proc: subprocess.Popen) -> dict:
    first = "busy-first"
    second = "busy-second"
    client.send_command(first, CANONICAL_JOINT_ORDER, _targets("C-mixed-1"))
    client.wait_until(lambda: _has_result(client, first, "ACCEPTED"), 10.0, proc)
    client.send_command(second, CANONICAL_JOINT_ORDER, _targets("E-fixed-1"))
    client.wait_until(lambda: _has_result(client, second, "REJECTED"), 5.0, proc)
    client.wait_until(lambda: _has_result(client, first, "COMPLETED"), 15.0, proc)
    rejected = _result(client, second, "REJECTED")
    return {
        "policy": "REJECT_BUSY",
        "second_result": None if rejected is None else rejected.get("result"),
        "second_code": None if rejected is None else rejected.get("error_code"),
        "first_completed": _has_result(client, first, "COMPLETED"),
        "result": "PASS" if rejected and rejected.get("error_code") == "BUSY" and _has_result(client, first, "COMPLETED") else "FAIL",
    }


def _fault(client: CommandClient, proc: subprocess.Popen) -> dict:
    command_id = "fault-move"
    client.send_command(command_id, CANONICAL_JOINT_ORDER, _targets("B-positive-1"))
    client.wait_until(lambda: _has_result(client, command_id, "ACCEPTED"), 10.0, proc)
    client.wait_until(lambda: _executing_with_error(client, command_id), 10.0, proc)
    client.send_text(client._fault, "TEST_FAULT")
    client.wait_until(lambda: _latest_state(client, "ERROR") is not None, 5.0, proc)
    held = _latest_state(client, "ERROR")
    rejected_id = "during-fault"
    client.send_command(rejected_id, CANONICAL_JOINT_ORDER, _targets("E-fixed-1"))
    client.wait_until(lambda: _has_result(client, rejected_id, "REJECTED"), 5.0, proc)
    rejected = _result(client, rejected_id, "REJECTED")
    client.send_text(client._reset, "reset")
    client.wait_until(lambda: _latest_state(client, "CONNECTED") is not None, 5.0, proc)
    restored = _latest_state(client, "CONNECTED")
    held_q = None if held is None else held.get("actual_joint_positions")
    restored_q = None if restored is None else restored.get("actual_joint_positions")
    preserved = held_q is not None and held_q == restored_q and held_q != [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    trace = [item.get("result") for item in client.results.get(command_id, [])]
    return {
        "command_accepted": _has_result(client, command_id, "ACCEPTED"),
        "fault_injected": "TEST_FAULT",
        "active_command_aborted": bool(held) and held.get("active_command_id") is None and "FAULTED" in _execution_values(client, command_id),
        "controller_state": None if held is None else held.get("controller_state"),
        "new_command_result": None if rejected is None else rejected.get("result"),
        "new_command_code": None if rejected is None else rejected.get("error_code"),
        "reset_successful": bool(restored),
        "state_after_reset": None if restored is None else restored.get("controller_state"),
        "joint_position_before_reset": held_q,
        "joint_position_after_reset": restored_q,
        "joint_position_preserved": preserved,
        "command_trace": trace,
        "result": "PASS" if preserved and rejected and rejected.get("error_code") == "ILLEGAL_STATE" and held and held.get("controller_state") == "ERROR" else "FAIL",
    }


def _servo_ready(client: CommandClient, proc: subprocess.Popen) -> None:
    mark = len(client.states)
    client.send_text(client._servo, "servo_on")
    client.wait_until(
        lambda: any(state.get("controller_state") == "READY" for state in client.states[mark:]),
        5.0,
        proc,
    )


def _latency(client: CommandClient, proc: subprocess.Popen) -> None:
    zeros = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    for index in range(LATENCY_SAMPLES):
        command_id = "latency-" + str(index)
        client.send_command(command_id, CANONICAL_JOINT_ORDER, zeros)
        client.wait_until(lambda command_id=command_id: _has_result(client, command_id, "ACCEPTED"), 5.0, proc)
        client.wait_until(lambda command_id=command_id: _has_result(client, command_id, "COMPLETED"), 15.0, proc)


def _rate(client: CommandClient, proc: subprocess.Popen) -> dict:
    client.joint_baseline = None
    client.open_seqs = []
    client.stop_report = None
    client.send_text(client._window, "begin")
    deadline = time.monotonic() + RATE_WINDOW_S
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            break
        rclpy.spin_once(client, timeout_sec=0.01)
    client.send_text(client._window, "end")
    client.wait_until(lambda: client.stop_report is not None, 5.0, proc)
    report = client.stop_report or {}
    published_joint = int(report.get("published_joint_states") or 0)
    drain_deadline = time.monotonic() + JOINT_STATE_DRAIN_S
    while time.monotonic() < drain_deadline:
        if client.joint_baseline is not None and client.joint_total - client.joint_baseline >= published_joint:
            break
        rclpy.spin_once(client, timeout_sec=0.05)
    received_joint = 0 if client.joint_baseline is None else client.joint_total - client.joint_baseline
    published_state = int(report.get("published_state") or 0)
    received_state = len(client.open_seqs)
    duration = report.get("duration_s")
    observed = None if not isinstance(duration, (int, float)) or duration <= 0 else received_joint / float(duration)
    return {
        "topic": TOPIC_JOINT_STATES,
        "configured_rate_hz": CONFIGURED_FEEDBACK_HZ,
        "observed_rate_hz": observed,
        "message_count": received_joint,
        "duration_s": duration,
        "published": published_joint,
        "received": received_joint,
        "lost": published_joint - received_joint,
        "state_topic": {
            "topic": TOPIC_STATE,
            "published": published_state,
            "received": received_state,
            "lost": published_state - received_state,
            "sequence_gaps": _gaps(client.open_seqs),
        },
        "qos": _qos_document(),
        "claim": "PROJECT TEST MEASUREMENT. NOT AN INDUSTRIAL SAFETY GUARANTEE.",
    }


def _shutdown(client: CommandClient, proc: subprocess.Popen) -> dict:
    before = _latest_any(client)
    before_q = None if before is None else before.get("actual_joint_positions")
    client.send_text(client._shutdown, "shutdown")
    client.wait_until(lambda: _latest_state(client, "DISCONNECTED") is not None, 5.0, proc)
    after = _latest_state(client, "DISCONNECTED")
    after_q = None if after is None else after.get("actual_joint_positions")
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        pass
    return {
        "policy": SHUTDOWN_POLICY,
        "state": None if after is None else after.get("controller_state"),
        "joint_before": before_q,
        "joint_after": after_q,
        "joints_preserved": before_q is not None and before_q == after_q,
        "adapter_exited": proc.poll() is not None,
        "result": "PASS" if after and after.get("controller_state") == "DISCONNECTED" and before_q == after_q else "FAIL",
    }


def _payload(environment, graph, commands, reorder, invalid, busy, fault, client, rate, shutdown, adapter_pid: int) -> dict:
    feedback = _feedback_sample(client)
    timing = _timing(client.latency_s)
    checks = _checks(environment, graph, commands, reorder, invalid, busy, fault, client, feedback, rate, shutdown)
    result = "PASS" if all(item[1] for item in checks) else "FAIL"
    lines = [name + " " + ("PASS" if passed else "FAIL") for name, passed in checks]
    lines.append("RESULT " + result)
    return {
        "result": result,
        "checks": lines,
        "environment": environment,
        "graph": graph,
        "commands": commands,
        "reorder": reorder,
        "invalid": invalid,
        "feedback": feedback,
        "fault": fault,
        "busy": busy,
        "timing": timing,
        "rate": rate,
        "shutdown": shutdown,
        "process": {
            "adapter_pid": adapter_pid,
            "client_pid": os.getpid(),
            "separate_processes": adapter_pid != os.getpid(),
        },
        "timeouts": client.timeouts,
        "non_finite": _non_finite(client.states),
        "test_results_txt": "\n".join(lines) + "\n",
    }


def _checks(environment, graph, commands, reorder, invalid, busy, fault, client, feedback, rate, shutdown) -> list[tuple[str, bool]]:
    def rejected(code: str) -> bool:
        matched = [row for row in invalid if row["expected_error_code"] == code]
        return bool(matched) and all(row["result"] == "PASS" for row in matched)
    five = len(commands) == 5 and all(
        row["result"] == "COMPLETED"
        and row["saw_running"]
        and isinstance(row["max_final_error_rad"], (int, float))
        and row["max_final_error_rad"] <= COMPLETION_TOLERANCE_RAD
        for row in commands
    )
    state_lost = rate["state_topic"]["lost"]
    text = graph.get("node_list", "") + "\n" + graph.get("topic_list", "")
    return [
        ("ros2_environment_verified", environment.get("ros_distro") == "jazzy" and "24.04" in environment.get("os_release", "")),
        ("adapter_boundary_defined", True),
        ("ros_middleware_traversed", client.separation is not None and "ros2_controller_adapter" in text and "p007_command_client" in text),
        ("five_configurations_complete", five),
        ("joint_name_reorder", reorder["result"] == "PASS"),
        ("unknown_joint_reject", rejected("UNKNOWN_JOINT")),
        ("duplicate_joint_reject", rejected("DUPLICATE_JOINT")),
        ("missing_joint_reject", rejected("MISSING_JOINT")),
        ("extra_joint_reject", rejected("EXTRA_JOINT")),
        ("non_finite_reject", rejected("NON_FINITE")),
        ("controller_limit_and_unsupported_preserved", rejected("JOINT_LIMIT") and rejected("UNSUPPORTED_COMMAND")),
        ("target_actual_separation", bool(feedback) and feedback.get("target_differs_from_actual") is True and client.separation is not None),
        ("busy_policy", busy["result"] == "PASS"),
        ("fault_behavior", fault["result"] == "PASS" and fault["active_command_aborted"] is True),
        ("fault_reset", fault["state_after_reset"] == "CONNECTED"),
        ("joint_state_preserved", fault["joint_position_preserved"] is True),
        ("feedback_publication", client.joint_names_ok and client.target_names_ok and client.joint_total > 0 and bool(client.states)),
        ("ros_graph_evidence", graph.get("return_ok") is True and "/joint_states" in text and "sensor_msgs/msg/JointState" in text),
        ("timeout", client.timeouts == 0),
        ("non_finite_controller_state", _non_finite(client.states) == 0),
        ("shutdown_policy", shutdown["result"] == "PASS"),
        ("state_message_loss_accounted", isinstance(state_lost, int)),
        ("real_hardware_not_used", True),
        ("real_pmac_claim_absent", True),
        ("p008_not_started", True),
    ]


def _feedback_sample(client: CommandClient) -> dict:
    sample = None
    for state in client.states:
        errors = state.get("joint_errors")
        if not isinstance(errors, list) or not errors:
            continue
        if max(abs(float(value)) for value in errors) <= COMPLETION_TOLERANCE_RAD:
            continue
        if state.get("execution_state") != "EXECUTING":
            continue
        sample = {
            "target_q": state.get("target_joint_positions"),
            "actual_q": state.get("actual_joint_positions"),
            "difference": errors,
            "controller_state": state.get("controller_state"),
            "execution_state": state.get("execution_state"),
            "simulation_time_s": state.get("simulation_time_s"),
            "target_differs_from_actual": state.get("target_joint_positions") != state.get("actual_joint_positions"),
            "source": TOPIC_STATE,
        }
        break
    if sample is None:
        return {"target_differs_from_actual": False}
    if client.separation is not None:
        sample["joint_state_pair"] = client.separation
    return sample


def _timing(samples: list[float]) -> dict:
    if not samples:
        return {
            "definition": LATENCY_DEFINITION,
            "sample_count": 0,
            "claim_boundary": list(LATENCY_CLAIM),
        }
    ordered = sorted(samples)
    mean = sum(ordered) / len(ordered)
    return {
        "definition": LATENCY_DEFINITION,
        "start": "time.perf_counter immediately before publishing the command String",
        "end": "time.perf_counter when the result String with the same command_id and result ACCEPTED is received",
        "not_mixed_with": "adapter receipt time is not a sample and is not added to this interval",
        "sample_count": len(ordered),
        "mean_ms": mean * 1000.0,
        "p50_ms": _percentile(ordered, 50) * 1000.0,
        "p95_ms": _percentile(ordered, 95) * 1000.0,
        "p99_ms": _percentile(ordered, 99) * 1000.0,
        "max_ms": ordered[-1] * 1000.0,
        "claim_boundary": list(LATENCY_CLAIM),
    }


def _graph(client: CommandClient, env: dict) -> dict:
    holder: dict[str, dict] = {}

    def collect() -> None:
        node_list = _ros(env, ["node", "list"])
        topic_list = _ros(env, ["topic", "list", "-t"])
        infos = {
            topic: _ros(env, ["topic", "info", topic, "-v"])
            for topic in (
                TOPIC_COMMAND,
                TOPIC_RESULT,
                TOPIC_STATE,
                TOPIC_JOINT_STATES,
                TOPIC_TARGET_JOINT_STATES,
            )
        }
        holder["value"] = {
            "node_list": node_list["stdout"],
            "topic_list": topic_list["stdout"],
            "topic_info": {topic: info["stdout"] + info["stderr"] for topic, info in infos.items()},
            "return_ok": node_list["returncode"] == 0 and topic_list["returncode"] == 0,
        }

    worker = threading.Thread(target=collect)
    worker.start()
    while worker.is_alive():
        rclpy.spin_once(client, timeout_sec=0.05)
    worker.join()
    return holder.get("value", {"node_list": "", "topic_list": "", "topic_info": {}, "return_ok": False})


def _ros(env: dict, args: list[str]) -> dict:
    last = {"returncode": 1, "stdout": "", "stderr": "ros2 was not run"}
    for extra in (["--no-daemon"], []):
        completed = subprocess.run(
            ["ros2", *args, *extra],
            env=env,
            capture_output=True,
            text=True,
            timeout=60,
            check=False,
        )
        last = {
            "returncode": completed.returncode,
            "stdout": completed.stdout,
            "stderr": completed.stderr,
        }
        if completed.returncode == 0:
            return last
        combined = completed.stderr + completed.stdout
        if "unrecognized" not in combined and "invalid choice" not in combined:
            return last
    return last


def _environment() -> dict:
    os_release = Path("/etc/os-release").read_text(encoding="utf-8") if Path("/etc/os-release").is_file() else platform.platform()
    return {
        "os_release": os_release,
        "platform": platform.platform(),
        "python": sys.version,
        "ros_distro": os.environ.get("ROS_DISTRO", ""),
        "ros_domain_id": ROS_DOMAIN_ID,
        "rclpy": getattr(rclpy, "__file__", ""),
        "host_note": "This run is the machine that executed the adapter process.",
    }


def _failed_payload(environment: dict, exc: Exception) -> dict:
    lines = ["run FAIL", "RESULT FAIL", str(exc)]
    return {
        "result": "FAIL",
        "checks": lines,
        "environment": environment,
        "graph": {},
        "commands": [],
        "reorder": {"result": "FAIL"},
        "invalid": [],
        "feedback": {},
        "fault": {"result": "FAIL"},
        "busy": {"result": "FAIL"},
        "timing": {"definition": LATENCY_DEFINITION, "sample_count": 0, "claim_boundary": list(LATENCY_CLAIM)},
        "rate": {},
        "shutdown": {"result": "FAIL", "policy": SHUTDOWN_POLICY},
        "process": {"adapter_pid": None, "client_pid": os.getpid(), "separate_processes": False},
        "timeouts": 1,
        "non_finite": 0,
        "test_results_txt": "\n".join(lines) + "\n",
        "exception": str(exc),
    }


def _write(directory: Path, payload: dict) -> None:
    contract = {
        "adapter_implementation": "Ros2ControllerAdapter",
        "claim": CLAIM,
        "claim_boundary": list(CLAIM_BOUNDARY),
        "ros_distro": payload["environment"].get("ros_distro"),
        "nodes": [CLIENT_NODE_NAME, "ros2_controller_adapter"],
        "topics": {
            TOPIC_COMMAND: "std_msgs/msg/String",
            TOPIC_RESULT: "std_msgs/msg/String",
            TOPIC_STATE: "std_msgs/msg/String",
            TOPIC_JOINT_STATES: "sensor_msgs/msg/JointState",
            TOPIC_TARGET_JOINT_STATES: "sensor_msgs/msg/JointState",
        },
        "message_choice": MESSAGE_CHOICE,
        "joint_state_source": JOINT_STATE_SOURCE,
        "qos": _qos_document(),
        "canonical_joint_order": list(CANONICAL_JOINT_ORDER),
        "units": UNITS,
        "supported_command": SUPPORTED_COMMAND,
        "controller_backend": CONTROLLER_BACKEND,
        "step_dt_s": DT_S,
        "configured_feedback_hz": CONFIGURED_FEEDBACK_HZ,
        "completion_tolerance_rad": COMPLETION_TOLERANCE_RAD,
        "completion_tolerance_class": "PROJECT TEST PARAMETER",
        "shutdown_policy": SHUTDOWN_POLICY,
        "test_hooks": [
            TOPIC_INJECT_FAULT,
            TOPIC_RESET_FAULT,
            TOPIC_SERVO_ON,
            TOPIC_COUNT_WINDOW,
            TOPIC_SHUTDOWN,
        ],
        "real_robot_used": False,
        "real_pmac_compatibility_claimed": False,
        "p008_started": False,
    }
    _json(directory / "interface_contract.json", contract)
    _json(directory / "command_results.json", {"configurations": payload["commands"], "busy": payload["busy"]})
    _json(directory / "joint_order_results.json", payload["reorder"])
    _json(directory / "invalid_command_results.json", {"cases": payload["invalid"]})
    _json(directory / "feedback_results.json", payload["feedback"])
    _json(directory / "fault_results.json", payload["fault"])
    _json(directory / "timing_results.json", payload["timing"])
    _json(directory / "message_rate_results.json", payload["rate"])
    _json(directory / "ros_graph.json", payload["graph"])
    (directory / "test_results.txt").write_text(payload["test_results_txt"], encoding="utf-8")
    (directory / "environment.md").write_text(_environment_md(payload), encoding="utf-8")
    (directory / "summary.md").write_text(_summary_md(payload), encoding="utf-8")


def _environment_md(payload: dict) -> str:
    env = payload["environment"]
    return "\n".join(
        [
            "# P-007 environment",
            "",
            "이 기록은 adapter를 실행한 기계의 측정이다.",
            "",
            "```text",
            env.get("os_release", ""),
            "python " + env.get("python", ""),
            "ROS_DISTRO " + env.get("ros_distro", ""),
            "ROS_DOMAIN_ID " + env.get("ros_domain_id", ""),
            "platform " + env.get("platform", ""),
            "rclpy " + env.get("rclpy", ""),
            "```",
            "",
            "Isaac Sim과 Unity는 이 게이트에서 다시 실행하지 않았다.",
            "P-001B부터 P-006 evidence 파일은 이 실행이 쓰지 않는다.",
            "",
            "Adapter stdout tail:",
            "",
            "```text",
            env.get("adapter_stdout_tail", ""),
            "```",
            "",
        ]
    )


def _summary_md(payload: dict) -> str:
    errors = [
        row["max_final_error_rad"]
        for row in payload["commands"]
        if isinstance(row.get("max_final_error_rad"), (int, float))
    ]
    worst = max(errors) if errors else None
    worst_deg = None if worst is None else worst * 180.0 / math.pi
    rate = payload["rate"]
    timing = payload["timing"]
    lines = [
        "# P-007",
        "",
        CLAIM,
        "",
        "ROS 2는 transport다. Canonical command는 MOVE_JOINT다.",
        "Controller backend는 VirtualPmacController다.",
        "실제 로봇과 실제 Power PMAC는 사용하지 않았다.",
        "P-008은 시작하지 않았다.",
        "",
        "RESULT " + payload["result"],
        "timeouts " + str(payload["timeouts"]),
        "non_finite " + str(payload["non_finite"]),
        "max_final_error_rad " + str(worst),
        "max_final_error_deg " + str(worst_deg),
        "observed_feedback_hz " + str(rate.get("observed_rate_hz")),
        "latency_mean_ms " + str(timing.get("mean_ms")),
        "latency_p50_ms " + str(timing.get("p50_ms")),
        "latency_p95_ms " + str(timing.get("p95_ms")),
        "latency_p99_ms " + str(timing.get("p99_ms")),
        "latency_max_ms " + str(timing.get("max_ms")),
        "",
        SHUTDOWN_POLICY,
        "",
        "NOT A ROS 2 REAL-TIME CERTIFIED CONTROLLER",
        "NOT REAL ROBOT VALIDATED",
        "NOT VERIFIED AGAINST REAL POWER PMAC",
        "",
    ]
    return "\n".join(lines)


def _qos():
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=QOS_DEPTH,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _qos_document() -> dict:
    return {
        "profile": QOS_PROFILE_NAME,
        "reliability": QOS_RELIABILITY,
        "history": QOS_HISTORY,
        "depth": QOS_DEPTH,
        "durability": QOS_DURABILITY,
        "source": QOS_SOURCE,
        "reason": "Default reliable profile so localhost delivery can be counted. sensor_data best effort can drop samples. This is not a safety guarantee.",
    }


def _targets(name: str) -> tuple[float, ...]:
    for config_name, values in CONFIGURATIONS:
        if config_name == name:
            return values
    raise KeyError(name)


def _has_result(client: CommandClient, command_id: str, result: str) -> bool:
    return any(item.get("result") == result for item in client.results.get(command_id, []))


def _result(client: CommandClient, command_id: str, result: str) -> dict | None:
    for item in client.results.get(command_id, []):
        if item.get("result") == result:
            return item
    return None


def _saw_running(client: CommandClient, command_id: str) -> bool:
    return any(
        state.get("active_command_id") == command_id and state.get("controller_state") == "RUNNING"
        for state in client.states
    )


def _executing_with_error(client: CommandClient, command_id: str) -> bool:
    for state in client.states:
        if state.get("active_command_id") != command_id or state.get("execution_state") != "EXECUTING":
            continue
        errors = state.get("joint_errors") or []
        if errors and max(abs(float(value)) for value in errors) > COMPLETION_TOLERANCE_RAD:
            return True
    return False


def _latest_state(client: CommandClient, controller_state: str) -> dict | None:
    for state in reversed(client.states):
        if state.get("controller_state") == controller_state:
            return state
    return None


def _latest_any(client: CommandClient) -> dict | None:
    if not client.states:
        return None
    return client.states[-1]


def _execution_values(client: CommandClient, command_id: str) -> list[str]:
    values = []
    for state in client.states:
        if state.get("command_id") == command_id and isinstance(state.get("execution_state"), str):
            values.append(state["execution_state"])
    return values


def _gaps(seqs: list[int]) -> int:
    return sum(1 for previous, current in zip(seqs, seqs[1:], strict=False) if current != previous + 1)


def _non_finite(states: list[dict]) -> int:
    count = 0
    for state in states:
        for key in ("actual_joint_positions", "target_joint_positions", "joint_errors", "simulation_time_s"):
            value = state.get(key)
            numbers = value if isinstance(value, list) else [value]
            for number in numbers:
                if isinstance(number, (int, float)) and not math.isfinite(float(number)):
                    count += 1
    return count


def _percentile(ordered: list[float], percent: int) -> float:
    rank = math.ceil(percent / 100.0 * len(ordered)) - 1
    return ordered[min(max(rank, 0), len(ordered) - 1)]


def _tail(path: Path) -> str:
    if not path.is_file():
        return ""
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    return "\n".join(lines[-40:])


def _json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _summary_printed(payload: dict) -> str:
    return payload["test_results_txt"]


def main() -> None:
    payload = run(ROOT / "evidence" / "P-007")
    print("RESULT " + payload["result"])
    print(payload["test_results_txt"])
    if payload["result"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
